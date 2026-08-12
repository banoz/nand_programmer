#!/usr/bin/env python3
"""
General-purpose protocol client for the NANDO programmer (parallel NAND).
Parses the real chip DB CSV, auto-identifies whatever chip is in the socket,
and can do a full read-only dump. Mirrors qt/cmd.h and
firmware/programmer/nand_programmer.c exactly (see test_nando.py for the
original block-level test this was generalized from).
"""
import os
import sys
import struct
import termios
import time
import math

def find_device():
    """The NAND programmer (STM32 VID:PID 0483:5740) doesn't always
    enumerate as the same /dev/ttyACM* node -- the ST-Link's own VCP
    (0483:3752) competes for the same naming range. Find it by USB
    VID/PID instead of assuming a fixed path."""
    import glob
    import subprocess
    for node in sorted(glob.glob("/dev/ttyACM*")):
        try:
            out = subprocess.run(
                ["udevadm", "info", "-q", "property", "-n", node],
                capture_output=True, text=True, timeout=5).stdout
        except Exception:
            continue
        props = dict(line.split("=", 1) for line in out.splitlines() if "=" in line)
        if props.get("ID_VENDOR_ID") == "0483" and props.get("ID_MODEL_ID") == "5740":
            return node
    raise RuntimeError("NAND programmer (0483:5740) not found among /dev/ttyACM* devices")


DEV = find_device()
CSV_PATH = "/home/user/Documents/Nando/nand_programmer/qt/nando_parallel_chip_db.csv"

CMD_NAND_READ_ID = 0x00
CMD_NAND_ERASE = 0x01
CMD_NAND_READ = 0x02
CMD_NAND_WRITE_S = 0x03
CMD_NAND_WRITE_D = 0x04
CMD_NAND_WRITE_E = 0x05
CMD_NAND_CONF = 0x06
CMD_NAND_READ_BB = 0x07
CMD_VERSION_GET = 0x08

RESP_DATA = 0x00
RESP_STATUS = 0x01
STATUS_OK = 0x00
STATUS_ERROR = 0x01
STATUS_BB = 0x02
STATUS_WRITE_ACK = 0x03
STATUS_BB_SKIP = 0x04
STATUS_PROGRESS = 0x05

FIELDS = [
    "name", "page_size", "block_size", "total_size", "spare_size",
    "bb_mark_off", "tCS", "tCLS", "tALS", "tCLR", "tAR", "tWP", "tRP",
    "tDS", "tCH", "tCLH", "tALH", "tWC", "tRC", "tREA", "row_cycles",
    "col_cycles", "read1_cmd", "read2_cmd", "read_spare_cmd", "read_id_cmd",
    "reset_cmd", "write1_cmd", "write2_cmd", "erase1_cmd", "erase2_cmd",
    "status_cmd", "set_features_cmd", "en_ecc_addr", "en_ecc_value",
    "dis_ecc_value", "ID1", "ID2", "ID3", "ID4", "ID5",
]


def load_chip_db(path):
    chips = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            chip = dict(zip(FIELDS, parts))
            chips.append(chip)
    return chips


def opt_int(v):
    return None if v == "-" else int(v)


def opt_byte(v):
    return 0xFF if v == "-" else (int(v) & 0xFF)


def match_chip(chips, chip_id):
    id1, id2, id3, id4, id5 = chip_id
    for c in chips:
        if int(c["ID1"]) != id1 or int(c["ID2"]) != id2:
            continue
        cid3 = opt_int(c["ID3"])
        if cid3 is None:
            return c
        if cid3 != id3:
            continue
        cid4 = opt_int(c["ID4"])
        if cid4 is None:
            return c
        if cid4 != id4:
            continue
        cid5 = opt_int(c["ID5"])
        if cid5 is None:
            return c
        if cid5 != id5:
            continue
        return c
    return None


def stm_params(c):
    tHCLK = 13.88
    tsuD_NOE = 25
    tCS, tCLS, tALS, tCLR, tAR = (float(c[k]) for k in
                                   ("tCS", "tCLS", "tALS", "tCLR", "tAR"))
    tWP, tRP, tDS, tREA = (float(c[k]) for k in ("tWP", "tRP", "tDS", "tREA"))
    tCH, tCLH, tALH = (float(c[k]) for k in ("tCH", "tCLH", "tALH"))
    tWC, tRC = float(c["tWC"]), float(c["tRC"])

    setup = max(tCS, tCLS, tALS, tCLR, tAR) - tWP
    setup = setup / tHCLK - 1
    setup = 1 if setup <= 0 else math.ceil(setup)

    wait = max(tWP, tRP)
    wait = wait / tHCLK - 1
    wait = 0 if wait <= 0 else math.ceil(wait)
    wait2 = (tREA + tsuD_NOE) / tHCLK - 1
    wait2 = 0 if wait2 <= 0 else math.ceil(wait2)
    wait = max(wait, wait2)

    hiz = max(tCS, tALS, tCLS) + (tWP - tDS)
    hiz = hiz / tHCLK - 1
    hiz = 0 if hiz <= 0 else math.ceil(hiz)

    hold = max(tCH, tCLH, tALH)
    hold = hold / tHCLK - 1
    hold = 2 if hold <= 0 else math.ceil(hold)

    while ((setup + 1) + (wait + 1) + (hold + 1)) * tHCLK < max(tWC, tRC):
        setup += 1

    ar = tAR / tHCLK - 4 - setup
    ar = 0 if ar <= 0 else math.ceil(ar)

    clr = tCLR / tHCLK - 4 - setup
    clr = 0 if clr <= 0 else math.ceil(clr)

    return dict(setupTime=int(setup), waitSetupTime=int(wait),
                holdSetupTime=int(hold), hiZSetupTime=int(hiz),
                clrSetupTime=int(clr), arSetupTime=int(ar))


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
    iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP |
               termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG |
               termios.IEXTEN)
    cflag &= ~(termios.CSIZE | termios.PARENB)
    cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW,
                       [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])
    return fd


def read_exact(fd, n, timeout=90.0):
    buf = b""
    deadline = time.time() + timeout
    while len(buf) < n:
        chunk = os.read(fd, n - len(buf))
        if chunk:
            buf += chunk
        else:
            if time.time() > deadline:
                raise TimeoutError(f"timed out reading {n} bytes, got {len(buf)}: {buf!r}")
            time.sleep(0.001)
    return buf


def send(fd, data):
    written = 0
    while written < len(data):
        written += os.write(fd, data[written:])


class RespError(Exception):
    pass


def read_response(fd):
    header = read_exact(fd, 2)
    code, info = header[0], header[1]
    if code == RESP_DATA:
        payload = read_exact(fd, info) if info else b""
        return "data", payload
    elif code == RESP_STATUS:
        if info == STATUS_OK:
            return "ok", None
        elif info == STATUS_ERROR:
            rest = read_exact(fd, 12)
            err_code = struct.unpack("<b", rest[0:1])[0]
            raise RespError(f"device returned STATUS_ERROR, err_code={err_code}")
        elif info in (STATUS_BB, STATUS_BB_SKIP):
            rest = read_exact(fd, 12)
            addr, size = struct.unpack("<QI", rest)
            return ("bb_skip" if info == STATUS_BB_SKIP else "bb"), (addr, size)
        elif info == STATUS_WRITE_ACK:
            rest = read_exact(fd, 12)
            ack_bytes = struct.unpack("<Q", rest[0:8])[0]
            return "write_ack", ack_bytes
        elif info == STATUS_PROGRESS:
            rest = read_exact(fd, 8)
            progress = struct.unpack("<Q", rest)[0]
            return "progress", progress
        else:
            raise RespError(f"unknown status subtype {info}")
    else:
        raise RespError(f"unknown response code {code}")


def cmd_version(fd):
    send(fd, bytes([CMD_VERSION_GET]))
    kind, payload = read_response(fd)
    assert kind == "data"
    return struct.unpack("<BBH", payload)


def cmd_read_id(fd):
    send(fd, bytes([CMD_NAND_READ_ID]))
    kind, payload = read_response(fd)
    assert kind == "data"
    return tuple(payload)


def cmd_conf(fd, c):
    sp = stm_params(c)
    conf = struct.pack(
        "<BBIIQIB",
        CMD_NAND_CONF, 0,
        int(c["page_size"]), int(c["block_size"]), int(c["total_size"]),
        int(c["spare_size"]), int(c["bb_mark_off"]),
    )
    hal_conf = struct.pack(
        "<BBBBBBBBBBBBBBBBBBBBBB",
        sp["setupTime"], sp["waitSetupTime"], sp["holdSetupTime"],
        sp["hiZSetupTime"], sp["clrSetupTime"], sp["arSetupTime"],
        int(c["row_cycles"]), int(c["col_cycles"]),
        opt_byte(c["read1_cmd"]), opt_byte(c["read2_cmd"]),
        opt_byte(c["read_spare_cmd"]), opt_byte(c["read_id_cmd"]),
        opt_byte(c["reset_cmd"]), opt_byte(c["write1_cmd"]),
        opt_byte(c["write2_cmd"]), opt_byte(c["erase1_cmd"]),
        opt_byte(c["erase2_cmd"]), opt_byte(c["status_cmd"]),
        opt_byte(c["status_cmd"]),  # Qt app reuses status_cmd here (see log)
        opt_byte(c["en_ecc_addr"]), opt_byte(c["en_ecc_value"]),
        opt_byte(c["dis_ecc_value"]),
    )
    send(fd, conf + hal_conf)
    kind, _ = read_response(fd)
    if kind != "ok":
        raise RespError(f"CONF failed: {kind}")


def cmd_read_bb(fd):
    send(fd, bytes([CMD_NAND_READ_BB]))
    bad = []
    while True:
        kind, payload = read_response(fd)
        if kind == "ok":
            break
        elif kind == "bb":
            bad.append(payload)
        elif kind == "progress":
            pass
        else:
            raise RespError(f"unexpected response during READ_BB: {kind}")
    return bad


def cmd_read_range(fd, addr, length, skip_bb=True, on_bb=None):
    """Read [addr, addr+length) with bad-block skip; returns bytes actually
    filled (bad ranges are left as-is in a pre-sized bytearray of 0xFF)."""
    flags = 1 if skip_bb else 0  # skipBB bit0
    send(fd, struct.pack("<BQQB", CMD_NAND_READ, addr, length, flags))
    out = bytearray()
    got = 0
    while got < length:
        kind, payload = read_response(fd)
        if kind == "data":
            out += payload
            got += len(payload)
        elif kind in ("bb", "bb_skip"):
            bb_addr, bb_size = payload
            if on_bb:
                on_bb(bb_addr, bb_size)
        else:
            raise RespError(f"unexpected response during READ: {kind}")
    return bytes(out)


if __name__ == "__main__":
    chips = load_chip_db(CSV_PATH)
    fd = open_port(DEV)
    try:
        major, minor, build = cmd_version(fd)
        print(f"Firmware version: {major}.{minor}.{build}")

        # Use a conservative, already-known-working conf just to read the ID
        # (mirrors how the real host app auto-detects: conf with a generic
        # profile, read ID, then re-conf with the matched chip's real params).
        generic = chips[0]
        cmd_conf(fd, generic)
        chip_id = cmd_read_id(fd)
        id_hex = " ".join(f"{b:02x}h" for b in chip_id)
        print(f"Read ID: {id_hex}")

        match = match_chip(chips, chip_id)
        if not match:
            print("No match in chip DB -- cannot safely proceed automatically.")
            sys.exit(2)

        print(f"Matched chip: {match['name']} "
              f"({int(match['total_size'])/1024/1024:.0f} MB, "
              f"page {match['page_size']}, block {match['block_size']}, "
              f"spare {match['spare_size']})")

        cmd_conf(fd, match)
        print("Reconfigured with matched chip's parameters.")
    finally:
        os.close(fd)
