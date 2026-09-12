#!/usr/bin/env python3
"""Protocol layer for the NANDO programmer (parallel NAND over USB CDC).

Mirrors the firmware wire format exactly:

  * command / response structs -- ``qt/cmd.h`` and
    ``firmware/programmer/nand_programmer.c`` (all ``__packed__``, so every
    struct format string here is little-endian with no alignment).
  * FSMC timing derivation -- the same arithmetic the Qt host app applies
    before sending CMD_NAND_CONF.

Framing note: the firmware reads one *USB packet* per command
(``USB_Data_Peek`` -> ``np_cmd_handler``), so a command must never be split
across packets and two commands must never share one. Every command here is
<= 64 bytes and written with a single ``write(2)``, which the CDC gadget
delivers as one packet (a 64-byte write is exactly one max-size packet).

Everything in this module is synchronous and blocking. Keeping the event
loop unblocked is server.py's job.
"""
import errno
import fcntl
import math
import os
import select
import struct
import termios
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.normpath(
    os.path.join(_HERE, os.pardir, "qt", "nando_parallel_chip_db.csv"))

USB_VID = "0483"
USB_PID = "5740"

# Firmware packet buffer is 64 bytes; a data response is a 2-byte header plus
# up to 62 payload bytes, and a write-data command is a 2-byte header plus up
# to 62 payload bytes. Both directions are capped by NP_PACKET_BUF_SIZE.
PACKET_SIZE = 64
WRITE_CHUNK = 62

# A single response must arrive within this long. The slowest legitimate gap
# is a block erase (a few ms) or a page read, so this is pure hang detection.
READ_TIMEOUT = 30.0

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

# np_cmd_flags_t bitfield.
FLAG_SKIP_BB = 1 << 0
FLAG_INC_SPARE = 1 << 1
FLAG_HW_ECC = 1 << 2

# Firmware error codes, sent negated (np_send_error(-ret)).
ERR_NAMES = {
    1: "NP_ERR_INTERNAL",
    100: "NP_ERR_ADDR_EXCEEDED",
    101: "NP_ERR_ADDR_INVALID",
    102: "NP_ERR_ADDR_NOT_ALIGN",
    103: "NP_ERR_NAND_WR",
    104: "NP_ERR_NAND_RD",
    105: "NP_ERR_NAND_ERASE",
    106: "NP_ERR_CHIP_NOT_CONF",
    107: "NP_ERR_CMD_DATA_SIZE",
    108: "NP_ERR_CMD_INVALID",
    109: "NP_ERR_BUF_OVERFLOW",
    110: "NP_ERR_LEN_NOT_ALIGN",
    111: "NP_ERR_LEN_EXCEEDED",
    112: "NP_ERR_LEN_INVALID",
    113: "NP_ERR_BBT_OVERFLOW",
    114: "NP_ERR_HAL_INVALID",
    115: "NP_ERR_PARAM_INVALID",
}
ERR_BBT_OVERFLOW = 113


class LinkError(Exception):
    """Transport-level failure: cable, permissions, disconnect, hang."""


class LinkTimeout(LinkError, TimeoutError):
    pass


class ProtocolError(LinkError):
    """A response the firmware should never have sent -- i.e. we are out of
    sync with the device's stream and must resynchronise."""


class DeviceError(LinkError):
    """The firmware explicitly reported an error (RESP_STATUS/STATUS_ERROR)."""

    def __init__(self, code):
        self.code = code
        self.name = ERR_NAMES.get(code, f"unknown({code})")
        super().__init__(f"device error {self.name} (code {code})")


def find_device():
    """The programmer (STM32 VID:PID 0483:5740) does not always enumerate as
    the same /dev/ttyACM* node -- an ST-Link's own VCP (0483:3752) competes
    for the same naming range. Locate it by USB VID/PID, via sysfs (no
    subprocess) with a udevadm fallback."""
    import glob
    candidates = sorted(glob.glob("/dev/ttyACM*"))
    for node in candidates:
        if _sysfs_ids(node) == (USB_VID, USB_PID):
            return node
    for node in candidates:
        if _udev_ids(node) == (USB_VID, USB_PID):
            return node
    raise LinkError(
        f"NAND programmer ({USB_VID}:{USB_PID}) not found among "
        f"{candidates or 'no /dev/ttyACM* nodes'} -- is it plugged in?")


def _sysfs_ids(node):
    base = f"/sys/class/tty/{os.path.basename(node)}/device"
    for _ in range(4):  # walk up from the interface to the usb_device
        try:
            vid = open(os.path.join(base, "idVendor")).read().strip()
            pid = open(os.path.join(base, "idProduct")).read().strip()
            return vid, pid
        except OSError:
            base = os.path.join(base, os.pardir)
    return None


def _udev_ids(node):
    import subprocess
    try:
        out = subprocess.run(
            ["udevadm", "info", "-q", "property", "-n", node],
            capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return None
    props = dict(l.split("=", 1) for l in out.splitlines() if "=" in l)
    return props.get("ID_VENDOR_ID"), props.get("ID_MODEL_ID")


class Link:
    """One exclusive connection to the programmer.

    Opened with TIOCEXCL so a second process (an old dump script, a stray
    server instance) cannot interleave bytes into the same stream -- the
    failure mode that makes a dump silently wrong rather than merely fail.
    """

    def __init__(self, path=None):
        # A path passed here is a deliberate pin and is honoured on every
        # (re)open. Without one we locate the programmer by VID/PID each time
        # the port is opened: it does not always come back on the same
        # /dev/ttyACM* node after a re-enumeration (a USB drop mid-command is
        # enough), so caching whichever node we happened to find first would
        # strand every later reopen -- including reconnect() -- on a name that
        # no longer exists.
        self.pinned = path is not None
        self.path = path
        self.fd = None

    def open(self):
        if self.fd is not None:
            return self
        path = self.path if self.pinned else find_device()
        try:
            fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        except OSError as e:
            if e.errno == errno.EACCES:
                raise LinkError(
                    f"permission denied opening {path} -- the invoking user "
                    f"must be in the 'dialout' group (and the group must be "
                    f"active in this login session)") from e
            raise LinkError(f"cannot open {path}: {e}") from e
        try:
            fcntl.ioctl(fd, termios.TIOCEXCL)
            _set_raw(fd)
            termios.tcflush(fd, termios.TCIOFLUSH)
        except OSError as e:
            os.close(fd)
            if e.errno == errno.EBUSY:
                raise LinkError(
                    f"{path} is already opened exclusively by another "
                    f"process") from e
            raise LinkError(f"cannot configure {path}: {e}") from e
        self.fd, self.path = fd, path
        return self

    def close(self):
        if self.fd is not None:
            fd, self.fd = self.fd, None
            # Drop the exclusive-open flag explicitly. The kernel only clears
            # it when the *last* fd on the tty closes, so if anything else
            # still holds the port open (a leaked fd from a crashed run, say)
            # a plain close would leave the device locked out for everyone,
            # including us.
            try:
                fcntl.ioctl(fd, termios.TIOCNXCL)
            except OSError:
                pass
            try:
                os.close(fd)
            except OSError:
                pass

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    def write(self, data):
        if self.fd is None:
            raise LinkError("link is not open")
        mv = memoryview(data)
        while mv:
            try:
                n = os.write(self.fd, mv)
            except OSError as e:
                raise LinkError(f"write failed: {e}") from e
            mv = mv[n:]

    def read_exact(self, n, timeout=READ_TIMEOUT):
        """Block until exactly n bytes arrive. Waits in select(2) rather than
        spinning on a VMIN=0 read, so an idle wait costs no CPU and a real
        stall is detected instead of being slept through."""
        if self.fd is None:
            raise LinkError("link is not open")
        buf = bytearray()
        deadline = time.monotonic() + timeout
        empty_reads = 0
        while len(buf) < n:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise LinkTimeout(
                    f"timed out after {timeout:.0f}s waiting for {n} bytes, "
                    f"got {len(buf)}: {bytes(buf)!r}")
            try:
                ready, _, _ = select.select([self.fd], [], [], min(remaining, 1.0))
            except OSError as e:
                raise LinkError(f"select failed: {e}") from e
            if not ready:
                continue
            try:
                chunk = os.read(self.fd, n - len(buf))
            except OSError as e:
                raise LinkError(f"read failed: {e}") from e
            if chunk:
                buf += chunk
                empty_reads = 0
            else:
                # Readable but empty: on a tty this means the far end went
                # away (unplugged / firmware reset), not "try again forever".
                empty_reads += 1
                if empty_reads > 50:
                    raise LinkError(
                        "device stopped responding (readable but empty) -- "
                        "programmer disconnected or firmware reset?")
        return bytes(buf)

    def resync(self, outstanding=0, quiet=0.5):
        """Discard whatever the firmware is still streaming, then flush both
        directions. Returns the number of bytes discarded.

        Required after any aborted command: the firmware finishes emitting the
        rest of a READ regardless of what the host does, and merely reopening
        the port does *not* discard bytes already queued in the tty. Without
        this, the next command reads the tail of the previous one and either
        blows up as a bad response code or -- far worse -- lands plausible but
        wrong bytes in the middle of a dump.

        `outstanding` is how many payload bytes the aborted command still owes
        us. The drain has to be allowed to run at least that long at the
        hardware's real rate, or it stops early and leaves the stream dirty --
        which looks exactly like the desync it was supposed to clear."""
        if self.fd is None:
            return 0
        # ~0.55 MB/s in practice; budget for half that, plus slack.
        limit = max(15.0, outstanding / (0.25 * 1024 * 1024) + 10.0)
        discarded = 0
        deadline = time.monotonic() + limit
        while time.monotonic() < deadline:
            ready, _, _ = select.select([self.fd], [], [], quiet)
            if not ready:
                break  # quiet for `quiet` seconds => the device is done
            try:
                chunk = os.read(self.fd, 65536)
            except OSError:
                break
            if not chunk:
                break
            discarded += len(chunk)
        try:
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except OSError:
            pass
        return discarded

    def read_response(self):
        header = self.read_exact(2)
        code, info = header[0], header[1]
        if code == RESP_DATA:
            return "data", (self.read_exact(info) if info else b"")
        if code != RESP_STATUS:
            raise ProtocolError(
                f"unknown response code 0x{code:02x} (info 0x{info:02x}) -- "
                f"stream out of sync")
        if info == STATUS_OK:
            return "ok", None
        if info == STATUS_ERROR:
            rest = self.read_exact(12)  # errCode + 11 pad
            raise DeviceError(rest[0])
        if info in (STATUS_BB, STATUS_BB_SKIP):
            addr, size = struct.unpack("<QI", self.read_exact(12))
            return ("bb_skip" if info == STATUS_BB_SKIP else "bb"), (addr, size)
        if info == STATUS_WRITE_ACK:
            rest = self.read_exact(12)  # ackBytes + 4 pad
            return "write_ack", struct.unpack("<Q", rest[:8])[0]
        if info == STATUS_PROGRESS:
            return "progress", struct.unpack("<Q", self.read_exact(8))[0]
        raise ProtocolError(f"unknown status subtype 0x{info:02x}")


def _set_raw(fd):
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(fd)
    iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
               termios.ISTRIP | termios.INLCR | termios.IGNCR |
               termios.ICRNL | termios.IXON | termios.IXOFF | termios.IXANY)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG |
               termios.IEXTEN)
    cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
    cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW,
                      [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])


# --------------------------------------------------------------------------
# Chip database (qt/nando_parallel_chip_db.csv)
# --------------------------------------------------------------------------

FIELDS = [
    "name", "page_size", "block_size", "total_size", "spare_size",
    "bb_mark_off", "tCS", "tCLS", "tALS", "tCLR", "tAR", "tWP", "tRP",
    "tDS", "tCH", "tCLH", "tALH", "tWC", "tRC", "tREA", "row_cycles",
    "col_cycles", "read1_cmd", "read2_cmd", "read_spare_cmd", "read_id_cmd",
    "reset_cmd", "write1_cmd", "write2_cmd", "erase1_cmd", "erase2_cmd",
    "status_cmd", "set_features_cmd", "en_ecc_addr", "en_ecc_value",
    "dis_ecc_value", "ID1", "ID2", "ID3", "ID4", "ID5",
]


def load_chip_db(path=None):
    chips = []
    with open(path or CSV_PATH) as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) != len(FIELDS):
                raise ValueError(
                    f"{path or CSV_PATH}:{lineno}: expected {len(FIELDS)} "
                    f"fields, got {len(parts)}")
            chips.append(dict(zip(FIELDS, parts)))
    if not chips:
        raise ValueError(f"{path or CSV_PATH} contains no chip entries")
    return chips


def opt_int(v):
    return None if v == "-" else int(v)


def opt_byte(v):
    return 0xFF if v == "-" else (int(v) & 0xFF)


def match_chip(chips, chip_id):
    """Match ID bytes against the DB, honouring '-' as 'don't care' for the
    3rd..5th bytes exactly as the Qt app does."""
    id1, id2, id3, id4, id5 = chip_id
    for c in chips:
        if int(c["ID1"]) != id1 or int(c["ID2"]) != id2:
            continue
        for field, actual in (("ID3", id3), ("ID4", id4), ("ID5", id5)):
            want = opt_int(c[field])
            if want is None:
                return c
            if want != actual:
                break
        else:
            return c
    return None


def is_plausible_id(chip_id):
    """All-equal ID bytes (ff ff ff ff ff, ef ef ef ef ef, 00 00 ...) mean the
    data bus is floating: no chip in the socket, or it is not seated. Worth
    distinguishing from 'real chip, just not in the DB'."""
    return len(set(chip_id)) > 1


def eff_sizes(chip):
    """Sizes in main+spare (OOB-inclusive) terms -- what every address and
    length in this module means, matching the firmware's inc_spare path."""
    page_size = int(chip["page_size"])
    spare_size = int(chip["spare_size"])
    block_size = int(chip["block_size"])
    pages_in_block = block_size // page_size
    pages = int(chip["total_size"]) // page_size
    page_eff = page_size + spare_size
    return page_eff, pages_in_block * page_eff, pages * page_eff


def stm_params(c):
    """FSMC timing registers derived from the chip's datasheet timings, using
    the Qt app's arithmetic (72MHz HCLK -> 13.88ns)."""
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

    wait = max(tWP, tRP) / tHCLK - 1
    wait = 0 if wait <= 0 else math.ceil(wait)
    wait2 = (tREA + tsuD_NOE) / tHCLK - 1
    wait2 = 0 if wait2 <= 0 else math.ceil(wait2)
    wait = max(wait, wait2)

    hiz = (max(tCS, tALS, tCLS) + (tWP - tDS)) / tHCLK - 1
    hiz = 0 if hiz <= 0 else math.ceil(hiz)

    hold = max(tCH, tCLH, tALH) / tHCLK - 1
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


# --------------------------------------------------------------------------
# Operations
# --------------------------------------------------------------------------

class Programmer:
    """Stateful session against one programmer.

    Every address and length taken or returned by these methods is in
    main+spare (OOB-inclusive) terms: all commands are issued with
    FLAG_INC_SPARE, so the firmware's page/block/total sizes are the
    ``eff_sizes()`` ones, not the datasheet main-area ones.
    """

    def __init__(self, link=None, chips=None):
        self.link = link if link is not None else Link()
        self.chips = chips if chips is not None else load_chip_db()
        self.chip = None        # matched DB entry the firmware is set up for
        self._conf_name = None  # None => firmware conf state unknown

    # -- session -----------------------------------------------------------

    def open(self):
        """Open the port and make sure the device is not still streaming the
        tail of a command some earlier, abandoned run issued. A crashed or
        killed process leaves the firmware mid-READ; the next process to open
        the port would otherwise read that tail as its own first response."""
        fresh = self.link.fd is None
        self.link.open()
        if fresh:
            self.sync()
        return self

    def sync(self):
        """Prove the stream is clean, draining it first if it is not.

        Two VERSION_GET round trips have to agree: a dirty stream can by
        chance produce one response-shaped read, but not two identical ones.
        On an idle device this is the fast path and costs a few milliseconds,
        which is what makes opening per operation affordable."""
        last = None
        for attempt in range(4):
            if attempt:
                # First pass assumes nothing is in flight; later passes allow
                # for a whole segment still being streamed at us.
                self.link.resync(
                    outstanding=0 if attempt == 1 else 16 * 1024 * 1024)
            try:
                first = self.version()
                if first == self.version():
                    return first
                last = LinkError(f"inconsistent version reads: {first}")
            except LinkError as e:
                last = e
        raise LinkError(
            f"programmer did not return a consistent version after draining "
            f"the stream -- last error: {last}")

    def close(self):
        self.link.close()
        self.chip = None
        self._conf_name = None

    def reconnect(self):
        """Full reset: drop the port, reopen, forget the firmware's conf.

        The firmware's chip_is_conf survives a host-side reopen but not a
        power cycle, and we cannot tell the two apart, so conf is always
        re-sent after this."""
        self.close()
        self.open()

    def _fail(self, exc, outstanding=0):
        """Put the link back into a known-clean state after a failed command.

        Drains the aborted command's remaining output, then proves the stream
        is actually back in sync by issuing a VERSION_GET and checking the
        reply; a drain that merely timed out would otherwise hand the next
        caller a stream that is still dirty. Returns the bytes discarded."""
        try:
            discarded = self.link.resync(outstanding=outstanding)
        except Exception:
            self.close()
            return 0
        # The firmware may have been left mid-write; force a re-conf, which
        # also re-inits its bad block table.
        self._conf_name = None
        try:
            self.version()
        except LinkError:
            # Still dirty (or the device went away). Drain again, harder; if
            # that still will not settle, drop the port so the next call
            # starts from a clean open rather than a stream we cannot trust.
            try:
                self.sync()
            except LinkError:
                self.close()
        return discarded

    def _cmd(self, payload, expect_ok=True, what="command"):
        if len(payload) > PACKET_SIZE:
            raise ProtocolError(
                f"{what} is {len(payload)} bytes, over the firmware's "
                f"{PACKET_SIZE}-byte packet buffer")
        self.link.write(payload)
        if not expect_ok:
            return None
        kind, _ = self.link.read_response()
        if kind != "ok":
            raise ProtocolError(f"{what}: expected OK, got {kind}")
        return None

    # -- identification ----------------------------------------------------

    def version(self):
        self.link.write(bytes([CMD_VERSION_GET]))
        kind, payload = self.link.read_response()
        if kind != "data" or len(payload) != 4:
            raise ProtocolError(f"bad VERSION_GET response: {kind} {payload!r}")
        major, minor, build = struct.unpack("<BBH", payload)
        return major, minor, build

    def conf(self, chip):
        """Send CMD_NAND_CONF for `chip`. Also resets the firmware's bad block
        table, so avoid re-sending it needlessly."""
        sp = stm_params(chip)
        head = struct.pack(
            "<BBIIQIB",
            CMD_NAND_CONF, 0,  # hal 0 = FSMC (parallel NAND)
            int(chip["page_size"]), int(chip["block_size"]),
            int(chip["total_size"]), int(chip["spare_size"]),
            int(chip["bb_mark_off"]),
        )
        # fsmc_conf_t, 22 bytes, in firmware/programmer/fsmc_nand.c order.
        hal_conf = bytes([
            sp["setupTime"], sp["waitSetupTime"], sp["holdSetupTime"],
            sp["hiZSetupTime"], sp["clrSetupTime"], sp["arSetupTime"],
            int(chip["row_cycles"]), int(chip["col_cycles"]),
            opt_byte(chip["read1_cmd"]), opt_byte(chip["read2_cmd"]),
            opt_byte(chip["read_spare_cmd"]), opt_byte(chip["read_id_cmd"]),
            opt_byte(chip["reset_cmd"]), opt_byte(chip["write1_cmd"]),
            opt_byte(chip["write2_cmd"]), opt_byte(chip["erase1_cmd"]),
            opt_byte(chip["erase2_cmd"]), opt_byte(chip["status_cmd"]),
            # set_features_cmd. Must be UNDEFINED_CMD (0xff) when the DB says
            # '-': the firmware treats anything else as a usable SET FEATURES
            # opcode and issues it before every write. Sending status_cmd here
            # (0x70 = READ STATUS) makes the chip drive the bus while the FSMC
            # writes to it -- bus contention immediately before a page program.
            opt_byte(chip["set_features_cmd"]),
            opt_byte(chip["en_ecc_addr"]), opt_byte(chip["en_ecc_value"]),
            opt_byte(chip["dis_ecc_value"]),
        ])
        assert len(hal_conf) == 22, len(hal_conf)
        self._conf_name = None
        self._cmd(head + hal_conf, what="CMD_NAND_CONF")
        self.chip = chip
        self._conf_name = chip["name"]

    def read_id(self):
        self.link.write(bytes([CMD_NAND_READ_ID]))
        kind, payload = self.link.read_response()
        if kind != "data" or len(payload) != 5:
            raise ProtocolError(f"bad READ_ID response: {kind} {payload!r}")
        return tuple(payload)

    def ensure_conf(self):
        """Make sure the firmware is configured for whatever is in the socket,
        re-identifying only when we do not already know.

        READ_ID is itself a chip command and is rejected with
        NP_ERR_CHIP_NOT_CONF on a freshly powered programmer, so a conf always
        has to come first -- with the last known chip if we have one, else the
        first DB entry purely as a bootstrap profile."""
        if self._conf_name is not None:
            return self.chip
        try:
            self.conf(self.chip or self.chips[0])
            chip_id = self.read_id()
        except (ProtocolError, LinkTimeout) as e:
            # A half-consumed CONF/READ_ID reply desyncs everything after it.
            self._fail(e)
            raise
        chip = match_chip(self.chips, chip_id)
        if chip is None:
            id_hex = " ".join(f"{b:02x}" for b in chip_id)
            if not is_plausible_id(chip_id):
                raise LinkError(
                    f"chip ID reads back as {id_hex} -- every byte identical "
                    f"means a floating bus: no chip in the socket, or it is "
                    f"not seated properly")
            raise LinkError(f"no chip DB match for ID {id_hex}")
        if chip["name"] != self._conf_name:
            self.conf(chip)
        return chip

    def identify(self):
        major, minor, build = self.version()
        chip = self.ensure_conf()
        chip_id = self.read_id()
        return {
            "firmware_version": f"{major}.{minor}.{build}",
            "chip_id_hex": " ".join(f"{b:02x}h" for b in chip_id),
            "chip_name": chip["name"],
            "page_size": int(chip["page_size"]),
            "block_size": int(chip["block_size"]),
            "spare_size": int(chip["spare_size"]),
            "total_size": int(chip["total_size"]),
            "page_size_eff": eff_sizes(chip)[0],
            "block_size_eff": eff_sizes(chip)[1],
            "total_size_eff": eff_sizes(chip)[2],
        }

    # -- validation --------------------------------------------------------

    def _check_range(self, addr, length, align, align_name):
        page_eff, block_eff, total_eff = eff_sizes(self.chip)
        if addr < 0 or length <= 0:
            raise ValueError(f"addr/length must be positive, got {addr}/{length}")
        if addr % align or length % align:
            raise ValueError(
                f"addr (0x{addr:x}) and length (0x{length:x}) must both be "
                f"{align_name}-aligned (0x{align:x} bytes, main+spare)")
        if addr + length > total_eff:
            raise ValueError(
                f"range 0x{addr:x}+0x{length:x} exceeds the chip's main+spare "
                f"size 0x{total_eff:x}")

    # -- read --------------------------------------------------------------

    def read(self, addr, length, sink, skip_bb=False, progress=None):
        """Read [addr, addr+length) and feed it to `sink(bytes)`.

        Returns (bytes_delivered, notices) where notices is a list of
        (kind, addr, size) with kind "bb_skip" (a block the firmware stepped
        over) or "bb" (a read error on a block it still returned data for).
        Only "bb_skip" advances the chip address past addr+length, so callers
        tracking position must not conflate the two."""
        self.ensure_conf()
        page_eff, block_eff, _ = eff_sizes(self.chip)
        self._check_range(addr, length, page_eff, "page")

        flags = FLAG_INC_SPARE | (FLAG_SKIP_BB if skip_bb else 0)
        got = 0
        bad = []
        try:
            self.link.write(struct.pack("<BQQB", CMD_NAND_READ, addr, length, flags))
            while got < length:
                kind, payload = self.link.read_response()
                if kind == "data":
                    sink(payload)
                    got += len(payload)
                    if progress:
                        progress(got, length)
                elif kind in ("bb", "bb_skip"):
                    bad.append((kind,) + payload)
                else:
                    raise ProtocolError(f"unexpected {kind} during READ")
            return got, bad
        except BaseException as e:
            self._fail(e, outstanding=max(length - got, 0))
            raise

    # -- erase -------------------------------------------------------------

    def erase(self, addr, length, progress=None):
        """Erase [addr, addr+length). The firmware requires *block* alignment
        here, unlike read/write which only need page alignment."""
        self.ensure_conf()
        _, block_eff, _ = eff_sizes(self.chip)
        self._check_range(addr, length, block_eff, "block")
        try:
            self.link.write(struct.pack("<BQQB", CMD_NAND_ERASE, addr, length,
                                        FLAG_INC_SPARE))
            bad = []
            while True:
                kind, payload = self.link.read_response()
                if kind == "ok":
                    return bad
                if kind == "progress":
                    if progress:
                        progress(payload, length)
                elif kind in ("bb", "bb_skip"):
                    bad.append((kind,) + payload)
                else:
                    raise ProtocolError(f"unexpected {kind} during ERASE")
        except BaseException as e:
            self._fail(e)
            raise

    # -- write -------------------------------------------------------------

    def write(self, addr, length, src, progress=None):
        """Write `length` bytes read from file object `src` to [addr, ...).

        Address-for-address: bad blocks are not skipped, so what lands at an
        address is what the caller put at that file offset. Returns the list
        of bad blocks the firmware reported during the write."""
        self.ensure_conf()
        page_eff, _, _ = eff_sizes(self.chip)
        self._check_range(addr, length, page_eff, "page")
        try:
            self.link.write(struct.pack("<BQQB", CMD_NAND_WRITE_S, addr, length,
                                        FLAG_INC_SPARE))
            kind, _ = self.link.read_response()
            if kind != "ok":
                raise ProtocolError(f"WRITE_S: expected OK, got {kind}")

            bad = []
            sent = 0
            acked = 0
            while sent < length:
                want = min(WRITE_CHUNK, length - sent)
                chunk = src.read(want)
                if len(chunk) != want:
                    raise ValueError(
                        f"source ran out at offset {sent}: wanted {want} "
                        f"bytes, got {len(chunk)}")
                self.link.write(struct.pack("<BB", CMD_NAND_WRITE_D,
                                            len(chunk)) + chunk)
                sent += len(chunk)
                # Mirror the firmware's own ack rule (np_cmd_nand_write_data):
                # it acks once a page's worth has accumulated, or at the end.
                if sent - acked >= page_eff or sent == length:
                    acked = self._await_write_ack(sent, bad)
                    if progress:
                        progress(sent, length)

            self.link.write(bytes([CMD_NAND_WRITE_E]))
            kind, _ = self.link.read_response()
            if kind != "ok":
                raise ProtocolError(f"WRITE_E: expected OK, got {kind}")
            return bad
        except BaseException as e:
            self._fail(e)
            raise

    def _await_write_ack(self, expected, bad):
        """Wait for the write ack for `expected` bytes.

        Bad-block notices are interleaved into this stream whenever a page
        program fails, so they have to be drained here rather than treated as
        a desync -- the old client aborted the whole write on the first one."""
        while True:
            kind, payload = self.link.read_response()
            if kind == "write_ack":
                if payload != expected:
                    raise ProtocolError(
                        f"write ack mismatch: device acked {payload}, host "
                        f"had sent {expected}")
                return payload
            if kind in ("bb", "bb_skip"):
                bad.append((kind,) + payload)
                continue
            raise ProtocolError(f"unexpected {kind} while awaiting write ack")

    # -- bad blocks --------------------------------------------------------

    def read_bad_blocks(self):
        """Firmware BBT scan. Raises DeviceError(ERR_BBT_OVERFLOW) if the chip
        has more bad blocks than the firmware's 20-entry table holds."""
        self.ensure_conf()
        try:
            self.link.write(bytes([CMD_NAND_READ_BB]))
            bad = []
            while True:
                kind, payload = self.link.read_response()
                if kind == "ok":
                    return bad
                if kind in ("bb", "bb_skip"):
                    bad.append((kind,) + payload)
                elif kind != "progress":
                    raise ProtocolError(f"unexpected {kind} during READ_BB")
        except BaseException as e:
            self._fail(e)
            raise
