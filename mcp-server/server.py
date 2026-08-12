#!/usr/bin/env python3
"""MCP server wrapping the NANDO NAND programmer's USB protocol.

Exposes the hardware I/O primitives (identify/read/write/dump/bad-block
scan/connectivity check) as MCP tools, built directly on nando_client.py.
Long operations report progress natively over MCP instead of the
background-task-and-poll-a-logfile pattern this was replacing.

Physical throughput is unchanged (~0.5-0.6 MB/s, USB CDC limited) -- the
win here is protocol reliability and not re-deriving this logic by hand
each session, not speed.
"""
import os
import struct
import time
from collections import Counter

from mcp.server.mcpserver import MCPServer, Context

import nando_client as nc

mcp = MCPServer(
    "nando",
    instructions=(
        "Tools for the NANDO USB NAND flash programmer. Only one chip can "
        "be addressed at a time (whatever is physically in the socket). "
        "write() erases and overwrites real hardware -- confirm with the "
        "user before calling it. All addresses/lengths are in "
        "main+spare (OOB-inclusive) byte terms unless noted."
    ),
)

# Single physical serial link -- one shared connection, reopened lazily.
_state = {"fd": None, "chip": None}


def _chip_db():
    return nc.load_chip_db(nc.CSV_PATH)


def _connect():
    if _state["fd"] is None:
        _state["fd"] = nc.open_port(nc.find_device())
    return _state["fd"]


def _identify_and_conf():
    """Open the port if needed, identify the chip, configure the firmware
    for it. Returns the matched chip dict. Raises if unidentified."""
    fd = _connect()
    chips = _chip_db()
    nc.cmd_conf(fd, chips[0])
    chip_id = nc.cmd_read_id(fd)
    chip = nc.match_chip(chips, chip_id)
    if not chip:
        raise RuntimeError(
            f"No chip DB match for ID {' '.join(f'{b:02x}h' for b in chip_id)}"
        )
    nc.cmd_conf(fd, chip)
    _state["chip"] = chip
    return chip, chip_id


def _eff_sizes(chip):
    page_size = int(chip["page_size"])
    spare_size = int(chip["spare_size"])
    block_size = int(chip["block_size"])
    pages_in_block = block_size // page_size
    pages = int(chip["total_size"]) // page_size
    page_size_eff = page_size + spare_size
    block_size_eff = pages_in_block * page_size_eff
    total_size_eff = pages * page_size_eff
    return page_size_eff, block_size_eff, total_size_eff


@mcp.tool()
def identify() -> dict:
    """Identify the chip currently in the programmer's socket: firmware
    version, raw ID bytes, and the matched entry from the chip database
    (name, page/block/spare size). Safe, read-only."""
    fd = _connect()
    major, minor, build = nc.cmd_version(fd)
    chip, chip_id = _identify_and_conf()
    return {
        "firmware_version": f"{major}.{minor}.{build}",
        "chip_id_hex": " ".join(f"{b:02x}h" for b in chip_id),
        "chip_name": chip["name"],
        "page_size": int(chip["page_size"]),
        "block_size": int(chip["block_size"]),
        "spare_size": int(chip["spare_size"]),
        "total_size": int(chip["total_size"]),
    }


@mcp.tool()
def conn_check() -> dict:
    """Fast (few-second) non-destructive connectivity/seating check: 3
    repeated ID reads plus single-page samples from ~11 locations spread
    across the chip. Flags pages that look 'stuck' at a near-constant
    non-erased byte value, which is the signature of a bad socket
    contact rather than real chip content (a genuinely blank/erased page
    reading all 0xFF is normal and NOT flagged)."""
    fd = _connect()
    ids = [nc.cmd_read_id(fd) for _ in range(3)]
    chip, _ = _identify_and_conf()

    page_size = int(chip["page_size"])
    spare_size = int(chip["spare_size"])
    block_size = int(chip["block_size"])
    page_size_eff = page_size + spare_size
    total_blocks = int(chip["total_size"]) // block_size

    sample_blocks = sorted(set(
        int(total_blocks * f) for f in
        [0, 0.05, 0.15, 0.25, 0.35, 0.5, 0.65, 0.75, 0.85, 0.95, 0.999]
    ))

    samples = []
    healthy = 0
    for blk in sample_blocks:
        addr = blk * page_size_eff * (block_size // page_size)
        nc.send(fd, struct.pack("<BQQB", nc.CMD_NAND_READ, addr, page_size_eff, 0 | 2))
        data = bytearray()
        while len(data) < page_size_eff:
            kind, payload = nc.read_response(fd)
            if kind == "data":
                data += payload
            elif kind not in ("bb", "bb_skip"):
                raise nc.RespError(f"unexpected response: {kind}")
        c = Counter(data)
        val, cnt = c.most_common(1)[0]
        frac = cnt / len(data)
        stuck = len(c) <= 2 and frac > 0.99 and val != 0xFF
        if not stuck:
            healthy += 1
        samples.append({
            "block": blk, "addr": f"0x{addr:08x}",
            "distinct_byte_values": len(c),
            "dominant_byte": f"0x{val:02x}", "dominant_fraction": round(frac, 3),
            "status": "stuck" if stuck else "ok",
        })

    return {
        "id_reads_consistent": len(set(ids)) == 1,
        "chip_id_hex": " ".join(f"{b:02x}h" for b in ids[0]),
        "chip_name": chip["name"],
        "samples": samples,
        "healthy_count": healthy,
        "sampled_count": len(samples),
        "verdict": "ok" if healthy == len(samples) else
            "possible contact issue -- reseat the chip and retry",
    }


@mcp.tool()
def bad_block_scan() -> dict:
    """Read the chip's bad block list. Tries the firmware's fast BBT scan
    first; if the chip has more bad blocks than the firmware's 20-entry
    table can hold (NP_ERR_BBT_OVERFLOW), automatically falls back to
    scanning the OOB bad-block marker byte directly (requires a full raw
    read of the chip, so this path is much slower)."""
    fd = _connect()
    chip, _ = _identify_and_conf()
    block_size = int(chip["block_size"])
    total_blocks = int(chip["total_size"]) // block_size

    try:
        bad = nc.cmd_read_bb(fd)
        return {
            "method": "firmware_bbt",
            "bad_blocks": [b // block_size for b, _ in bad],
            "total_blocks": total_blocks,
        }
    except nc.RespError as e:
        if "err_code=113" not in str(e):
            raise
        # BBT overflow -- fall back to a raw scan via a full raw dump.
        page_size_eff, block_size_eff, total_size_eff = _eff_sizes(chip)
        tmp_path = "/tmp/nando_bbscan_tmp.bin"
        _dump_raw(chip, tmp_path)
        page_size = int(chip["page_size"])
        bb_mark_off = int(chip["bb_mark_off"])
        bad_blocks = []
        with open(tmp_path, "rb") as f:
            for block in range(total_blocks):
                f.seek(block * block_size_eff + page_size + bb_mark_off)
                marker = f.read(1)
                if marker and marker[0] != 0xFF:
                    bad_blocks.append(block)
        os.remove(tmp_path)
        return {
            "method": "raw_oob_scan_fallback",
            "bad_blocks": bad_blocks,
            "total_blocks": total_blocks,
            "note": "firmware BBT overflowed (>20 bad blocks); scanned OOB "
                    "markers directly from a full raw read instead",
        }


def _dump_raw(chip, out_path, ctx=None):
    """Address-for-address (skipBB=0) full dump, segmented with retry."""
    page_size_eff, block_size_eff, total_size_eff = _eff_sizes(chip)
    seg_len = 64 * block_size_eff
    fd = _connect()
    written = 0
    with open(out_path, "wb") as f:
        addr = 0
        while addr < total_size_eff:
            this_len = min(seg_len, total_size_eff - addr)
            for attempt in range(5):
                try:
                    nc.send(fd, struct.pack("<BQQB", nc.CMD_NAND_READ, addr, this_len, 0 | 2))
                    out = bytearray()
                    while len(out) < this_len:
                        kind, payload = nc.read_response(fd)
                        if kind == "data":
                            out += payload
                        elif kind not in ("bb", "bb_skip"):
                            raise nc.RespError(f"unexpected: {kind}")
                    break
                except (nc.RespError, TimeoutError):
                    if attempt == 4:
                        raise
                    _state["fd"] = None
                    fd = _connect()
                    nc.cmd_conf(fd, chip)
            f.write(out)
            written += len(out)
            addr += this_len
    return written


@mcp.tool()
async def dump_full(ctx: Context, label: str = "", raw: bool = True) -> dict:
    """Full-chip read-only dump to a file under
    /home/user/Documents/Nando/dumps/. Does not modify the chip.

    raw=True: address-for-address (skipBB=0), includes bad blocks'
    literal content -- correct positional addressing throughout, but
    the file is exactly the nominal chip size regardless of bad blocks.
    raw=False: skips bad blocks (skipBB=1) -- output shrinks by one
    block per bad block found, matches upstream chip semantics, NOT
    directly address-comparable past the first bad block.

    Takes several minutes for a 128MB-class chip (~0.5-0.6 MB/s)."""
    chip, chip_id = _identify_and_conf()
    os.makedirs("/home/user/Documents/Nando/dumps", exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    tag = f"{label}_" if label else ""
    suffix = "_raw" if raw else ""
    out_path = f"/home/user/Documents/Nando/dumps/{tag}{chip['name']}_{ts}{suffix}.bin"

    fd = _connect()
    page_size_eff, block_size_eff, total_size_eff = _eff_sizes(chip)

    if raw:
        await ctx.report_progress(0, total_size_eff, f"raw dump of {chip['name']} -> {out_path}")
        written = 0
        t0 = time.time()
        with open(out_path, "wb") as f:
            addr = 0
            seg_len = 64 * block_size_eff
            while addr < total_size_eff:
                this_len = min(seg_len, total_size_eff - addr)
                for attempt in range(5):
                    try:
                        nc.send(fd, struct.pack("<BQQB", nc.CMD_NAND_READ, addr, this_len, 0 | 2))
                        out = bytearray()
                        while len(out) < this_len:
                            kind, payload = nc.read_response(fd)
                            if kind == "data":
                                out += payload
                            elif kind not in ("bb", "bb_skip"):
                                raise nc.RespError(f"unexpected: {kind}")
                        break
                    except (nc.RespError, TimeoutError):
                        if attempt == 4:
                            raise
                        _state["fd"] = None
                        fd = _connect()
                        nc.cmd_conf(fd, chip)
                f.write(out)
                written += len(out)
                addr += this_len
                await ctx.report_progress(written, total_size_eff,
                                           f"{written/1024/1024:.1f}/{total_size_eff/1024/1024:.1f} MB")
        elapsed = time.time() - t0
        return {"path": out_path, "bytes_written": written,
                "elapsed_s": round(elapsed, 1), "method": "raw"}
    else:
        bb = bad_block_scan()
        known_bad = len(bb["bad_blocks"])
        target_total = total_size_eff - known_bad * block_size_eff
        await ctx.report_progress(
            0, target_total,
            f"skip-bb dump of {chip['name']} -> {out_path} "
            f"({known_bad} bad block(s) known, target {target_total} bytes)")
        written = 0
        t0 = time.time()
        with open(out_path, "wb") as f:
            addr = 0
            seg_len = 64 * block_size_eff
            while written < target_total:
                this_len = min(seg_len, target_total - written)
                nc.send(fd, struct.pack("<BQQB", nc.CMD_NAND_READ, addr, this_len, 1 | 2))
                out = bytearray()
                n_skip = 0
                while len(out) < this_len:
                    kind, payload = nc.read_response(fd)
                    if kind == "data":
                        out += payload
                    elif kind in ("bb", "bb_skip"):
                        n_skip += 1
                    else:
                        raise nc.RespError(f"unexpected: {kind}")
                f.write(out)
                written += len(out)
                addr += this_len + n_skip * block_size_eff
                await ctx.report_progress(written, target_total,
                                           f"{written/1024/1024:.1f}/{target_total/1024/1024:.1f} MB")
        elapsed = time.time() - t0
        return {"path": out_path, "bytes_written": written,
                "elapsed_s": round(elapsed, 1), "method": "skip_bad_blocks",
                "bad_blocks_skipped": known_bad}


@mcp.tool()
async def write(ctx: Context, path: str, erase_first: bool = True) -> dict:
    """Erase and write a file to the chip currently in the socket,
    address-for-address (no bad-block skip), then read back the whole
    thing and verify byte-for-byte. The file must be exactly the chip's
    main+spare (OOB-inclusive) size.

    DESTRUCTIVE: erases and overwrites the physical chip. Confirm with
    the user before calling this. Takes several minutes for a 128MB-class
    chip (erase + write + verify, each ~4 min for write/verify)."""
    chip, _ = _identify_and_conf()
    page_size_eff, block_size_eff, total_size_eff = _eff_sizes(chip)

    file_size = os.path.getsize(path)
    if file_size != total_size_eff:
        raise ValueError(
            f"file size {file_size} does not match chip main+spare size "
            f"{total_size_eff} -- refusing to write"
        )

    fd = _connect()

    if erase_first:
        await ctx.report_progress(0, total_size_eff * 2, "erasing entire chip")
        nc.send(fd, struct.pack("<BQQB", nc.CMD_NAND_ERASE, 0, total_size_eff, 0 | 2))
        while True:
            kind, _ = nc.read_response(fd)
            if kind == "ok":
                break
            if kind not in ("progress", "bb", "bb_skip"):
                raise nc.RespError(f"unexpected during erase: {kind}")

    await ctx.report_progress(0, total_size_eff * 2, f"writing {path}")
    nc.send(fd, struct.pack("<BQQB", nc.CMD_NAND_WRITE_S, 0, total_size_eff, 0 | 2))
    kind, _ = nc.read_response(fd)
    if kind != "ok":
        raise nc.RespError(f"WRITE_S failed: {kind}")

    CHUNK = 62
    bytes_written = 0
    bytes_ack = 0
    with open(path, "rb") as f:
        while bytes_written < total_size_eff:
            chunk = f.read(CHUNK)
            nc.send(fd, struct.pack("<BB", nc.CMD_NAND_WRITE_D, len(chunk)) + chunk)
            bytes_written += len(chunk)
            if bytes_written - bytes_ack >= page_size_eff or bytes_written == total_size_eff:
                kind, ack_bytes = nc.read_response(fd)
                if kind != "write_ack" or ack_bytes != bytes_written:
                    raise nc.RespError(f"write ack mismatch: {kind} {ack_bytes}")
                bytes_ack = bytes_written
                await ctx.report_progress(bytes_written, total_size_eff * 2,
                                           f"write {bytes_written/1024/1024:.1f}/{total_size_eff/1024/1024:.1f} MB")
    nc.send(fd, bytes([nc.CMD_NAND_WRITE_E]))
    kind, _ = nc.read_response(fd)
    if kind != "ok":
        raise nc.RespError(f"WRITE_E failed: {kind}")

    await ctx.report_progress(total_size_eff, total_size_eff * 2, "reading back and verifying")
    nc.send(fd, struct.pack("<BQQB", nc.CMD_NAND_READ, 0, total_size_eff, 0 | 2))
    got = 0
    mismatches = 0
    first_mismatch = None
    with open(path, "rb") as f:
        while got < total_size_eff:
            kind, payload = nc.read_response(fd)
            if kind == "data":
                expected = f.read(len(payload))
                if expected != payload:
                    for i, (a, b) in enumerate(zip(expected, payload)):
                        if a != b:
                            mismatches += 1
                            if first_mismatch is None:
                                first_mismatch = got + i
                got += len(payload)
            elif kind not in ("bb", "bb_skip"):
                raise nc.RespError(f"unexpected during verify: {kind}")
            await ctx.report_progress(total_size_eff + got, total_size_eff * 2,
                                       f"verify {got/1024/1024:.1f}/{total_size_eff/1024/1024:.1f} MB")

    return {
        "bytes_written": bytes_written,
        "bytes_verified": got,
        "mismatches": mismatches,
        "first_mismatch_offset": (f"0x{first_mismatch:x}" if first_mismatch is not None else None),
        "result": "PASS" if mismatches == 0 else "FAIL",
    }


if __name__ == "__main__":
    mcp.run("stdio")
