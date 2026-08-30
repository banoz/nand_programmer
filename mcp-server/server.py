#!/usr/bin/env python3
"""MCP server for the NANDO USB NAND programmer.

A thin adapter over nando_client.Programmer: this module owns concurrency,
progress reporting and file layout; the protocol itself lives next door.

Two rules shape the design.

1. One physical link. Every tool runs under a single hardware lock, and the
   port is opened with TIOCEXCL, so nothing can interleave into the stream.
2. Never block the event loop. All hardware I/O is blocking and slow (a full
   dump is ~4 minutes at the USB CDC-limited ~0.55 MB/s), so it runs in a
   worker thread and progress is bridged back to the loop. Otherwise the
   server stops answering the client -- including cancellations -- for the
   whole operation.
"""
import hashlib
import os
import tempfile
import threading
import time
from collections import Counter

import anyio
import anyio.from_thread
import anyio.to_thread

from mcp.server.mcpserver import MCPServer, Context

import nando_client as nc

DUMP_DIR = os.environ.get("NANDO_DUMP_DIR", "/home/user/Documents/Nando/dumps")

# Segment size for streamed reads. Big enough that per-command overhead is
# irrelevant, small enough that a retry after a glitch is cheap.
SEGMENT_BLOCKS = 64
READ_RETRIES = 5

# Don't emit more than this many progress notifications per second.
PROGRESS_INTERVAL = 0.5

mcp = MCPServer(
    "nando",
    instructions=(
        "Tools for the NANDO USB NAND flash programmer. Only one chip can be "
        "addressed at a time (whatever is physically in the socket). "
        "write() and write_range() erase and overwrite real hardware -- "
        "confirm with the user before calling them. All addresses and lengths "
        "are in main+spare (OOB-inclusive) byte terms; identify() reports "
        "both the datasheet sizes and the effective ones."
    ),
)

_prog = nc.Programmer()
_lock = threading.RLock()


class _Progress:
    """Bridges progress from the worker thread back to the event loop, rate
    limited so a fast inner loop cannot flood the client."""

    def __init__(self, ctx):
        self.ctx = ctx
        self.last = 0.0

    def __call__(self, current, total, message="", force=False):
        if self.ctx is None:
            return
        now = time.monotonic()
        if not force and now - self.last < PROGRESS_INTERVAL:
            return
        self.last = now
        anyio.from_thread.run(self.ctx.report_progress, float(current),
                              float(total), message)


async def _hw(ctx, worker):
    """Run `worker(report)` against the programmer in a worker thread."""
    report = _Progress(ctx)

    def run():
        with _lock:
            _prog.open()
            return worker(report)

    return await anyio.to_thread.run_sync(run, abandon_on_cancel=False)


def _mb(n):
    return n / 1024 / 1024


# --------------------------------------------------------------------------
# Shared read/dump implementation
# --------------------------------------------------------------------------

def _read_segments(report, sink, addr, length, skip_bb, label):
    """Stream [addr, addr+length) into `sink`, in segments, with retry.

    A retry is only safe because Programmer.read() resynchronises the link on
    failure -- the firmware keeps streaming the rest of an aborted READ, and
    those bytes would otherwise be picked up as the start of the retried
    segment and written into the output as plausible-looking garbage."""
    _, block_eff, _ = nc.eff_sizes(_prog.chip)
    seg_len = SEGMENT_BLOCKS * block_eff
    done = 0
    skipped = []
    cursor = addr
    t0 = time.monotonic()
    report(0, length, f"{label}: starting", force=True)
    while done < length:
        this_len = min(seg_len, length - done)
        for attempt in range(READ_RETRIES):
            buf = bytearray()
            base = done

            def inner(seg_got, seg_total, _base=base):
                rate = _mb(_base + seg_got) / max(time.monotonic() - t0, 0.001)
                report(_base + seg_got, length,
                       f"{label}: {_mb(_base + seg_got):.1f}/{_mb(length):.1f} MB "
                       f"({rate:.2f} MB/s)")

            try:
                _, bad = _prog.read(cursor, this_len, buf.extend,
                                    skip_bb=skip_bb, progress=inner)
                break
            except (nc.LinkError, nc.DeviceError) as e:
                if attempt == READ_RETRIES - 1:
                    raise nc.LinkError(
                        f"{label}: giving up on 0x{cursor:x}+0x{this_len:x} "
                        f"after {READ_RETRIES} attempts: {e}") from e
                report(done, length,
                       f"{label}: retry {attempt + 1} at 0x{cursor:x} ({e})",
                       force=True)
        sink(bytes(buf))
        done += len(buf)
        # Only blocks the firmware actually stepped over move the chip address
        # on; a "bb" notice is a read error on a block whose data we still got.
        stepped = sum(1 for kind, _a, _s in bad if kind == "bb_skip")
        cursor += this_len + stepped * block_eff
        skipped.extend(bad)
    report(length, length, f"{label}: done", force=True)
    return done, skipped


def _scan_bad_blocks(report):
    """Bad block list, preferring the firmware's BBT and falling back to
    reading the OOB markers when it overflows its 20-entry table."""
    chip = _prog.ensure_conf()
    page_size = int(chip["page_size"])
    block_size = int(chip["block_size"])
    bb_mark_off = int(chip["bb_mark_off"])
    total_blocks = int(chip["total_size"]) // block_size
    page_eff, block_eff, total_eff = nc.eff_sizes(chip)

    try:
        bad = _prog.read_bad_blocks()
        # The firmware reports main-area addresses (page * page_size), so the
        # block index divides by the main-area block size.
        return {
            "method": "firmware_bbt",
            "bad_blocks": sorted({a // block_size for _kind, a, _s in bad}),
            "total_blocks": total_blocks,
        }
    except nc.DeviceError as e:
        if e.code != nc.ERR_BBT_OVERFLOW:
            raise

    fd, tmp_path = tempfile.mkstemp(prefix="nando_bbscan_", suffix=".bin")
    os.close(fd)
    try:
        with open(tmp_path, "wb") as f:
            _read_segments(report, f.write, 0, total_eff, False,
                           "bad block scan (raw)")
        bad_blocks = []
        with open(tmp_path, "rb") as f:
            for block in range(total_blocks):
                f.seek(block * block_eff + page_size + bb_mark_off)
                marker = f.read(1)
                if marker and marker[0] != 0xFF:
                    bad_blocks.append(block)
    finally:
        try:
            os.remove(tmp_path)
        except OSError:
            pass
    return {
        "method": "raw_oob_scan_fallback",
        "bad_blocks": bad_blocks,
        "total_blocks": total_blocks,
        "note": "the firmware's bad block table overflowed (>20 bad blocks), "
                "so the OOB markers were read directly from a full raw read",
    }


# --------------------------------------------------------------------------
# Tools
# --------------------------------------------------------------------------

@mcp.tool()
async def identify(ctx: Context) -> dict:
    """Identify the chip currently in the programmer's socket: firmware
    version, raw ID bytes, and the matched chip database entry with both the
    datasheet sizes and the effective (main+spare) ones every other tool
    addresses in. Safe, read-only."""
    return await _hw(ctx, lambda report: _prog.identify())


@mcp.tool()
async def reset_link(ctx: Context) -> dict:
    """Drop and reopen the USB connection to the programmer and forget any
    cached chip configuration. Use this after unplugging/replugging the
    programmer, swapping the chip in the socket, or if a tool reported a
    protocol desync. Safe, read-only."""
    def worker(report):
        _prog.reconnect()
        return {"port": _prog.link.path, "state": "reconnected"}
    return await _hw(ctx, worker)


@mcp.tool()
async def conn_check(ctx: Context) -> dict:
    """Fast (few-second) non-destructive connectivity/seating check: repeated
    ID reads plus single-page samples from ~11 locations spread across the
    chip. Flags pages that look 'stuck' at a near-constant non-erased byte
    value, the signature of a bad socket contact rather than real content (a
    genuinely blank page reading all 0xFF is normal and NOT flagged)."""
    def worker(report):
        chip = _prog.ensure_conf()
        ids = [_prog.read_id() for _ in range(3)]
        page_eff, block_eff, _ = nc.eff_sizes(chip)
        total_blocks = int(chip["total_size"]) // int(chip["block_size"])
        sample_blocks = sorted(set(
            min(int(total_blocks * f), total_blocks - 1)
            for f in (0, .05, .15, .25, .35, .5, .65, .75, .85, .95, .999)))

        samples = []
        healthy = 0
        for i, blk in enumerate(sample_blocks):
            addr = blk * block_eff
            buf = bytearray()
            _prog.read(addr, page_eff, buf.extend)
            counts = Counter(buf)
            val, cnt = counts.most_common(1)[0]
            frac = cnt / len(buf)
            stuck = len(counts) <= 2 and frac > 0.99 and val != 0xFF
            healthy += not stuck
            samples.append({
                "block": blk, "addr": f"0x{addr:08x}",
                "distinct_byte_values": len(counts),
                "dominant_byte": f"0x{val:02x}",
                "dominant_fraction": round(frac, 3),
                "status": "stuck" if stuck else "ok",
            })
            report(i + 1, len(sample_blocks), f"sampled block {blk}")

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
    return await _hw(ctx, worker)


@mcp.tool()
async def bad_block_scan(ctx: Context) -> dict:
    """Read the chip's bad block list. Uses the firmware's fast bad block
    table scan; if the chip has more bad blocks than the firmware's 20-entry
    table holds, automatically falls back to reading the OOB marker bytes from
    a full raw read (much slower). Safe, read-only."""
    return await _hw(ctx, _scan_bad_blocks)


@mcp.tool()
async def read_range(ctx: Context, addr: int, length: int,
                     out_path: str = "", skip_bb: bool = False) -> dict:
    """Read [addr, addr+length) from the chip. addr and length are main+spare
    byte offsets and must be page-aligned (page_size_eff from identify()).

    Writes to out_path when given; otherwise the data is only summarised
    (sha256 plus a hex preview of the first 256 bytes), so a large read
    without out_path is a checksum, not a transfer. Safe, read-only."""
    def worker(report):
        _prog.ensure_conf()
        digest = hashlib.sha256()
        head = bytearray()

        def sink(data):
            digest.update(data)
            if len(head) < 256:
                head.extend(data[:256 - len(head)])

        t0 = time.monotonic()
        if out_path:
            with open(out_path, "wb") as f:
                def sink_file(data, _s=sink, _f=f):
                    _f.write(data)
                    _s(data)
                got, skipped = _read_segments(report, sink_file, addr, length,
                                              skip_bb, "read")
        else:
            got, skipped = _read_segments(report, sink, addr, length, skip_bb,
                                          "read")
        return {
            "addr": f"0x{addr:x}",
            "length": f"0x{length:x}",
            "bytes_read": got,
            "path": out_path or None,
            "sha256": digest.hexdigest(),
            "preview_hex": head.hex(),
            "bad_blocks_encountered": [f"{kind}@0x{a:x}" for kind, a, _ in skipped],
            "elapsed_s": round(time.monotonic() - t0, 1),
        }
    return await _hw(ctx, worker)


@mcp.tool()
async def dump_full(ctx: Context, label: str = "", raw: bool = True) -> dict:
    """Full-chip read-only dump to a file under the dumps directory.

    raw=True: address-for-address, bad blocks included as-is. The file is
    exactly the chip's main+spare size and every offset in it is the chip
    address -- this is what you want for comparing dumps or rebuilding images.
    raw=False: bad blocks skipped, matching upstream flashing semantics. The
    file shrinks by one block per bad block and is NOT address-comparable past
    the first one.

    Takes several minutes for a 128MB-class chip (~0.55 MB/s)."""
    def worker(report):
        chip = _prog.ensure_conf()
        page_eff, block_eff, total_eff = nc.eff_sizes(chip)
        os.makedirs(DUMP_DIR, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        tag = f"{label}_" if label else ""
        out_path = os.path.join(
            DUMP_DIR, f"{tag}{chip['name']}_{ts}{'_raw' if raw else ''}.bin")

        if raw:
            target = total_eff
            known_bad = None
        else:
            bb = _scan_bad_blocks(report)
            known_bad = len(bb["bad_blocks"])
            target = total_eff - known_bad * block_eff

        digest = hashlib.sha256()
        t0 = time.monotonic()
        tmp_path = out_path + ".part"
        try:
            with open(tmp_path, "wb") as f:
                def sink(data):
                    f.write(data)
                    digest.update(data)
                got, skipped = _read_segments(
                    report, sink, 0, target, not raw,
                    f"{'raw' if raw else 'skip-bb'} dump of {chip['name']}")
            os.replace(tmp_path, out_path)
        except BaseException:
            # A partial dump under the real name is worse than no dump: it
            # looks like a complete one to everything downstream.
            try:
                os.remove(tmp_path)
            except OSError:
                pass
            raise

        elapsed = time.monotonic() - t0
        result = {
            "path": out_path,
            "bytes_written": got,
            "sha256": digest.hexdigest(),
            "elapsed_s": round(elapsed, 1),
            "rate_mb_s": round(_mb(got) / max(elapsed, 0.001), 2),
            "method": "raw" if raw else "skip_bad_blocks",
        }
        if known_bad is not None:
            result["bad_blocks_skipped"] = known_bad
        return result
    return await _hw(ctx, worker)


def _write_verify(report, chip, addr, length, path, erase_first):
    page_eff, block_eff, total_eff = nc.eff_sizes(chip)

    if erase_first:
        # ERASE needs block alignment even though WRITE only needs page
        # alignment -- check up front rather than after the file is open.
        if addr % block_eff or length % block_eff:
            raise ValueError(
                f"erase_first=True requires block alignment: addr 0x{addr:x} "
                f"and length 0x{length:x} must be multiples of 0x{block_eff:x} "
                f"(block_size_eff). Use erase_first=False only if the region "
                f"is already erased.")
        _prog.erase(addr, length,
                    lambda cur, tot: report(cur, tot * 3, f"erasing {_mb(cur):.1f}/{_mb(tot):.1f} MB"))

    t0 = time.monotonic()
    with open(path, "rb") as f:
        bad = _prog.write(
            addr, length, f,
            lambda cur, tot: report(tot + cur, tot * 3,
                                    f"writing {_mb(cur):.1f}/{_mb(tot):.1f} MB"))
    write_s = time.monotonic() - t0

    mismatches = 0
    first_mismatch = None
    state = {"off": 0}
    with open(path, "rb") as f:
        def compare(data):
            nonlocal mismatches, first_mismatch
            expected = f.read(len(data))
            if expected != data:
                for i, (a, b) in enumerate(zip(expected, data)):
                    if a != b:
                        mismatches += 1
                        if first_mismatch is None:
                            first_mismatch = state["off"] + i
            state["off"] += len(data)

        def vreport(cur, tot, msg="", force=False):
            report(2 * tot + cur, tot * 3,
                   f"verifying {_mb(cur):.1f}/{_mb(tot):.1f} MB", force=force)

        _read_segments(vreport, compare, addr, length, False, "verify")

    return {
        "addr": f"0x{addr:x}",
        "length": f"0x{length:x}",
        "erased": erase_first,
        "bytes_written": length,
        "bytes_verified": state["off"],
        "mismatches": mismatches,
        "first_mismatch_offset": (f"0x{first_mismatch:x}"
                                  if first_mismatch is not None else None),
        "bad_blocks_reported": [f"{kind}@0x{a:x}" for kind, a, _ in bad],
        "write_s": round(write_s, 1),
        "result": "PASS" if mismatches == 0 else "FAIL",
    }


@mcp.tool()
async def write(ctx: Context, path: str, erase_first: bool = True) -> dict:
    """Erase and write a file over the WHOLE chip in the socket,
    address-for-address (bad blocks are not skipped), then read it all back
    and verify byte for byte. The file must be exactly the chip's main+spare
    (OOB-inclusive) size -- see total_size_eff from identify(). To write only
    part of the chip and leave the rest untouched, use write_range().

    DESTRUCTIVE: erases and overwrites the physical chip. Confirm with the
    user before calling. Takes roughly 12 minutes for a 128MB-class chip
    (erase, then ~4 min write and ~4 min verify)."""
    def worker(report):
        chip = _prog.ensure_conf()
        _, _, total_eff = nc.eff_sizes(chip)
        size = os.path.getsize(path)
        if size != total_eff:
            raise ValueError(
                f"file size {size} does not match the chip's main+spare size "
                f"{total_eff} -- refusing to write. Use write_range() for a "
                f"partial image.")
        return _write_verify(report, chip, 0, total_eff, path, erase_first)
    return await _hw(ctx, worker)


@mcp.tool()
async def write_range(ctx: Context, addr: int, length: int, path: str,
                      erase_first: bool = True) -> dict:
    """Erase and write a file to [addr, addr+length) on the chip in the
    socket, address-for-address (bad blocks are not skipped), then read that
    region back and verify byte for byte. Everything outside the range is left
    untouched. addr and length are main+spare byte offsets, must be
    block-aligned when erase_first is true (page-aligned otherwise), and the
    file must be exactly `length` bytes.

    DESTRUCTIVE: erases and overwrites that region of the physical chip.
    Confirm the exact address range with the user before calling -- getting it
    wrong corrupts a region you meant to leave alone."""
    def worker(report):
        chip = _prog.ensure_conf()
        size = os.path.getsize(path)
        if size != length:
            raise ValueError(
                f"file size {size} does not match the requested length "
                f"{length} -- refusing to write")
        return _write_verify(report, chip, addr, length, path, erase_first)
    return await _hw(ctx, worker)


if __name__ == "__main__":
    mcp.run("stdio")
