# nando-mcp

An MCP server wrapping the NANDO programmer's USB protocol, so an MCP client
(e.g. Claude Code) can drive the hardware directly instead of re-deriving the
wire protocol from scratch each session.

Two modules:

* `nando_client.py` -- the protocol. Structs mirroring `qt/cmd.h` and
  `firmware/programmer/nand_programmer.c`, the chip DB parser, the FSMC timing
  arithmetic from the Qt host app, and a `Programmer` class with the actual
  operations. Blocking, no MCP dependency, usable from a plain script.
* `server.py` -- the MCP adapter. Concurrency, progress reporting, file
  layout. Nothing protocol-specific.

## Setup

```
python3 -m venv venv
./venv/bin/pip install -r requirements.txt
```

The invoking user needs `dialout` group membership for `/dev/ttyACM*` access.
Group changes only take effect in a fresh login shell/session.

## Running

```
./venv/bin/python server.py
```

Speaks MCP over stdio. Register it with a client by pointing it at that
command. The device is located by USB VID/PID (0483:5740) at call time, not at
import, so the server starts fine with the programmer unplugged and picks it up
whenever it appears -- including at a different `/dev/ttyACM*` node.

## Tools

Read-only:

- `identify()` -- firmware version, chip ID, matched chip DB entry, and both
  the datasheet sizes and the effective (main+spare) sizes.
- `conn_check()` -- fast scattered-sample connectivity/seating check.
- `bad_block_scan()` -- bad block list; falls back to a raw OOB-marker scan if
  the firmware's 20-entry bad block table overflows.
- `read_range(addr, length, out_path, skip_bb)` -- read a region; writes a file
  if `out_path` is given, otherwise reports sha256 and a hex preview.
- `dump_full(label, raw)` -- full-chip dump to a file, with sha256. Written to
  a `.part` file and renamed on success, so a failed dump never leaves a
  truncated file under the real name.
- `reset_link()` -- drop and reopen the USB connection, forgetting the cached
  chip configuration. For after a replug or a chip swap.

Destructive -- confirm with the user before calling:

- `write(path, erase_first)` -- erase + write + verify over the whole chip.
  The file must be exactly the chip's main+spare size.
- `write_range(addr, length, path, erase_first)` -- the same over
  `[addr, addr+length)` only; everything outside is untouched.

### Addressing

Every address and length is in **main+spare (OOB-inclusive)** terms, because
every command is issued with the firmware's `inc_spare` flag. For a 2048+64
chip the effective page is 2112 bytes and the effective block 135168 -- neither
is a power of two, so round address arithmetic in hex will usually be
misaligned. `identify()` reports `page_size_eff` / `block_size_eff` /
`total_size_eff`; use them.

Reads and writes need page alignment. **Erase needs block alignment**, so
`write_range(..., erase_first=True)` requires block-aligned bounds.

## Design notes

These are the two things that make the difference between "usually works" and
"trustworthy", and both are easy to get wrong:

**One exclusive link.** The port is opened with `TIOCEXCL`, so a stray second
process cannot interleave bytes into the same stream, and all tools serialise
on one lock. `TIOCEXCL` is explicitly cleared on close: the kernel only drops
it when the *last* fd on the tty closes, so leaking an fd elsewhere would
otherwise lock the device out until the holder exits.

**Resynchronisation after a failure.** The firmware finishes emitting the rest
of an aborted READ no matter what the host does, and reopening the port does
*not* discard bytes already queued in the tty. So any failed command drains the
device until it goes quiet -- with a time budget derived from how much the
command still owed -- flushes, and then proves the stream is clean by
round-tripping a VERSION_GET. Without this, a retry reads the tail of the
previous command and writes plausible-looking but wrong bytes into the middle
of a dump. `Programmer.open()` does the same drain, because the previous
process may have been killed mid-read.

**Nothing blocks the event loop.** Hardware I/O runs in a worker thread with
progress bridged back, so the server keeps answering the client (including
cancellation) throughout a multi-minute dump.

Throughput is hardware-limited to roughly 0.6 MB/s (USB CDC, 62-byte protocol
chunks) regardless of this server.

## Testing

`test_client.py` is a real client-server smoke test: it spawns `server.py` as a
subprocess over stdio exactly as a real MCP client would, exercises the
read-only tools, and asserts the two properties a single sequential call cannot
show -- that progress notifications arrive *during* an operation, and that the
server keeps answering other requests while one runs.

```
./venv/bin/python test_client.py          # read-only, ~30 s
./venv/bin/python test_client.py --dump   # also a full-chip dump, ~4 min
```
