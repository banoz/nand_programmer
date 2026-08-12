# nando-mcp

An MCP server wrapping the NANDO programmer's USB protocol, so an MCP
client (e.g. Claude Code) can drive the hardware directly instead of
re-deriving the wire protocol from scratch each session.

Built directly on the protocol logic in `nando_client.py` -- no vendored
copy of the firmware's command definitions, just Python structs matching
`qt/cmd.h` and `firmware/programmer/nand_programmer.c`.

## Setup

```
python3 -m venv venv
./venv/bin/pip install -r requirements.txt
```

The invoking user needs `dialout` group membership for `/dev/ttyACM*`
access. Group changes only take effect in a fresh login shell/session.

## Running

```
./venv/bin/python server.py
```

Speaks MCP over stdio. Register it with an MCP client by pointing it at
that command.

## Tools

- `identify()` -- firmware version, chip ID, matched chip DB entry. Read-only.
- `conn_check()` -- fast (few-second) scattered-sample connectivity/seating
  check. Read-only.
- `bad_block_scan()` -- bad block list; falls back to a raw OOB-marker scan
  if the firmware's 20-entry bad-block table overflows. Read-only.
- `dump_full(label, raw)` -- full-chip dump to a file, with progress
  reported over MCP. Read-only.
- `write(path, erase_first)` -- erase + write + verify a file to the chip,
  address-for-address. **Destructive** -- confirm with the user before
  calling.

The physical device only supports one chip in the socket at a time and one
serial connection; the server holds a single shared connection, reopened
lazily on error.

Throughput is hardware-limited to roughly 0.5-0.6 MB/s (USB CDC, 62-byte
protocol chunks) regardless of this server -- the win here is not
re-deriving/debugging the protocol by hand each session, not speed.

## Testing

`test_client.py` is a real client-server smoke test: spawns `server.py` as
a subprocess over stdio exactly as a real MCP client would, and exercises
`dump_full` with a live progress callback.
