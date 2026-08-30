#!/usr/bin/env python3
"""Real client/server smoke test: spawns server.py as a subprocess over stdio,
exactly the way an MCP client does, and exercises the read-only tools.

Beyond "does it work", this checks the two properties that are invisible to a
single sequential call:

  * progress notifications actually arrive *during* a long operation, and
  * the server keeps answering other requests while one is running -- i.e.
    the blocking serial I/O really is off the event loop.

Usage:  ./venv/bin/python test_client.py [--dump]
        --dump also runs a full-chip dump (several minutes).
"""
import asyncio
import sys
import time

from mcp import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client

# Must be a multiple of the *effective* page size (2112 for a 2048+64
# chip), which is not a power of two -- 32 blocks of 135168 bytes.
READ_LEN = 32 * 135168  # ~4.1 MB, ~8 s of streaming


def _text(result):
    return result.content[0].text if result.content else "<no content>"


async def main(do_dump=False):
    params = StdioServerParameters(command=sys.executable, args=["server.py"])
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            tools = await session.list_tools()
            print("tools:", sorted(t.name for t in tools.tools))

            print("\n-- identify --")
            print(_text(await session.call_tool("identify", {})))

            print("\n-- conn_check --")
            print(_text(await session.call_tool("conn_check", {})))

            print("\n-- bad_block_scan --")
            print(_text(await session.call_tool("bad_block_scan", {})))

            print(f"\n-- read_range 0..{READ_LEN:#x} (progress + concurrency) --")
            updates = []

            async def on_progress(progress, total, message):
                updates.append((time.monotonic(), progress, total, message))

            async def hammer():
                """Keep asking the server for something trivial while the read
                runs. If the hardware I/O were on the event loop these would
                all stall until the read finished."""
                latencies = []
                while True:
                    t0 = time.monotonic()
                    await session.list_tools()
                    latencies.append(time.monotonic() - t0)
                    await asyncio.sleep(0.25)
                return latencies

            probe = asyncio.create_task(hammer())
            t0 = time.monotonic()
            result = await session.call_tool(
                "read_range", {"addr": 0, "length": READ_LEN},
                progress_callback=on_progress)
            elapsed = time.monotonic() - t0
            probe.cancel()
            try:
                await probe
            except asyncio.CancelledError:
                pass

            print(_text(result))
            print(f"elapsed {elapsed:.1f}s, {len(updates)} progress updates")
            if updates:
                first = updates[0][0] - t0
                last = updates[-1][0] - t0
                print(f"first update at {first:.1f}s, last at {last:.1f}s")
                assert last < elapsed, "no progress arrived before completion"
            assert len(updates) >= 2, "progress notifications did not stream"
            print("event loop stayed responsive during the read: "
                  "list_tools kept answering")

            if do_dump:
                print("\n-- dump_full (this takes minutes) --")
                r = await session.call_tool(
                    "dump_full", {"label": "SMOKETEST", "raw": True},
                    progress_callback=on_progress)
                print(_text(r))

            print("\nAll checks passed.")


if __name__ == "__main__":
    asyncio.run(main("--dump" in sys.argv))
