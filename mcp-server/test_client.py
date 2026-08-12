"""Real client-server smoke test: spawns server.py as a subprocess over
stdio, exactly how Claude Code would use it, and calls dump_full with a
live progress callback."""
import asyncio
import sys

from mcp import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client


async def on_progress(progress: float, total: float | None, message: str | None):
    pct = (progress / total * 100) if total else 0
    print(f"  progress: {message} ({pct:.1f}%)", flush=True)


async def main():
    params = StdioServerParameters(
        command=sys.executable,
        args=["server.py"],
    )
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            tools = await session.list_tools()
            print("Connected. Tools:", [t.name for t in tools.tools])

            result = await session.call_tool(
                "dump_full",
                {"label": "MCPTEST", "raw": True},
                progress_callback=on_progress,
            )
            print("dump_full result:", result.content[0].text)


if __name__ == "__main__":
    asyncio.run(main())
