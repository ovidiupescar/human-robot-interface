"""Bridge daemon entry point.

Hosts a FastAPI app on 127.0.0.1:8765 with:
  * MCP HTTP+SSE server mounted at /mcp/ — Hermes connects as a client
  * (future) WebSocket events endpoint at /events for the platform adapter

Run:
  ros2 run robot_mcp_bridge daemon
or directly under systemd via the unit shipped in the environment repo.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI
from mcp.server.fastmcp import FastMCP

from robot_mcp_bridge.ros_node import RosBridge

log = logging.getLogger("robot_mcp_bridge")

# --- MCP tool surface -------------------------------------------------------

mcp = FastMCP("robot")


@mcp.tool()
def speak(text: str, language: str = "", voice: str = "") -> dict:
    """Speak `text` through the robot's TTS pipeline.

    Args:
        text: The utterance to synthesize. Empty string is a no-op.
        language: 'ro' or 'en'. Empty = use current language preference.
        voice: Override the engine's default voice for the chosen language.
               Empty = engine default.

    Returns:
        {"ok": bool, "text": str, "language": str}
    """
    return RosBridge().speak(text=text, language=language, voice=voice)


# --- FastAPI app ------------------------------------------------------------


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Eagerly initialize the rclpy node so the first MCP call doesn't pay
    # the startup cost. Errors here are fatal — let systemd restart us.
    log.info("starting RosBridge")
    RosBridge()
    log.info("RosBridge ready")
    try:
        yield
    finally:
        log.info("shutting down RosBridge")
        RosBridge().shutdown()


def build_app() -> FastAPI:
    app = FastAPI(
        title="robot_mcp_bridge",
        version="0.1.0",
        lifespan=lifespan,
    )

    # Mount the MCP HTTP+SSE app under /mcp. Hermes config will point at
    # http://127.0.0.1:8765/mcp/sse for the SSE stream.
    app.mount("/mcp", mcp.sse_app())

    @app.get("/healthz")
    def healthz():
        return {"status": "ok"}

    return app


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="ROS2 <-> Hermes MCP bridge daemon")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Bind host (loopback only by default).")
    parser.add_argument("--port", type=int, default=8765,
                        help="Bind port (default 8765).")
    parser.add_argument("--log-level", default="info",
                        help="uvicorn log level: debug|info|warning|error")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    # Make SIGTERM (systemd) and SIGINT (terminal) hand off to uvicorn's
    # signal handlers cleanly.
    def _handle_signal(signum, _frame):
        log.info("received signal %s, exiting", signum)
        sys.exit(0)

    signal.signal(signal.SIGTERM, _handle_signal)

    config = uvicorn.Config(
        build_app(),
        host=args.host,
        port=args.port,
        log_level=args.log_level,
        access_log=False,
    )
    uvicorn.Server(config).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
