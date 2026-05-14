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
from typing import Optional

import asyncio
import json as _json

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from mcp.server.fastmcp import FastMCP

from robot_mcp_bridge.event_bus import get_bus
from robot_mcp_bridge.ros_node import RosBridge

log = logging.getLogger("robot_mcp_bridge")

# ============================================================
# MCP tool surface
#
# Each tool is a thin function wrapping a RosBridge method. Tools are
# intentionally chatty in their type hints/docstrings — the docstring is
# the LLM's only documentation, so it pays to spell out what each arg
# accepts and what shape comes back.
# ============================================================

mcp = FastMCP("robot")


# ---- face / speech ---------------------------------------------------------


@mcp.tool()
def speak(text: str, language: str = "", voice: str = "") -> dict:
    """Speak `text` through the robot's TTS pipeline (fire-and-forget).

    Args:
        text: The utterance. Empty string is a no-op.
        language: 'ro' or 'en'. Empty = use current language preference.
        voice: Override the engine's default voice for the language.

    Returns:
        {"ok": bool, "text": str, "language": str} or {"ok": False, "error": str}
    """
    return RosBridge().speak(text=text, language=language, voice=voice)


@mcp.tool()
def speak_sync(text: str, timeout_seconds: float = 30.0) -> dict:
    """Speak `text` and BLOCK until synthesis finishes.

    Calls the legacy /speak service. Prefer `speak()` for low-latency
    streaming output; use this when you need to know when the utterance
    is done (e.g., to chain a follow-up action).
    """
    return RosBridge().speak_sync(text=text, timeout_seconds=timeout_seconds)


@mcp.tool()
def set_face(state: str, amplitude: float = 0.0) -> dict:
    """Set the ESP32 face state.

    Args:
        state: One of: standby, processing, speaking, aggressive.
        amplitude: 0.0-1.0 — currently only consumed by 'speaking' (mouth intensity).

    Returns:
        {"ok": bool, "state": str, "amplitude": float}
    """
    return RosBridge().set_face(state=state, amplitude=amplitude)


@mcp.tool()
def listen(timeout_seconds: float = 15.0) -> dict:
    """Wait for the next final transcript from the STT pipeline.

    Returns the transcript text or an empty result on timeout. Does not
    re-start the mic — capture is always running in audio_capture_node.
    """
    return RosBridge().listen(timeout_seconds=timeout_seconds)


# ---- identity --------------------------------------------------------------


@mcp.tool()
def who_is_here() -> dict:
    """Return the most recently identified person, if any.

    {"present": False} when nobody has been identified yet, otherwise:
    {"present": True, "person_id", "name", "voice_confidence", "is_new"}.
    """
    return RosBridge().who_is_here()


@mcp.tool()
def register_person(name: str) -> dict:
    """Enroll the currently speaking person under `name`."""
    return RosBridge().register_person(name=name)


@mcp.tool()
def list_persons() -> dict:
    """List all known persons in the identity graph."""
    return RosBridge().list_persons()


@mcp.tool()
def forget_person(person_id: str) -> dict:
    """Remove a person and their relations from the graph."""
    return RosBridge().forget_person(person_id=person_id)


# ---- location --------------------------------------------------------------


@mcp.tool()
def where_am_i() -> dict:
    """Return the current physical location of the robot.

    {"known": False} when unknown, else:
    {"known": True, "location_id", "name", "parent", "confidence"}.
    """
    return RosBridge().where_am_i()


@mcp.tool()
def learn_location(name: str, parent: str = "") -> dict:
    """Sample the current visual scene and bind it to a new named location."""
    return RosBridge().learn_location(name=name, parent=parent)


@mcp.tool()
def set_current_location(name: str, parent: str = "") -> dict:
    """Manually set the robot's current location by name (overrides recognition)."""
    return RosBridge().set_current_location(name=name, parent=parent)


@mcp.tool()
def list_locations() -> dict:
    """List all known locations in the graph."""
    return RosBridge().list_locations()


# ---- memory ----------------------------------------------------------------


@mcp.tool()
def remember(subject_id: str, subject_type: str, content: str,
              tags: str = "", source: str = "manual",
              confidence: float = 1.0) -> dict:
    """Store a fact about a subject (person, location, self, ...).

    Args:
        subject_id: Stable id of the subject (e.g., person_id).
        subject_type: 'Person' | 'Location' | 'Self' | ...
        content: The fact text.
        tags: Comma-separated tag list.
        source: Where this fact came from ('manual', 'conversation', ...).
        confidence: 0.0-1.0.
    """
    return RosBridge().remember(subject_id=subject_id, subject_type=subject_type,
                                 content=content, tags=tags, source=source,
                                 confidence=confidence)


@mcp.tool()
def recall(subject_id: str, subject_type: str = "Person",
            query: str = "", limit: int = 10) -> dict:
    """Retrieve facts about a subject, ranked by relevance to `query`."""
    return RosBridge().recall(subject_id=subject_id, subject_type=subject_type,
                               query=query, limit=limit)


@mcp.tool()
def relate_persons(a_id: str, b_id: str, relation: str,
                    description: str = "",
                    bidirectional: bool = False) -> dict:
    """Add a relationship edge between two persons in the graph.

    Args:
        a_id, b_id: Person ids.
        relation: Free-form label (e.g., 'parent_of', 'colleague_of').
        description: Optional context.
        bidirectional: If True, the relation applies in both directions.
    """
    return RosBridge().relate_persons(a_id=a_id, b_id=b_id, relation=relation,
                                       description=description,
                                       bidirectional=bidirectional)


@mcp.tool()
def find_related(subject_id: str, relation: str = "", hops: int = 1) -> dict:
    """Walk the graph from `subject_id` following edges of type `relation`."""
    return RosBridge().find_related(subject_id=subject_id,
                                     relation=relation, hops=hops)


@mcp.tool()
def cypher(query: str, params: Optional[dict] = None) -> dict:
    """Run a raw Cypher query against the knowledge graph.

    Reserved for the memorist skill / advanced users. Most callers should
    use `remember`, `recall`, `relate_persons`, `find_related` instead.
    """
    return RosBridge().cypher(query=query, params=params or {})


# ============================================================
# FastAPI app
# ============================================================


@asynccontextmanager
async def lifespan(app: FastAPI):
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
    app.mount("/mcp", mcp.sse_app())

    @app.get("/healthz")
    def healthz():
        return {"status": "ok"}

    @app.websocket("/events")
    async def events(ws: WebSocket) -> None:
        """Stream perception events as JSON lines to a single subscriber.

        Used by the Hermes ROS2 platform adapter (which runs inside Hermes
        in Python 3.11 and therefore cannot import rclpy). Multiple
        connections are supported; each gets its own queue and own pace.
        """
        await ws.accept()
        bus = get_bus()
        queue = await bus.subscribe()
        try:
            # Send a hello so the client knows the stream is alive.
            await ws.send_text(_json.dumps({"type": "hello",
                                              "schema_version": 1}))
            while True:
                event = await queue.get()
                await ws.send_text(_json.dumps(event))
        except WebSocketDisconnect:
            pass
        finally:
            await bus.unsubscribe(queue)

    return app


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="ROS2 <-> Hermes MCP bridge daemon")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--log-level", default="info")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

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
