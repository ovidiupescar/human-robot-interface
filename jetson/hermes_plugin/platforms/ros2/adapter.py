"""ROS2 perception platform adapter for Hermes.

Inherits from BasePlatformAdapter (gateway/platforms/base.py). Subscribes to
the ros2_bridge_daemon's /events WebSocket (ws://127.0.0.1:8765/events) and
translates its JSON events into MessageEvents dispatched into a single
persistent Hermes session keyed off chat_id="robot-main", so every
voice/vision/sensor event accumulates in the same conversation.

Outbound send() calls publish to /tts/say through the daemon's MCP tools —
but we keep a direct ws->ros2-bridge MCP call out of this adapter to avoid
re-implementing MCP. The skill-side path (`Hermes -> MCP tool 'speak'`) is
the canonical outbound channel; this adapter's send() falls back to the
same MCP tool when the gateway emits a textual response that should be
spoken aloud.

This adapter intentionally does NOT import rclpy: ROS2 Humble's rclpy
binary extension is built for Python 3.10 and Hermes runs on Python 3.11.
All ROS2 access happens out-of-process via the ros2_bridge_daemon.
"""

from __future__ import annotations

import asyncio
import datetime as _dt
import json
import logging
import os
import time
from typing import Any, Dict, Optional

# --- Hermes imports (gated for editor-time / lint) ---
try:
    from gateway.platforms.base import (
        BasePlatformAdapter,
        MessageEvent,
        MessageType,
        SendResult,
    )
    from gateway.session import Platform
    HERMES_AVAILABLE = True
except ImportError:
    HERMES_AVAILABLE = False
    BasePlatformAdapter = object  # type: ignore

    class MessageEvent:           # type: ignore
        def __init__(self, **kw):
            self.__dict__.update(kw)

    class MessageType:            # type: ignore
        TEXT = "text"

    class SendResult:             # type: ignore
        def __init__(self, success=False, message_id=None, error=None,
                     raw_response=None, retryable=False,
                     continuation_message_ids=()):
            self.success = success
            self.message_id = message_id
            self.error = error
            self.raw_response = raw_response
            self.retryable = retryable
            self.continuation_message_ids = continuation_message_ids

    class Platform:               # type: ignore
        def __init__(self, name):
            self.name = name


# Optional websocket client (Python 3.11 — installed when Hermes venv has it).
try:
    import websockets
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False


# Optional MCP client (Hermes ships its own — we lean on httpx fallback).
try:
    import httpx
    HTTPX_AVAILABLE = True
except ImportError:
    HTTPX_AVAILABLE = False


# Stable chat_id so all ROS2 events land in ONE persistent session.
CHAT_ID = os.environ.get("ROS2_ROBOT_CHAT_ID", "robot-main")
USER_ID = "physical-world"

# Default daemon endpoints — overridable via env for testing.
EVENTS_URL = os.environ.get("ROS2_BRIDGE_EVENTS_URL",
                              "ws://127.0.0.1:8765/events")
MCP_TTS_URL = os.environ.get("ROS2_BRIDGE_MCP_URL",
                              "http://127.0.0.1:8765/mcp-http/mcp")


log = logging.getLogger(__name__)


# ============================================================
# Event formatting — match the prompt shape the old adapter produced so
# downstream skills/prompts don't need to change.
# ============================================================


def _iso_now() -> str:
    return _dt.datetime.now(_dt.timezone.utc).isoformat(
        timespec="milliseconds").replace("+00:00", "Z")


def _format_voice(ev: Dict[str, Any]) -> tuple[str, Dict[str, Any]]:
    """Return (formatted_text, augmentations) for a 'voice' event."""
    ts = ev.get("ts") or _iso_now()
    rel = "just now"
    spk = ev.get("speaker_name") or "Unknown"
    if ev.get("speaker_id"):
        spk = f"{spk}({ev['speaker_id']})"
    loc = f" @{ev['location']}" if ev.get("location") else ""
    addr = f"addressee={ev.get('addressee_score', 0.0):.2f}"
    if ev.get("addressee_hint"):
        addr += f", hint:{ev['addressee_hint']}"
    lang_tag = f", lang={ev['lang']}" if ev.get("lang") else ""
    text = ev.get("text") or ""
    formatted = (f"[{ts} ({rel}) | USER_VOICE from {spk}{loc} | {addr}"
                 f"{lang_tag}]: {text}")
    aug = {
        "current_language": ev.get("lang") or "",
        "language_source": ev.get("language_source") or "",
        "frame_b64": ev.get("frame_b64"),
    }
    return formatted, aug


def _format_meta(ev: Dict[str, Any]) -> str:
    ts = ev.get("ts") or _iso_now()
    typ = ev.get("type") or "meta"
    extra = {k: v for k, v in ev.items() if k not in ("ts", "type")}
    if extra:
        return f"[{ts} | META {typ} {json.dumps(extra)}]"
    return f"[{ts} | META {typ}]"


# ============================================================
# The adapter Hermes instantiates
# ============================================================


class Ros2Adapter(BasePlatformAdapter):
    """Bridges the ros2_bridge_daemon to a Hermes platform session.

    No rclpy import: all ROS2 traffic flows through the daemon over its
    WebSocket (inbound events) and MCP tools (outbound speech / actions).
    """

    RECONNECT_BACKOFF_S = 2.0
    MAX_BACKOFF_S = 30.0

    def __init__(self, config, **kwargs):
        platform = Platform("ros2") if HERMES_AVAILABLE else Platform("ros2")
        if HERMES_AVAILABLE:
            super().__init__(config=config, platform=platform)
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._task: Optional[asyncio.Task] = None
        self._connected = False
        self._stop = False

    # ---- Hermes lifecycle ----

    async def connect(self) -> bool:
        if not WEBSOCKETS_AVAILABLE:
            log.error("websockets library not installed in Hermes venv — "
                      "run: pip install websockets")
            return False
        self._loop = asyncio.get_running_loop()
        self._stop = False
        self._task = asyncio.create_task(self._run_event_loop(),
                                           name="ros2-adapter-events")
        return True

    async def disconnect(self) -> None:
        self._stop = True
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
        self._task = None
        self._connected = False

    async def send(self, chat_id: str, content: str,
                   reply_to: Optional[str] = None,
                   metadata: Optional[Dict[str, Any]] = None
                   ) -> SendResult:
        if not content:
            return SendResult(success=True,
                              message_id=str(int(time.time() * 1000)))

        language = ""
        if metadata and isinstance(metadata, dict):
            language = (metadata.get("language")
                        or metadata.get("response_language") or "")

        ok = await self._call_speak_via_mcp(content, language)
        if ok:
            return SendResult(success=True,
                              message_id=str(int(time.time() * 1000)))
        return SendResult(success=False,
                           error="ros2_bridge_daemon MCP speak unreachable",
                           retryable=True)

    async def send_typing(self, chat_id: str,
                          metadata: Optional[Dict[str, Any]] = None) -> None:
        # Best-effort: ask the daemon to set face to PROCESSING.
        # Failures are non-fatal; the reflex node also drives the face.
        await self._call_mcp_tool("set_face", {"state": "processing",
                                                 "amplitude": 0.0})

    async def send_image(self, chat_id: str, image_url: str,
                         caption: Optional[str] = None,
                         reply_to: Optional[str] = None,
                         metadata: Optional[Dict[str, Any]] = None
                         ) -> SendResult:
        # Robot has no image output channel yet.
        return SendResult(success=False,
                           error="no image output on this platform",
                           retryable=False)

    async def get_chat_info(self, chat_id: str) -> Dict[str, Any]:
        return {"name": "Robot physical session", "type": "dm",
                "chat_id": chat_id}

    # ---- internal: event loop on /events WebSocket ----

    async def _run_event_loop(self) -> None:
        backoff = self.RECONNECT_BACKOFF_S
        while not self._stop:
            try:
                async with websockets.connect(EVENTS_URL,
                                                open_timeout=5) as ws:
                    self._connected = True
                    backoff = self.RECONNECT_BACKOFF_S
                    log.info("connected to %s", EVENTS_URL)
                    async for raw in ws:
                        try:
                            ev = json.loads(raw)
                        except json.JSONDecodeError:
                            continue
                        await self._handle_event(ev)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self._connected = False
                if self._stop:
                    break
                log.warning("events WS error: %s — retrying in %.1fs",
                             exc, backoff)
                await asyncio.sleep(backoff)
                backoff = min(self.MAX_BACKOFF_S, backoff * 2)

    async def _handle_event(self, ev: Dict[str, Any]) -> None:
        typ = ev.get("type")
        if typ == "hello":
            log.info("daemon hello, schema_version=%s",
                     ev.get("schema_version"))
            return
        if typ == "voice":
            raw_text = (ev.get("text") or "").strip()
            # Slash commands (e.g. /sethome, /reset) MUST reach Hermes's
            # command dispatcher unwrapped — otherwise they read as natural
            # language and the agent treats them as a tool-calling prompt.
            # Hermes routes commands by detecting a leading '/' at position 0.
            if raw_text.startswith("/"):
                self._dispatch(raw_text, augmentations=None)
                return
            text, aug = _format_voice(ev)
            self._dispatch(text, augmentations=aug)
            return
        if typ == "wake_word":
            # Wake events are an internal activation cue: they OPEN the
            # daemon's window so the next 'voice' events get forwarded.
            # They are NOT themselves user utterances — if we dispatch them
            # into the Hermes session the agent will helpfully reply
            # "I'm listening" to every wake, which then becomes a TTS turn
            # the mic-duck won't fully absorb.
            log.info("wake_word fired (model=%s, window=%.1fs)",
                     ev.get("model"), ev.get("window_s", 0))
            return
        # The remaining events (voice_started, voice_ended,
        # language_changed, person_identified, location_changed,
        # context_warm) are silent in the session for now — they're useful
        # diagnostics in the /events feed but the agent doesn't need to
        # see them as user messages. Re-enable on a case-by-case basis if
        # downstream skills want them.
        return

    def _dispatch(self, formatted_text: str,
                   augmentations: Optional[Dict[str, Any]] = None) -> None:
        if not HERMES_AVAILABLE:
            return
        source = self.build_source(
            chat_id=CHAT_ID,
            chat_name="Robot",
            chat_type="dm",
            user_id=USER_ID,
            user_name="Physical World",
        )
        # MessageEvent doesn't expose a metadata field; the closest typed slot
        # is `raw_message` (Any). Stash augmentations there so downstream
        # skills can pull them off without us inventing a new attribute.
        evt = MessageEvent(
            text=formatted_text,
            message_type=MessageType.TEXT,
            source=source,
            message_id=str(int(time.time() * 1000)),
            timestamp=_dt.datetime.now(),
            raw_message={"augmentations": augmentations} if augmentations else None,
        )
        asyncio.create_task(self.handle_message(evt))

    # ---- outbound: MCP tool call over Streamable HTTP ----
    #
    # Streamable HTTP requires an `initialize` exchange before tools/call
    # is accepted; the server returns an Mcp-Session-Id header that must
    # be sent on every subsequent request. We do it ourselves over httpx
    # to keep the dependency surface minimal (no full mcp.client SDK).

    _next_id = 1
    _session_id: Optional[str] = None
    _session_lock: Optional[asyncio.Lock] = None

    def _ensure_session_lock(self) -> asyncio.Lock:
        if self._session_lock is None:
            self._session_lock = asyncio.Lock()
        return self._session_lock

    async def _ensure_mcp_session(self) -> None:
        async with self._ensure_session_lock():
            if self._session_id is not None:
                return
            init = {
                "jsonrpc": "2.0",
                "id": self._next_id,
                "method": "initialize",
                "params": {
                    "protocolVersion": "2025-03-26",
                    "capabilities": {},
                    "clientInfo": {"name": "ros2-platform-adapter",
                                    "version": "0.2.0"},
                },
            }
            self._next_id += 1
            headers = {
                "Content-Type": "application/json",
                "Accept": "application/json, text/event-stream",
            }
            async with httpx.AsyncClient(timeout=5.0) as client:
                resp = await client.post(MCP_TTS_URL, json=init,
                                           headers=headers)
                resp.raise_for_status()
                sid = (resp.headers.get("Mcp-Session-Id")
                       or resp.headers.get("mcp-session-id"))
                if sid:
                    self._session_id = sid
                # Fire the 'initialized' notification per spec.
                notify = {
                    "jsonrpc": "2.0",
                    "method": "notifications/initialized",
                    "params": {},
                }
                hdrs2 = dict(headers)
                if sid:
                    hdrs2["Mcp-Session-Id"] = sid
                await client.post(MCP_TTS_URL, json=notify, headers=hdrs2)

    async def _call_mcp_tool(self, name: str, arguments: Dict[str, Any]
                              ) -> bool:
        if not HTTPX_AVAILABLE:
            log.error("httpx not installed in Hermes venv")
            return False
        try:
            await self._ensure_mcp_session()
        except Exception as exc:
            log.warning("MCP initialize failed: %s", exc)
            return False

        payload = {
            "jsonrpc": "2.0",
            "id": self._next_id,
            "method": "tools/call",
            "params": {"name": name, "arguments": arguments},
        }
        self._next_id += 1
        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json, text/event-stream",
        }
        if self._session_id:
            headers["Mcp-Session-Id"] = self._session_id
        try:
            async with httpx.AsyncClient(timeout=10.0) as client:
                resp = await client.post(MCP_TTS_URL, json=payload,
                                           headers=headers)
                resp.raise_for_status()
                return True
        except Exception as exc:
            log.warning("MCP %s call failed: %s", name, exc)
            # Forget the session so we'll re-initialize on retry.
            self._session_id = None
            return False

    async def _call_speak_via_mcp(self, text: str, language: str = "") -> bool:
        return await self._call_mcp_tool("speak",
                                           {"text": text,
                                            "language": language})


# ============================================================
# Plugin registration entry point — Hermes calls this on load
# ============================================================


def _env_enablement() -> dict | None:
    """Hermes-side enablement gate.

    The robot is the only thing this Hermes ever talks to; we always want
    the platform alive. Return a non-empty dict so the gateway treats the
    platform as 'enabled' without requiring per-deployment env vars.
    """
    return {
        "chat_id": CHAT_ID,
        "events_url": EVENTS_URL,
        "mcp_url": MCP_TTS_URL,
    }


def _check_requirements() -> bool:
    """Adapter is healthy iff the websockets+httpx clients are importable.
    The actual connectivity to ros2_bridge_daemon is exercised at connect()
    time with a reconnect loop, so a temporarily-down daemon doesn't fail
    plugin load.
    """
    return WEBSOCKETS_AVAILABLE and HTTPX_AVAILABLE


def register(ctx):
    """Hermes plugin entry point. `ctx` is the platform-registration context."""
    ctx.register_platform(
        name="ros2",
        label="ROS2",
        adapter_factory=lambda config: Ros2Adapter(config),
        check_fn=_check_requirements,
        env_enablement_fn=_env_enablement,
        install_hint=(
            "pip install websockets httpx; ensure ros2_bridge_daemon is "
            "running on http://127.0.0.1:8765"
        ),
    )
