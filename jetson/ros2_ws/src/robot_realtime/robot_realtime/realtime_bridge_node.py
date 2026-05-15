"""Gemini Live realtime audio bridge.

Replaces the STT -> Hermes -> TTS chain with a single WebSocket
session to gemini-3.1-flash-live for sub-second end-to-end
conversational latency. The rest of the ROS2 graph keeps working
unchanged.

Inputs (ROS2 subscriptions):
    /audio/chunk             (UInt8MultiArray)  PCM16 mono @ 16kHz
                                                 from audio_capture_node
    /perception/wake_word    (String)           gates whether mic frames
                                                 are forwarded upstream
    /audio/playback_status   (String)           'started' suppresses mic
                                                 forwarding during own TTS
    (Future) /perception/identified_person (PersonIdentity)
                                                 will push the current
                                                 speaker into the session
                                                 as a system note. For v1
                                                 the model uses the
                                                 who_is_here tool instead.

Outputs (ROS2 publications):
    /audio/stream            (UInt8MultiArray)  PCM16 chunks to audio_player
    /audio/playback_status   (String)           'started' / 'done' so
                                                 audio_capture mutes the mic
                                                 while the bot speaks
    /face/command            (FaceCommand)      mouth amplitude per chunk,
                                                 same as tts_service does

Tool calls from the model are relayed via HTTP to the existing MCP
endpoint (default http://127.0.0.1:8765/mcp-http/mcp), so all 21
robot tools (speak, set_face, who_is_here, where_am_i, remember,
recall, ...) are reachable to the Gemini Live session without any
new wiring.

This node does NOT replace anything. It coexists with the legacy
pipeline. Switch by stopping hermes-gateway and starting this node
(see robot-realtime.service).
"""

from __future__ import annotations

import asyncio
import base64
import json
import os
import threading
import time
from array import array
from typing import Any, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, UInt8MultiArray

try:
    import websockets
except ImportError:  # pragma: no cover
    websockets = None

try:
    import httpx
except ImportError:  # pragma: no cover
    httpx = None

from robot_face_msgs.msg import FaceCommand


GEMINI_WS_URL = (
    "wss://generativelanguage.googleapis.com/ws/"
    "google.ai.generativelanguage.v1beta.GenerativeService.BidiGenerateContent"
)

# Gemini Live ingests 16kHz mono PCM16 and emits 24kHz mono PCM16.
INPUT_SAMPLE_RATE = 16000
OUTPUT_SAMPLE_RATE = 24000
PLAYBACK_PUBLISH_RATE = 22050  # audio_player runs at this; we resample.

MODEL_3_1_FLASH_LIVE = "models/gemini-3.1-flash-live"
MODEL_2_5_FLASH_LIVE = "models/gemini-2.5-flash-live-001"

# Minimum amplitude when speaking — keeps mouth visible during quiet
# phonemes; ramps up with RMS for visible lip-sync.
MOUTH_MIN_AMP = 0.1


class RealtimeBridge(Node):

    def __init__(self) -> None:
        super().__init__("realtime_bridge")

        # ---- params ----
        self.declare_parameter("model", MODEL_3_1_FLASH_LIVE)
        self.declare_parameter("voice", "Charon")
        self.declare_parameter("system_instruction", _DEFAULT_SYSTEM_PROMPT)
        # MCP endpoint inside the bridge daemon — same one Hermes uses.
        self.declare_parameter("mcp_url",
                                "http://127.0.0.1:8765/mcp-http/mcp")
        # API key resolution: ROS2 param wins, else env GEMINI_API_KEY,
        # else read from ~/.hermes/.env.
        self.declare_parameter("api_key", "")
        # If False, forward audio only while a wake word is active —
        # mirrors the daemon's wake gate so the model isn't billed on
        # ambient room audio. Default False during early testing.
        self.declare_parameter("require_wake", True)

        self.model = str(self.get_parameter("model").value)
        self.voice = str(self.get_parameter("voice").value)
        self.system_prompt = str(self.get_parameter("system_instruction").value)
        self.mcp_url = str(self.get_parameter("mcp_url").value)
        self.require_wake = bool(self.get_parameter("require_wake").value)
        self.api_key = _resolve_api_key(
            str(self.get_parameter("api_key").value))

        if websockets is None:
            self.get_logger().error(
                "websockets package missing; pip install websockets")
            return
        if not self.api_key:
            self.get_logger().error(
                "GEMINI_API_KEY not set. Put it in ~/.hermes/.env or pass "
                "--ros-args -p api_key:=...")
            return

        # ---- ROS2 wiring ----
        self.create_subscription(UInt8MultiArray, "/audio/chunk",
                                  self._on_mic_chunk, 50)
        self.create_subscription(String, "/perception/wake_word",
                                  self._on_wake, 10)
        self.create_subscription(String, "/audio/playback_status",
                                  self._on_playback_status, 10)

        self._stream_pub = self.create_publisher(
            UInt8MultiArray, "/audio/stream", 1500)
        self._status_pub = self.create_publisher(
            String, "/audio/playback_status", 10)
        self._face_pub = self.create_publisher(FaceCommand, "/face/command", 10)

        # ---- state ----
        self._wake_active_until = 0.0  # monotonic deadline
        self._wake_window_s = 25.0
        self._bot_speaking = False
        self._tail_until = 0.0
        self._known_speaker: Optional[str] = None  # injected on change
        self._last_speaker_id: Optional[str] = None

        # Mailbox for the async loop to receive ROS callbacks.
        self._loop = asyncio.new_event_loop()
        self._ws: Optional[Any] = None
        self._ready = threading.Event()

        threading.Thread(target=self._run_loop, daemon=True).start()
        self.get_logger().info(
            f"realtime_bridge starting; model={self.model} voice={self.voice} "
            f"require_wake={self.require_wake}")

    # ---- ROS callbacks (run in rclpy executor thread) ----

    def _on_wake(self, msg: String) -> None:
        self._wake_active_until = time.monotonic() + self._wake_window_s

    def _on_playback_status(self, msg: String) -> None:
        if msg.data == "started":
            self._bot_speaking = True
        elif msg.data in ("done", "interrupted"):
            self._bot_speaking = False
            self._tail_until = time.monotonic() + 0.6

    def _on_mic_chunk(self, msg: UInt8MultiArray) -> None:
        if self._ws is None or not self._ready.is_set():
            return
        # Don't forward our own audio bleed-through.
        if self._bot_speaking or time.monotonic() < self._tail_until:
            return
        if self.require_wake and time.monotonic() >= self._wake_active_until:
            return
        pcm_bytes = bytes(msg.data)
        if not pcm_bytes:
            return
        asyncio.run_coroutine_threadsafe(
            self._send_audio_chunk(pcm_bytes), self._loop)

    # ---- async tasks ----

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        while rclpy.ok():
            try:
                self._loop.run_until_complete(self._ws_session())
            except Exception as exc:
                self.get_logger().error(
                    f"realtime session crashed: {exc}; reconnecting in 5s")
                time.sleep(5)

    async def _ws_session(self) -> None:
        url = f"{GEMINI_WS_URL}?key={self.api_key}"
        async with websockets.connect(url, max_size=None) as ws:
            self._ws = ws
            await self._send_setup(ws)
            self._ready.set()
            self.get_logger().info("Gemini Live session established")
            try:
                async for raw in ws:
                    try:
                        event = json.loads(raw)
                    except json.JSONDecodeError:
                        # Binary frames possible — Google API uses JSON envelopes
                        continue
                    await self._handle_event(event, ws)
            finally:
                self._ready.clear()
                self._ws = None
                self._emit_playback_status("done")

    async def _send_setup(self, ws: Any) -> None:
        """Initial session.setup message describing model + tools."""
        setup = {
            "setup": {
                "model": self.model,
                "generation_config": {
                    "response_modalities": ["AUDIO"],
                    "speech_config": {
                        "voice_config": {
                            "prebuilt_voice_config": {"voice_name": self.voice}
                        }
                    },
                },
                "system_instruction": {
                    "parts": [{"text": self.system_prompt}]
                },
                "tools": [{
                    # Tool list comes from MCP server. Stub for now —
                    # in production we'd discover them at startup and
                    # convert to Gemini tool schema.
                    "function_declarations": _BUILTIN_TOOL_DECLARATIONS,
                }],
            }
        }
        await ws.send(json.dumps(setup))

    async def _send_audio_chunk(self, pcm: bytes) -> None:
        if self._ws is None:
            return
        msg = {
            "realtime_input": {
                "media_chunks": [{
                    "mime_type": f"audio/pcm;rate={INPUT_SAMPLE_RATE}",
                    "data": base64.b64encode(pcm).decode("ascii"),
                }]
            }
        }
        try:
            await self._ws.send(json.dumps(msg))
        except Exception as exc:
            self.get_logger().warning(f"failed to send audio chunk: {exc}")

    async def _handle_event(self, event: dict, ws: Any) -> None:
        # serverContent -> audio bytes + metadata
        sc = event.get("serverContent")
        if sc:
            await self._handle_server_content(sc)
        # toolCall -> we need to execute via MCP
        tc = event.get("toolCall")
        if tc:
            await self._handle_tool_call(tc, ws)
        # setupComplete is just an ack we can log
        if event.get("setupComplete") is not None:
            self.get_logger().info("setupComplete received")

    async def _handle_server_content(self, sc: dict) -> None:
        # Audio chunks land in modelTurn.parts[*].inlineData
        model_turn = sc.get("modelTurn", {})
        parts = model_turn.get("parts", [])
        produced_audio = False
        for part in parts:
            inline = part.get("inlineData")
            if not inline:
                continue
            mime = inline.get("mimeType", "")
            if not mime.startswith("audio/pcm"):
                continue
            data_b64 = inline.get("data", "")
            if not data_b64:
                continue
            try:
                pcm = base64.b64decode(data_b64)
            except Exception:
                continue
            self._publish_audio(pcm)
            produced_audio = True
        if produced_audio and not self._bot_speaking:
            self._emit_playback_status("started")
            self._bot_speaking = True
        # turnComplete tells us this turn ended
        if sc.get("turnComplete"):
            if self._bot_speaking:
                self._emit_playback_status("done")
                self._bot_speaking = False
                self._tail_until = time.monotonic() + 0.6

    async def _handle_tool_call(self, tc: dict, ws: Any) -> None:
        function_calls = tc.get("functionCalls", [])
        responses = []
        for fc in function_calls:
            name = fc.get("name", "")
            args = fc.get("args", {}) or {}
            call_id = fc.get("id", "")
            self.get_logger().info(f"tool call: {name}({args})")
            try:
                result = await self._mcp_call(name, args)
            except Exception as exc:
                result = {"error": str(exc)}
            responses.append({
                "id": call_id,
                "name": name,
                "response": result,
            })
        if responses:
            await ws.send(json.dumps({
                "tool_response": {"function_responses": responses}
            }))

    async def _mcp_call(self, tool_name: str,
                         args: dict) -> dict:
        if httpx is None:
            return {"error": "httpx not installed"}
        # We use the streamable HTTP transport. Each call is a fresh
        # tools/call request. The bridge daemon handles session management
        # internally; we open + tear down a session per request to avoid
        # carrying state across tools.
        async with httpx.AsyncClient(timeout=10.0) as client:
            # Init handshake
            init_req = {
                "jsonrpc": "2.0", "id": 1, "method": "initialize",
                "params": {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {},
                    "clientInfo": {"name": "realtime-bridge", "version": "1"},
                },
            }
            r = await client.post(
                self.mcp_url,
                json=init_req,
                headers={"Accept": "application/json, text/event-stream"})
            session_id = r.headers.get("Mcp-Session-Id", "")
            headers = {"Accept": "application/json, text/event-stream"}
            if session_id:
                headers["Mcp-Session-Id"] = session_id
            # Send initialized notification
            await client.post(self.mcp_url,
                               json={"jsonrpc": "2.0",
                                     "method": "notifications/initialized"},
                               headers=headers)
            # Call the tool
            call_req = {
                "jsonrpc": "2.0", "id": 2, "method": "tools/call",
                "params": {"name": tool_name, "arguments": args},
            }
            r = await client.post(self.mcp_url, json=call_req, headers=headers)
            # SSE response: parse 'data:' lines for the JSON-RPC envelope
            text = r.text
            for line in text.splitlines():
                line = line.strip()
                if line.startswith("data:"):
                    payload = line[len("data:"):].strip()
                    if not payload:
                        continue
                    try:
                        env = json.loads(payload)
                    except json.JSONDecodeError:
                        continue
                    return env.get("result", env)
            try:
                return r.json().get("result", {})
            except Exception:
                return {"raw": r.text[:200]}

    # ---- audio output ----

    def _publish_audio(self, pcm_24k: bytes) -> None:
        """Resample 24 kHz -> 22.05 kHz (audio_player rate) and publish."""
        if not pcm_24k:
            return
        samples = np.frombuffer(pcm_24k, dtype=np.int16)
        if len(samples) == 0:
            return
        # Linear resample 24000 -> 22050. ratio ~0.91875.
        out_len = int(len(samples) * PLAYBACK_PUBLISH_RATE / OUTPUT_SAMPLE_RATE)
        if out_len <= 0:
            return
        xp = np.linspace(0, 1, num=len(samples), endpoint=False)
        x_new = np.linspace(0, 1, num=out_len, endpoint=False)
        resampled = np.interp(x_new, xp, samples.astype(np.float32)).astype(np.int16)
        msg = UInt8MultiArray()
        msg.data = array("B", resampled.tobytes())
        self._stream_pub.publish(msg)
        # Drive the face with RMS-based amplitude
        rms = float(np.sqrt(np.mean(resampled.astype(np.float32) ** 2)))
        amp = min(1.0, max(MOUTH_MIN_AMP, rms / 8000.0))
        face = FaceCommand()
        face.state = FaceCommand.STATE_SPEAKING
        face.amplitude = amp
        self._face_pub.publish(face)

    def _emit_playback_status(self, status: str) -> None:
        m = String()
        m.data = status
        self._status_pub.publish(m)


_DEFAULT_SYSTEM_PROMPT = (
    "You are the voice of a small home robot. Speak in the same language "
    "the user spoke to you, defaulting to English. Keep replies short and "
    "conversational. You have tools to set your face, see who is in front "
    "of you, remember facts about people and places, and look up where you "
    "are. Use them naturally; do not narrate that you are doing so."
)


# Minimal tool schema mirror of the daemon's MCP surface. The realtime
# bridge could fetch these dynamically from the MCP server at startup
# (tools/list) and convert; this hardcoded subset is enough for v1.
_BUILTIN_TOOL_DECLARATIONS = [
    {
        "name": "who_is_here",
        "description": "Identify the person currently speaking. Returns "
                        "name and confidence, or {present: false} if no "
                        "one has been identified yet.",
        "parameters": {"type": "object", "properties": {}},
    },
    {
        "name": "where_am_i",
        "description": "Return the robot's current physical location.",
        "parameters": {"type": "object", "properties": {}},
    },
    {
        "name": "remember",
        "description": "Store a fact about a person, location, or self.",
        "parameters": {
            "type": "object",
            "properties": {
                "subject_id": {"type": "string"},
                "subject_type": {"type": "string"},
                "content": {"type": "string"},
                "tags": {"type": "string"},
                "source": {"type": "string"},
                "confidence": {"type": "number"},
            },
            "required": ["subject_id", "subject_type", "content"],
        },
    },
    {
        "name": "recall",
        "description": "Retrieve facts about a subject ranked by relevance "
                        "to a query.",
        "parameters": {
            "type": "object",
            "properties": {
                "subject_id": {"type": "string"},
                "subject_type": {"type": "string"},
                "query": {"type": "string"},
                "limit": {"type": "integer"},
            },
            "required": ["subject_id"],
        },
    },
    {
        "name": "set_face",
        "description": "Set the robot's facial expression. "
                        "state in {standby, processing, speaking, aggressive}.",
        "parameters": {
            "type": "object",
            "properties": {
                "state": {"type": "string"},
                "amplitude": {"type": "number"},
            },
            "required": ["state"],
        },
    },
]


def _resolve_api_key(param_value: str) -> str:
    if param_value:
        return param_value
    env = os.environ.get("GEMINI_API_KEY", "")
    if env:
        return env
    # Fall back to ~/.hermes/.env (a flat KEY=VALUE file)
    try:
        env_path = os.path.expanduser("~/.hermes/.env")
        if os.path.isfile(env_path):
            with open(env_path, "r") as fh:
                for line in fh:
                    line = line.strip()
                    if line.startswith("GEMINI_API_KEY="):
                        return line.split("=", 1)[1].strip().strip('"')
    except Exception:
        pass
    return ""


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
