"""Gemini Live realtime audio bridge.

Replaces the STT -> Hermes -> TTS chain with a single Gemini Live
session for sub-second end-to-end conversational latency. The rest
of the ROS2 graph keeps working unchanged.

Uses the official google-genai SDK (client.aio.live.connect) rather
than raw websockets, mirroring the Mark-XXXIX project's tested
pattern. The SDK handles wire-format details, reconnects, and tool
schema conversion.

Inputs (ROS2 subscriptions):
    /audio/chunk             (UInt8MultiArray)  PCM16 mono @ 16kHz
                                                 from audio_capture_node
    /perception/wake_word    (String)           gates whether mic frames
                                                 are forwarded upstream
    /audio/playback_status   (String)           'started' suppresses mic
                                                 forwarding during own TTS

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
pipeline. Switch by stopping hermes-gateway and starting this node.
"""

from __future__ import annotations

import asyncio
import json
import os
import threading
import time
from array import array
from typing import Any, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray

try:
    from google import genai
    from google.genai import types as genai_types
except ImportError:  # pragma: no cover
    genai = None
    genai_types = None

try:
    import httpx
except ImportError:  # pragma: no cover
    httpx = None

from robot_face_msgs.msg import FaceCommand


# Gemini Live ingests 16kHz mono PCM16 and emits 24kHz mono PCM16.
INPUT_SAMPLE_RATE = 16000
OUTPUT_SAMPLE_RATE = 24000
PLAYBACK_PUBLISH_RATE = 22050  # audio_player runs at this; we resample.

# December 2025 native-audio variant. Better prosody and interruption
# handling than the bidirectional Live API base models. Validated in
# the Mark-XXXIX project. Override via the 'model' ROS2 param.
DEFAULT_MODEL = "models/gemini-2.5-flash-native-audio-preview-12-2025"

# Minimum amplitude when speaking — keeps mouth visible during quiet
# phonemes; ramps up with RMS for visible lip-sync.
MOUTH_MIN_AMP = 0.1


class RealtimeBridge(Node):

    def __init__(self) -> None:
        super().__init__("realtime_bridge")

        # ---- params ----
        self.declare_parameter("model", DEFAULT_MODEL)
        self.declare_parameter("voice", "Charon")
        self.declare_parameter("system_instruction", _DEFAULT_SYSTEM_PROMPT)
        # MCP endpoint inside the bridge daemon — same one Hermes uses.
        self.declare_parameter("mcp_url",
                                "http://127.0.0.1:8765/mcp-http/mcp")
        # API key resolution: ROS2 param wins, else env GEMINI_API_KEY,
        # else read from ~/.hermes/.env.
        self.declare_parameter("api_key", "")
        # Forward audio only while a wake word is active so we aren't
        # billed on ambient noise. Default ON.
        self.declare_parameter("require_wake", True)
        self.declare_parameter("wake_window_s", 25.0)

        self.model = str(self.get_parameter("model").value)
        self.voice = str(self.get_parameter("voice").value)
        self.system_prompt = str(self.get_parameter("system_instruction").value)
        self.mcp_url = str(self.get_parameter("mcp_url").value)
        self.require_wake = bool(self.get_parameter("require_wake").value)
        self.wake_window_s = float(self.get_parameter("wake_window_s").value)
        self.api_key = _resolve_api_key(
            str(self.get_parameter("api_key").value))

        if genai is None:
            self.get_logger().error(
                "google-genai package missing; pip install google-genai")
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
        self._wake_active_until = 0.0
        self._bot_speaking = False
        self._tail_until = 0.0
        # Total int16 bytes published to /audio/stream during the
        # current turn. Used to defer the playback_status='done' until
        # audio_player has actually finished draining its ring buffer
        # (otherwise the speaker bleeds bot voice back into the mic
        # and Gemini sees continuous user speech, never turn-completing).
        self._turn_audio_bytes = 0
        self._session: Optional[Any] = None
        self._ready = threading.Event()

        # Async loop + queues — created in the worker thread.
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._mic_queue: Optional[asyncio.Queue] = None

        # Build the genai client + Live config once.
        self._client = genai.Client(
            api_key=self.api_key,
            http_options={"api_version": "v1beta"},
        )
        self._live_config = _build_live_config(
            voice=self.voice, system_prompt=self.system_prompt)

        threading.Thread(target=self._run_loop, daemon=True).start()
        self.get_logger().info(
            f"realtime_bridge starting; model={self.model} voice={self.voice} "
            f"require_wake={self.require_wake}")

    # ---- ROS callbacks (rclpy executor thread) ----

    def _on_wake(self, _msg: String) -> None:
        was_open = time.monotonic() < self._wake_active_until
        self._wake_active_until = time.monotonic() + self.wake_window_s
        if not was_open:
            self.get_logger().info(
                f"wake window opened (window_s={self.wake_window_s})")

    def _on_playback_status(self, msg: String) -> None:
        if msg.data == "started":
            self._bot_speaking = True
        elif msg.data in ("done", "interrupted"):
            self._bot_speaking = False
            self._tail_until = time.monotonic() + 0.6

    def _on_mic_chunk(self, msg: UInt8MultiArray) -> None:
        if not self._ready.is_set() or self._loop is None:
            return
        if self._bot_speaking or time.monotonic() < self._tail_until:
            return
        if self.require_wake and time.monotonic() >= self._wake_active_until:
            return
        pcm = bytes(msg.data)
        if not pcm:
            return
        # Hand off to the asyncio loop without blocking the rclpy executor.
        self._loop.call_soon_threadsafe(self._enqueue_mic, pcm)

    def _enqueue_mic(self, pcm: bytes) -> None:
        if self._mic_queue is not None:
            try:
                self._mic_queue.put_nowait(pcm)
            except asyncio.QueueFull:
                # Drop oldest if we ever get behind.
                try:
                    self._mic_queue.get_nowait()
                    self._mic_queue.put_nowait(pcm)
                except asyncio.QueueEmpty:
                    pass

    # ---- async session lifecycle ----

    def _run_loop(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        while rclpy.ok():
            try:
                self._loop.run_until_complete(self._session_main())
            except Exception as exc:
                msg = str(exc).replace(self.api_key, "<KEY>")
                self.get_logger().error(
                    f"realtime session crashed: {msg}; reconnecting in 3s")
                time.sleep(3)
            finally:
                self._ready.clear()
                self._session = None

    async def _session_main(self) -> None:
        self._mic_queue = asyncio.Queue(maxsize=200)
        async with self._client.aio.live.connect(
                model=self.model, config=self._live_config) as session:
            self._session = session
            self._ready.set()
            self.get_logger().info("Gemini Live session established")
            # asyncio.TaskGroup needs Python 3.11; ROS2 Humble is 3.10.
            # Use asyncio.wait(FIRST_EXCEPTION) to get equivalent
            # structured-concurrency semantics: if either coroutine
            # raises, cancel the other and surface the error.
            send_task = asyncio.create_task(self._send_realtime())
            recv_task = asyncio.create_task(self._receive_loop())
            try:
                done, pending = await asyncio.wait(
                    [send_task, recv_task],
                    return_when=asyncio.FIRST_EXCEPTION)
                for t in pending:
                    t.cancel()
                # Drain cancellations so they don't show as un-retrieved
                # exceptions during shutdown.
                for t in pending:
                    try:
                        await t
                    except (asyncio.CancelledError, Exception):
                        pass
                for t in done:
                    t.result()  # propagate first exception, if any
            finally:
                # On exit, signal end of TTS so audio_capture unmutes.
                if self._bot_speaking:
                    self._emit_playback_status("done")
                    self._bot_speaking = False

    async def _send_realtime(self) -> None:
        """Drain mic queue → send to Gemini Live."""
        assert self._mic_queue is not None
        assert self._session is not None
        sent_count = 0
        while True:
            chunk = await self._mic_queue.get()
            try:
                await self._session.send_realtime_input(
                    media={"mime_type": f"audio/pcm;rate={INPUT_SAMPLE_RATE}",
                           "data": chunk})
                sent_count += 1
                # Once per second of audio (50 * 20ms chunks)
                if sent_count % 50 == 0:
                    self.get_logger().info(
                        f"sent {sent_count} mic chunks "
                        f"({sent_count * 0.02:.1f}s of audio)")
            except Exception as exc:
                self.get_logger().warning(f"send_realtime_input failed: {exc}")
                raise  # let TaskGroup tear down → reconnect

    async def _receive_loop(self) -> None:
        """Consume server-side events: audio out, tool calls, turn boundaries."""
        assert self._session is not None
        async for response in self._session.receive():
            # Tool calls
            tool_call = getattr(response, "tool_call", None)
            if tool_call is not None:
                self.get_logger().info("server: tool_call event")
                await self._handle_tool_call(tool_call)
                continue
            # Server content carries audio chunks and turn flags
            sc = getattr(response, "server_content", None)
            if sc is not None:
                await self._handle_server_content(sc)
                continue
            # Some events (setup_complete, generation_complete) we just log
            if getattr(response, "setup_complete", None) is not None:
                self.get_logger().info("server: setup_complete")
            else:
                # Anything else - dump for diagnosis
                attrs = [a for a in dir(response)
                         if not a.startswith("_")
                         and getattr(response, a, None) is not None]
                self.get_logger().info(f"server: unknown event, attrs={attrs}")

    async def _handle_server_content(self, sc: Any) -> None:
        produced_bytes = 0
        model_turn = getattr(sc, "model_turn", None)
        if model_turn is not None:
            for part in getattr(model_turn, "parts", []) or []:
                inline = getattr(part, "inline_data", None)
                if inline is None:
                    continue
                mime = getattr(inline, "mime_type", "") or ""
                if not mime.startswith("audio/pcm"):
                    continue
                data = getattr(inline, "data", None)
                if not data:
                    continue
                self._publish_audio(data)
                produced_bytes += len(data)
        turn_complete = bool(getattr(sc, "turn_complete", False))
        interrupted = bool(getattr(sc, "interrupted", False))
        if produced_bytes or turn_complete or interrupted:
            self.get_logger().info(
                f"server_content: audio={produced_bytes}B "
                f"turn_complete={turn_complete} interrupted={interrupted}")
        if produced_bytes and not self._bot_speaking:
            self._emit_playback_status("started")
            self._bot_speaking = True
        if turn_complete and self._bot_speaking:
            # Defer 'done' until audio_player has had time to drain its
            # ring buffer of all the chunks we published this turn —
            # otherwise the speaker is still playing the reply when
            # audio_capture unmutes the mic, the bot voice bleeds in,
            # gets forwarded to Gemini as user speech, and the model
            # never gets to turn-complete again on a fresh user query.
            asyncio.create_task(self._delayed_emit_done())
        if interrupted and self._bot_speaking:
            self._emit_playback_status("interrupted")
            self._bot_speaking = False
            self._tail_until = time.monotonic() + 0.6
            self._turn_audio_bytes = 0

    async def _delayed_emit_done(self) -> None:
        # Estimate playback duration from bytes published this turn.
        # int16 mono @ PLAYBACK_PUBLISH_RATE => 2 bytes/sample.
        sample_bytes = max(self._turn_audio_bytes, 0)
        playback_s = sample_bytes / (PLAYBACK_PUBLISH_RATE * 2.0)
        # Reset counter NOW so the next turn's count starts clean even
        # if we're still sleeping when its first audio chunk lands.
        self._turn_audio_bytes = 0
        # Small extra margin for audio_player's output latency + the
        # speaker tail (room echo).
        wait_s = max(playback_s, 0.0) + 0.5
        self.get_logger().info(
            f"deferring done by {wait_s:.2f}s "
            f"(playback_s={playback_s:.2f})")
        try:
            await asyncio.sleep(wait_s)
        except asyncio.CancelledError:
            pass
        self._emit_playback_status("done")
        self._bot_speaking = False
        self._tail_until = time.monotonic() + 0.6

    async def _handle_tool_call(self, tool_call: Any) -> None:
        function_calls = getattr(tool_call, "function_calls", []) or []
        responses = []
        for fc in function_calls:
            name = getattr(fc, "name", "")
            args = dict(getattr(fc, "args", {}) or {})
            call_id = getattr(fc, "id", "")
            self.get_logger().info(f"tool call: {name}({args})")
            try:
                result = await self._mcp_call(name, args)
            except Exception as exc:
                result = {"error": str(exc)}
            responses.append(genai_types.FunctionResponse(
                id=call_id, name=name, response=result))
        if responses and self._session is not None:
            try:
                await self._session.send_tool_response(
                    function_responses=responses)
            except Exception as exc:
                self.get_logger().warning(
                    f"send_tool_response failed: {exc}")

    async def _mcp_call(self, tool_name: str, args: dict) -> dict:
        if httpx is None:
            return {"error": "httpx not installed"}
        async with httpx.AsyncClient(timeout=10.0) as client:
            init_req = {
                "jsonrpc": "2.0", "id": 1, "method": "initialize",
                "params": {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {},
                    "clientInfo": {"name": "realtime-bridge", "version": "1"},
                },
            }
            r = await client.post(
                self.mcp_url, json=init_req,
                headers={"Accept": "application/json, text/event-stream"})
            session_id = r.headers.get("Mcp-Session-Id", "")
            headers = {"Accept": "application/json, text/event-stream"}
            if session_id:
                headers["Mcp-Session-Id"] = session_id
            await client.post(self.mcp_url,
                               json={"jsonrpc": "2.0",
                                     "method": "notifications/initialized"},
                               headers=headers)
            call_req = {
                "jsonrpc": "2.0", "id": 2, "method": "tools/call",
                "params": {"name": tool_name, "arguments": args},
            }
            r = await client.post(self.mcp_url, json=call_req, headers=headers)
            for line in r.text.splitlines():
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
        out_len = int(len(samples) * PLAYBACK_PUBLISH_RATE / OUTPUT_SAMPLE_RATE)
        if out_len <= 0:
            return
        xp = np.linspace(0, 1, num=len(samples), endpoint=False)
        x_new = np.linspace(0, 1, num=out_len, endpoint=False)
        resampled = np.interp(
            x_new, xp, samples.astype(np.float32)).astype(np.int16)
        msg = UInt8MultiArray()
        pcm_out_bytes = resampled.tobytes()
        msg.data = array("B", pcm_out_bytes)
        self._stream_pub.publish(msg)
        # Track playback length for the deferred 'done' below.
        self._turn_audio_bytes += len(pcm_out_bytes)
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


# Tool schema mirror of the daemon's MCP surface. The realtime bridge
# could fetch these dynamically from the MCP server at startup
# (tools/list) and convert; this hardcoded subset is enough for v1.
_BUILTIN_TOOLS = [
    {
        "name": "who_is_here",
        "description": "Identify the person currently speaking. Returns "
                        "name and confidence, or present=false if no one "
                        "has been identified yet.",
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
        "description": "Set the robot's facial expression. state in "
                        "{standby, processing, speaking, aggressive}.",
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


def _build_live_config(*, voice: str, system_prompt: str) -> Any:
    return genai_types.LiveConnectConfig(
        response_modalities=["AUDIO"],
        speech_config=genai_types.SpeechConfig(
            voice_config=genai_types.VoiceConfig(
                prebuilt_voice_config=genai_types.PrebuiltVoiceConfig(
                    voice_name=voice))),
        system_instruction=genai_types.Content(
            parts=[genai_types.Part(text=system_prompt)]),
        tools=[genai_types.Tool(function_declarations=_BUILTIN_TOOLS)],
    )


def _resolve_api_key(param_value: str) -> str:
    if param_value:
        return param_value.strip()
    env = os.environ.get("GEMINI_API_KEY", "")
    if env:
        return env.strip()
    # Fall back to ~/.hermes/.env. Strip inline '#' comments and quotes.
    try:
        env_path = os.path.expanduser("~/.hermes/.env")
        if os.path.isfile(env_path):
            with open(env_path, "r") as fh:
                for line in fh:
                    raw = line.strip()
                    if not raw.startswith("GEMINI_API_KEY="):
                        continue
                    value = raw.split("=", 1)[1].strip()
                    if value.startswith('"') and value.count('"') >= 2:
                        return value.split('"', 2)[1]
                    if "#" in value:
                        value = value.split("#", 1)[0]
                    return value.strip().strip('"').strip("'")
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
