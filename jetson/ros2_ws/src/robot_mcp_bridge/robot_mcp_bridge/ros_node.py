"""Long-lived rclpy node used by the bridge daemon.

This module owns all ROS2 graph access. The MCP tools and the events WS
endpoint both call into the singleton `RosBridge` instance — they never
touch rclpy directly. That single ownership keeps the DDS graph simple
(one node, one set of subscriptions) and lets us shut down cleanly.
"""

from __future__ import annotations

import base64
import datetime as _dt
import json
import os
import threading
import time
from pathlib import Path
from typing import Any, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, UInt8MultiArray

from robot_face_msgs.msg import FaceCommand
from robot_face_msgs.srv import Speak
from robot_perception_msgs.msg import LanguagePreference, TtsRequest

from .event_bus import get_bus

# Wake acknowledgement: a short tone burst (PCM16 mono @ 22050 Hz). 880 Hz
# for 120 ms with a small linear envelope. Kept inline so the daemon
# doesn't ship asset files separately.
def _wake_beep_pcm() -> bytes:
    import math
    import struct
    sr = 22050
    n = int(sr * 0.12)
    fade = int(sr * 0.012)
    samples = []
    for i in range(n):
        env = 1.0
        if i < fade:
            env = i / fade
        elif i > n - fade:
            env = (n - i) / fade
        s = int(0.35 * env * 32767 * math.sin(2 * math.pi * 880 * i / sr))
        samples.append(s)
    return struct.pack('<' + 'h' * len(samples), *samples)

_WAKE_BEEP_PCM = _wake_beep_pcm()

# Try the typed transcript msg (bilingual builds); fall back to plain String.
try:
    from robot_perception_msgs.msg import Transcript
    HAS_TYPED_TRANSCRIPT = True
except ImportError:
    HAS_TYPED_TRANSCRIPT = False

# Graph msg/srv types are optional — only available once robot_graph_msgs
# is built. Wrap so the daemon still starts on a partial workspace.
try:
    from robot_graph_msgs.msg import (
        IdentifiedSpeech,
        LocationIdentity,
        PersonIdentity,
    )
    from robot_graph_msgs.srv import (
        CypherQuery,
        FindRelated,
        ForgetPerson,
        IdentifyLocation,
        LearnLocation,
        ListLocations,
        ListPersons,
        Recall,
        Relate,
        RegisterPerson,
        Remember,
        RenamePerson,
        SetCurrentLocation,
    )
    GRAPH_AVAILABLE = True
except ImportError:
    GRAPH_AVAILABLE = False


def _iso_now() -> str:
    return _dt.datetime.now(_dt.timezone.utc).isoformat(
        timespec="milliseconds").replace("+00:00", "Z")


# ============================================================
# Chat log — human-readable line stream of WHAT was heard, said, and
# called. Lives at ~/robot_data/chat.log by default. Independent of the
# systemd journal (which is the firehose). Subscribers tail it via
# `robot-logs` for an at-a-glance view of the conversation.
# ============================================================


class _ChatLog:
    """Thread-safe append-only chat log."""

    def __init__(self, path: Optional[Path] = None) -> None:
        if path is None:
            base = Path(os.environ.get("ROBOT_DATA_DIR",
                                          str(Path.home() / "robot_data")))
            path = base / "chat.log"
        self._path = path
        self._lock = threading.Lock()
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            # Touch the file so robot-logs has something to tail even
            # before the first turn.
            self._path.touch(exist_ok=True)
        except Exception:
            pass

    def write(self, kind: str, text: str, **tags: Any) -> None:
        """Append a single timestamped line.

        Format: `HH:MM:SS  KIND  text  [tag=value tag=value …]`
        Multi-line text is replaced with `↩ ` to keep one event per line.
        """
        clean = (text or "").replace("\n", " ↩ ").strip()
        tag_str = ""
        if tags:
            tag_str = "  " + " ".join(f"{k}={v}" for k, v in tags.items()
                                       if v is not None and v != "")
        ts = _dt.datetime.now().strftime("%H:%M:%S")
        line = f"{ts}  {kind:<5}  {clean}{tag_str}\n"
        with self._lock:
            try:
                with open(self._path, "a", encoding="utf-8") as f:
                    f.write(line)
            except Exception:
                pass


# Singleton — RosBridge initializes it.
_CHAT_LOG: Optional[_ChatLog] = None


def chat_log() -> _ChatLog:
    global _CHAT_LOG
    if _CHAT_LOG is None:
        _CHAT_LOG = _ChatLog()
    return _CHAT_LOG


# Map state names accepted by the face MCP tool to the integer constants
# used on the wire. Lives here so the tool layer doesn't depend on
# robot_face_msgs directly.
_FACE_STATE_NAMES = {
    "standby":    FaceCommand.STATE_STANDBY,
    "processing": FaceCommand.STATE_PROCESSING,
    "speaking":   FaceCommand.STATE_SPEAKING,
    "aggressive": FaceCommand.STATE_AGGRESSIVE,
}


class _BridgeNode(Node):
    """Single ROS2 node holding publishers, subscriptions and service clients.

    Methods are kept thin: they translate Python args to ROS messages, fire,
    and return synchronously. The owning RosBridge serializes access from
    HTTP request handlers via short critical sections.
    """

    SERVICE_WAIT_S = 2.0       # how long to wait for a service to be available
    CALL_TIMEOUT_S = 10.0      # default service call timeout

    # ---- wake-word gating ----
    #
    # The bus only forwards voice events to /events subscribers (== the
    # Hermes platform adapter) when a wake word has fired within the last
    # WAKE_WINDOW_S seconds. Without this gate, every ambient utterance
    # the VAD picks up — including the robot's own TTS reverb — becomes a
    # Hermes user message, which spirals into a self-reply loop.
    # NOTE: whisper transcribe latency on Jetson Orin Nano CPU is 4-6s
    # for the 'small' model, and the user can take 1-2s to start speaking
    # after the wake-ack beep. Keep this generous so the final transcript
    # still lands inside the window even after STT queue drains.
    WAKE_WINDOW_S = 25.0

    # Acoustic-wake "swallow" window. The very first short transcript
    # arriving in the N seconds after the acoustic wake fires is almost
    # always the wake utterance itself ("hey jarvis"), often mistranscribed
    # by whisper ("Hey John", "Hey Jodavis", "Jarvis"). Dispatching it
    # makes Hermes start responding to the wake greeting, and the user's
    # real question arrives mid-reply and triggers an interrupt. Swallow
    # the first short transcript within this window.
    ACOUSTIC_WAKE_SWALLOW_S = 2.5
    ACOUSTIC_WAKE_SWALLOW_MAX_WORDS = 4

    # Text wake fallback. STT sometimes catches "hey jarvis / hey robot"
    # at the start of an utterance when the acoustic wake_word_node misses
    # (accent, distance, model confidence < threshold). When that happens
    # we open the wake window from the transcript side too. Leading punctuation
    # and whitespace are stripped before the prefix check.
    TEXT_WAKE_PHRASES = (
        "hey jarvis",
        "hey robot",
        "hei jarvis",
        "salut robotule",
        "robot",
    )

    # ---- mic-mute strategy ----
    #
    # Even with audio_capture dropping chunks during playback, the
    # speech_recognizer may have a transcript half-baked when TTS starts.
    # As a third defence layer, the daemon also tracks /audio/playback_status
    # and refuses to forward voice / wake events to /events for the duration
    # of TTS playback + a hold-over tail.
    BOT_SPEAKING_TAIL_S = 0.6

    def __init__(self) -> None:
        super().__init__("robot_mcp_bridge")

        # Event bus for fanning perception updates out to WebSocket clients.
        self._bus = get_bus()

        # Wake-word window — set by /perception/wake_word, cleared by timeout.
        self._wake_until: float = 0.0
        # Acoustic-wake swallow window: deadline before which the next
        # short transcript is treated as the wake utterance itself
        # (whisper transcription of "hey jarvis") and dropped.
        self._acoustic_wake_until: float = 0.0
        # Bot-speaking suppression — set by /audio/playback_status callbacks.
        self._bot_speaking: bool = False
        self._bot_speaking_until: float = 0.0

        # Publishers
        self._tts_pub = self.create_publisher(TtsRequest, "/tts/say", 10)
        self._face_pub = self.create_publisher(FaceCommand, "/face/command", 10)

        # Speech (legacy synchronous API)
        self._speak_cli = self.create_client(Speak, "/speak")

        # Latest transcript cache + wakeup event for `listen()`
        self._last_transcript: Optional[str] = None
        self._transcript_event = threading.Event()

        # Perception state caches (also used to format event payloads).
        self._last_person: Any = None
        self._last_location: Any = None
        self._last_addressee: float = 0.5
        self._addressee_hint: str = ""
        self._current_language: str = "ro"
        self._language_source: str = "default"
        self._latest_frame_b64: Optional[str] = None
        self._latest_frame_t: float = 0.0

        # ---- perception subscriptions (fan out to /events WS) ----
        if HAS_TYPED_TRANSCRIPT:
            self.create_subscription(Transcript, "/perception/transcript",
                                     self._on_typed_transcript, 10)
            self.create_subscription(LanguagePreference, "/language/current",
                                     self._on_language, 10)
        else:
            self.create_subscription(String, "/perception/transcript",
                                     self._on_legacy_transcript, 10)

        self.create_subscription(Bool, "/perception/voice_active",
                                 self._on_voice_active, 10)
        self.create_subscription(String, "/perception/wake_word",
                                 self._on_wake_word, 5)
        self.create_subscription(Float32, "/perception/addressee_score",
                                 self._on_addressee_score, 10)
        self.create_subscription(String, "/perception/addressee_hint",
                                 self._on_addressee_hint, 5)
        self.create_subscription(String, "/agent/context_warm",
                                 self._on_context_warm, 10)
        self.create_subscription(UInt8MultiArray,
                                 "/vision/frame_at_utterance",
                                 self._on_frame, 5)
        self.create_subscription(String, "/audio/playback_status",
                                 self._on_playback_status, 10)

        # Graph service clients — created lazily when first needed.
        self._graph_clients: dict[str, Any] = {}

        if GRAPH_AVAILABLE:
            self.create_subscription(PersonIdentity,
                                     "/perception/identified_person",
                                     self._on_person, 10)
            self.create_subscription(LocationIdentity,
                                     "/perception/current_location",
                                     self._on_location, 10)
            self.create_subscription(IdentifiedSpeech,
                                     "/perception/identified_speech",
                                     self._on_identified_speech, 10)

    # ---- subscriber callbacks ----
    #
    # Each callback both updates the local state cache AND publishes a JSON
    # event onto the bus. The platform adapter listens on /events and turns
    # these into Hermes MessageEvent dispatches.

    def _wake_active(self) -> bool:
        return time.monotonic() < self._wake_until

    def _bot_is_speaking(self) -> bool:
        return self._bot_speaking or time.monotonic() < self._bot_speaking_until

    def _on_playback_status(self, msg: String) -> None:
        status = msg.data
        if status == "started":
            self._bot_speaking = True
        else:   # 'done' | 'interrupted' | anything else
            self._bot_speaking = False
            self._bot_speaking_until = (
                time.monotonic() + self.BOT_SPEAKING_TAIL_S)

    def _text_wake_match(self, text: str) -> Optional[str]:
        """If `text` starts with a known wake phrase, return the matched
        phrase; else return None. Strips leading punctuation/whitespace,
        case-insensitive comparison.
        """
        if not text:
            return None
        s = text.lstrip().lstrip("!?.,:; ").lower()
        for phrase in self.TEXT_WAKE_PHRASES:
            if s.startswith(phrase):
                # Require a word boundary after the phrase so 'robotic'
                # doesn't match 'robot'.
                tail = s[len(phrase):len(phrase) + 1]
                if tail == "" or not tail.isalpha():
                    return phrase
        return None

    def _emit_voice(self, text: str, lang: str = "") -> None:
        if not text:
            return
        self._last_transcript = text
        self._transcript_event.set()

        # Mic-mute gate (Pipecat-style AlwaysUserMuteStrategy): if the
        # bot is currently speaking, treat any inbound transcript as
        # leak-through and discard. Don't even check wake-word — a wake
        # match while the bot is mid-sentence is almost certainly the
        # bot's own audio bleed.
        if self._bot_is_speaking():
            chat_log().write("USER", text, lang=lang or self._current_language,
                              status="dropped:bot_speaking")
            return

        # Belt-and-suspenders: if the acoustic wake_word_node missed but
        # the transcript itself starts with a known wake phrase, open the
        # window now.
        matched_phrase = self._text_wake_match(text)
        if matched_phrase is not None:
            self._open_wake_window(source="transcript",
                                     detail=matched_phrase)
            # Suppress wake-only utterances. When the user says just "Hey
            # Jarvis." and pauses, we should open the window but NOT also
            # dispatch the wake phrase as a chat message — otherwise
            # Hermes starts replying to "Hey Jarvis." and the user's
            # follow-up arrives mid-reply, triggering an interrupt.
            stripped_tail = (text.lstrip().lstrip("!?.,:; ")
                              [len(matched_phrase):]
                              .strip(" .!?,;:"))
            if not stripped_tail:
                chat_log().write("USER", text,
                                  lang=lang or self._current_language,
                                  status="wake_only")
                return

        # Acoustic-wake swallow: the first short transcript landing in
        # the few seconds after an acoustic wake is overwhelmingly the
        # wake utterance itself, mistranscribed by whisper as anything
        # from "Hey John" to "Jodavis" to "Hey." Drop it so Hermes
        # doesn't try to reply to the greeting before the real question
        # arrives. We clear the deadline regardless so only the first
        # transcript gets swallowed.
        now_mono = time.monotonic()
        if now_mono < self._acoustic_wake_until:
            self._acoustic_wake_until = 0.0
            word_count = len([w for w in text.split() if w])
            if word_count <= self.ACOUSTIC_WAKE_SWALLOW_MAX_WORDS:
                chat_log().write("USER", text,
                                  lang=lang or self._current_language,
                                  status="swallowed:post_wake")
                return

        # Hard gate: only forward voice events to the platform adapter
        # while a wake word is recent. Without this the robot replies to
        # ambient noise (including its own TTS bleed) every utterance.
        if not self._wake_active():
            chat_log().write("USER", text, lang=lang or self._current_language,
                              status="dropped:no_wake")
            return

        chat_log().write("USER", text, lang=lang or self._current_language)
        event = {
            "type": "voice",
            "ts": _iso_now(),
            "text": text,
            "lang": lang or self._current_language,
            "language_source": self._language_source,
            "speaker_id": getattr(self._last_person, "person_id", "") if self._last_person else "",
            "speaker_name": getattr(self._last_person, "primary_name", "") if self._last_person else "",
            "location": (self._last_location.name if self._last_location and
                          not getattr(self._last_location, "is_unknown", False) else ""),
            "addressee_score": self._last_addressee,
            "addressee_hint": self._addressee_hint,
            "frame_b64": (self._latest_frame_b64
                          if self._latest_frame_b64 and
                          (time.time() - self._latest_frame_t) < 30.0
                          else None),
        }
        self._bus.publish(event)

    def _on_typed_transcript(self, msg) -> None:
        self._emit_voice(text=msg.text, lang=msg.language)

    def _on_legacy_transcript(self, msg: String) -> None:
        self._emit_voice(text=msg.data)

    def _on_language(self, msg) -> None:
        new_lang = msg.language or self._current_language
        if new_lang != self._current_language:
            self._bus.publish({
                "type": "language_changed",
                "ts": _iso_now(),
                "from": self._current_language,
                "to": new_lang,
                "source": msg.source or "",
            })
        self._current_language = new_lang
        self._language_source = msg.source or self._language_source

    def _on_voice_active(self, msg: Bool) -> None:
        # Mirror the same wake-gating policy as voice events.
        if not self._wake_active():
            return
        self._bus.publish({
            "type": "voice_started" if msg.data else "voice_ended",
            "ts": _iso_now(),
        })

    def _open_wake_window(self, *, source: str, detail: str = "") -> None:
        """Open the listening window AND give the user an acknowledgement.

        Called from both the acoustic wake handler and the transcript
        wake fallback. Idempotent within the same window — back-to-back
        wakes just refresh the deadline; the ack only fires when the
        window transitions from closed → open.

        Suppressed while the bot is speaking — a wake match heard while
        TTS is playing is almost certainly the bot triggering itself.
        """
        if self._bot_is_speaking():
            return

        now = time.monotonic()
        was_open = now < self._wake_until
        self._wake_until = now + self.WAKE_WINDOW_S
        # Remember the moment this window opened so _emit_voice can
        # recognize "the first transcript right after wake is probably
        # just the wake utterance itself, mis-transcribed by whisper"
        # and refuse to dispatch it as a chat message.
        if source == "acoustic" and not was_open:
            self._acoustic_wake_until = now + self.ACOUSTIC_WAKE_SWALLOW_S
        self._bus.publish({
            "type": "wake_word",
            "ts": _iso_now(),
            "source": source,         # 'acoustic' | 'transcript'
            "model": detail,          # model id or matched phrase
            "window_s": self.WAKE_WINDOW_S,
        })
        if not was_open:
            chat_log().write("WAKE", detail or "(wake)", source=source)
            try:
                self.publish_wake_ack()
            except Exception:
                pass

    def _on_wake_word(self, msg: String) -> None:
        """Acoustic wake fired by wake_word_node (openWakeWord)."""
        self._open_wake_window(source="acoustic", detail=msg.data)

    def _on_addressee_score(self, msg: Float32) -> None:
        self._last_addressee = float(msg.data)

    def _on_addressee_hint(self, msg: String) -> None:
        self._addressee_hint = msg.data

    def _on_context_warm(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._bus.publish({
            "type": "context_warm",
            "ts": _iso_now(),
            "payload": payload,
        })

    def _on_frame(self, msg) -> None:
        try:
            raw = bytes(msg.data)
            self._latest_frame_b64 = base64.b64encode(raw).decode("ascii")
            self._latest_frame_t = time.time()
        except Exception:
            self._latest_frame_b64 = None

    def _on_person(self, msg) -> None:
        prev_id = getattr(self._last_person, "person_id", None) if self._last_person else None
        self._last_person = msg
        if msg.person_id and msg.person_id != prev_id:
            self._bus.publish({
                "type": "person_identified",
                "ts": _iso_now(),
                "person_id": msg.person_id,
                "name": msg.primary_name,
                "voice_confidence": float(msg.voice_confidence),
                "is_new": bool(msg.is_new),
            })

    def _on_location(self, msg) -> None:
        prev_id = getattr(self._last_location, "location_id", None) if self._last_location else None
        self._last_location = msg
        if (not getattr(msg, "is_unknown", False)
                and msg.location_id and msg.location_id != prev_id):
            self._bus.publish({
                "type": "location_changed",
                "ts": _iso_now(),
                "location_id": msg.location_id,
                "name": msg.name,
                "parent": msg.parent_name,
                "confidence": float(msg.confidence),
            })

    def _on_identified_speech(self, msg) -> None:
        # Authoritative source when speaker is known. Updates the person
        # cache and emits a voice event with the speaker pre-resolved.
        if msg.speaker and msg.speaker.person_id:
            self._last_person = msg.speaker
        if msg.location and not getattr(msg.location, "is_unknown", False):
            self._last_location = msg.location
        self._emit_voice(text=msg.text, lang=self._current_language)

    # ---- outbound ----

    def publish_tts(self, text: str, language: str = "", voice: str = "") -> None:
        msg = TtsRequest()
        msg.text = text
        msg.language = language
        msg.voice = voice
        self._tts_pub.publish(msg)

    def publish_face(self, state: int, amplitude: float = 0.0) -> None:
        msg = FaceCommand()
        msg.state = int(state)
        msg.amplitude = max(0.0, min(1.0, float(amplitude)))
        self._face_pub.publish(msg)

    def publish_wake_ack(self) -> None:
        """Tell the user we heard the wake word — face goes to PROCESSING
        and a short tone plays through the existing /audio/playback pipe.
        """
        # Face cue (no-op if face hardware absent — face_bridge swallows
        # disconnects gracefully and the rest of the system doesn't care).
        self.publish_face(FaceCommand.STATE_PROCESSING, 0.6)

        # Audio beep via the same UInt8MultiArray /audio/playback topic
        # audio_player already subscribes to. Lazy-publisher creation so
        # we don't carry the publisher cost when no acoustic ack is desired.
        from array import array
        from std_msgs.msg import UInt8MultiArray
        if not hasattr(self, "_beep_pub") or self._beep_pub is None:
            self._beep_pub = self.create_publisher(
                UInt8MultiArray, "/audio/playback", 5)
        msg = UInt8MultiArray()
        msg.data = array('B', _WAKE_BEEP_PCM)
        self._beep_pub.publish(msg)

    def call_speak(self, text: str, timeout: float | None = None) -> Optional[Any]:
        timeout = timeout or self.CALL_TIMEOUT_S
        if not self._speak_cli.wait_for_service(timeout_sec=self.SERVICE_WAIT_S):
            return None
        req = Speak.Request()
        req.text = text
        future = self._speak_cli.call_async(req)
        deadline = time.time() + timeout
        while time.time() < deadline and not future.done():
            time.sleep(0.02)
        return future.result()

    # ---- graph clients (lazy) ----

    def _graph_client(self, key: str, srv_type, service_name: str):
        if not GRAPH_AVAILABLE:
            return None
        cli = self._graph_clients.get(key)
        if cli is None:
            cli = self.create_client(srv_type, service_name)
            self._graph_clients[key] = cli
        return cli

    def _call_graph(self, key: str, srv_type, service_name: str, request,
                     timeout: float | None = None) -> Optional[Any]:
        cli = self._graph_client(key, srv_type, service_name)
        if cli is None:
            return None
        timeout = timeout or self.CALL_TIMEOUT_S
        if not cli.wait_for_service(timeout_sec=self.SERVICE_WAIT_S):
            return None
        future = cli.call_async(request)
        deadline = time.time() + timeout
        while time.time() < deadline and not future.done():
            time.sleep(0.02)
        return future.result()


class RosBridge:
    """Singleton wrapping the rclpy node and its background spin thread.

    All public methods are safe to call from any thread (FastAPI handlers,
    MCP tool callbacks). The actual rclpy work happens on the executor's
    own threads via topic publish (lock-free) or async service calls.
    """

    _instance: Optional["RosBridge"] = None
    _lock = threading.Lock()

    def __new__(cls) -> "RosBridge":
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._start()
        return cls._instance

    def _start(self) -> None:
        if not rclpy.ok():
            rclpy.init()
        self._node = _BridgeNode()
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, name="rosbridge-spin", daemon=True
        )
        self._spin_thread.start()
        # Give the executor a tick to register publishers before first publish.
        time.sleep(0.05)

    def shutdown(self) -> None:
        try:
            self._executor.shutdown(timeout_sec=2.0)
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

    # ============================================================
    # MCP tool surface — each method returns a JSON-serializable dict
    # ============================================================

    # ---- face / speech ----

    def speak(self, text: str, language: str = "", voice: str = "") -> dict:
        if not text:
            return {"ok": False, "error": "empty text"}
        self._node.publish_tts(text=text, language=language, voice=voice)
        return {"ok": True, "text": text, "language": language or "default"}

    def speak_sync(self, text: str, timeout_seconds: float = 30.0) -> dict:
        """Legacy blocking speak: calls the /speak service and waits for
        completion. Use `speak()` for fire-and-forget streaming."""
        resp = self._node.call_speak(text, timeout=timeout_seconds)
        if resp is None:
            return {"ok": False, "error": "speak service unavailable or timed out"}
        return {"ok": bool(resp.success),
                "message": resp.message,
                "duration_seconds": float(resp.duration_seconds)}

    def set_face(self, state: str, amplitude: float = 0.0) -> dict:
        state_int = _FACE_STATE_NAMES.get(state.lower())
        if state_int is None:
            return {"ok": False,
                     "error": f"unknown state '{state}'",
                     "known_states": sorted(_FACE_STATE_NAMES.keys())}
        self._node.publish_face(state_int, amplitude)
        return {"ok": True, "state": state, "amplitude": float(amplitude)}

    def listen(self, timeout_seconds: float = 15.0) -> dict:
        self._node._transcript_event.clear()
        self._node._last_transcript = None
        if self._node._transcript_event.wait(timeout=timeout_seconds):
            return {"ok": True, "text": self._node._last_transcript or ""}
        return {"ok": False, "text": "", "error": "timeout"}

    # ---- identity ----

    def who_is_here(self) -> dict:
        p = self._node._last_person
        if p is None:
            return {"present": False}
        return {"present": True, "person_id": p.person_id,
                "name": p.primary_name,
                "voice_confidence": float(p.voice_confidence),
                "is_new": bool(p.is_new)}

    def register_person(self, name: str) -> dict:
        if not GRAPH_AVAILABLE:
            return {"ok": False, "error": "graph msgs not built"}
        req = RegisterPerson.Request()
        req.name = name
        req.audio_pcm_int16 = []
        req.sample_rate = 16000
        resp = self._node._call_graph("register_person", RegisterPerson,
                                       "/identity/register_person", req)
        if resp is None:
            return {"ok": False, "error": "service unavailable"}
        return {"ok": bool(resp.success), "message": resp.message,
                "person_id": resp.identity.person_id,
                "name": resp.identity.primary_name}

    def list_persons(self) -> dict:
        if not GRAPH_AVAILABLE:
            return {"ok": False, "persons": []}
        resp = self._node._call_graph("list_persons", ListPersons,
                                       "/identity/list_persons",
                                       ListPersons.Request())
        if resp is None:
            return {"ok": False, "persons": []}
        return {"ok": True,
                "persons": [{"id": p.person_id, "name": p.primary_name}
                            for p in resp.persons]}

    def forget_person(self, person_id: str) -> dict:
        if not GRAPH_AVAILABLE:
            return {"ok": False, "error": "graph msgs not built"}
        req = ForgetPerson.Request()
        req.person_id = person_id
        resp = self._node._call_graph("forget_person", ForgetPerson,
                                       "/identity/forget_person", req)
        return {"ok": bool(resp and resp.success),
                "message": resp.message if resp else ""}

    # ---- location ----

    def where_am_i(self) -> dict:
        l = self._node._last_location
        if l is None or getattr(l, "is_unknown", True):
            return {"known": False}
        return {"known": True, "location_id": l.location_id, "name": l.name,
                "parent": l.parent_name, "confidence": float(l.confidence)}

    def learn_location(self, name: str, parent: str = "") -> dict:
        if not GRAPH_AVAILABLE:
            return {"ok": False, "error": "graph msgs not built"}
        req = LearnLocation.Request()
        req.name = name
        req.parent_name = parent
        req.sample_count = 5
        resp = self._node._call_graph("learn_location", LearnLocation,
                                       "/location/learn", req)
        return {"ok": bool(resp and resp.success),
                "message": resp.message if resp else "",
                "location_id": resp.identity.location_id if resp else ""}

    def set_current_location(self, name: str, parent: str = "") -> dict:
        if not GRAPH_AVAILABLE:
            return {"ok": False, "error": "graph msgs not built"}
        req = SetCurrentLocation.Request()
        req.name = name
        req.parent_name = parent
        resp = self._node._call_graph("set_current_location",
                                       SetCurrentLocation,
                                       "/location/set_current", req)
        return {"ok": bool(resp and resp.success),
                "message": resp.message if resp else "",
                "location_id": resp.identity.location_id if resp else ""}

    def list_locations(self) -> dict:
        if not GRAPH_AVAILABLE:
            return {"ok": False, "locations": []}
        resp = self._node._call_graph("list_locations", ListLocations,
                                       "/location/list",
                                       ListLocations.Request())
        if resp is None:
            return {"ok": False, "locations": []}
        return {"ok": True,
                "locations": [{"id": l.location_id, "name": l.name,
                                "parent": l.parent_name}
                              for l in resp.locations]}

    # ---- memory ----

    def remember(self, subject_id: str, subject_type: str, content: str,
                 tags: str = "", source: str = "manual",
                 confidence: float = 1.0) -> dict:
        if not GRAPH_AVAILABLE:
            return {"ok": False, "error": "graph msgs not built"}
        req = Remember.Request()
        req.subject_id = subject_id
        req.subject_type = subject_type
        req.content = content
        req.tags = tags
        req.source = source
        req.confidence = float(confidence)
        resp = self._node._call_graph("remember", Remember,
                                       "/memory/remember", req)
        return {"ok": bool(resp and resp.success),
                "fact_id": resp.fact_id if resp else ""}

    def recall(self, subject_id: str, subject_type: str = "Person",
               query: str = "", limit: int = 10) -> dict:
        if not GRAPH_AVAILABLE:
            return {"ok": False, "facts": []}
        req = Recall.Request()
        req.subject_id = subject_id
        req.subject_type = subject_type
        req.query = query
        req.limit = int(limit)
        resp = self._node._call_graph("recall", Recall,
                                       "/memory/recall", req)
        if resp is None:
            return {"ok": False, "facts": []}
        return {"ok": True,
                "facts": [{"id": fid, "content": content, "score": float(score)}
                          for fid, content, score in zip(
                              resp.fact_ids, resp.contents, resp.scores)]}

    def relate_persons(self, a_id: str, b_id: str, relation: str,
                       description: str = "",
                       bidirectional: bool = False) -> dict:
        if not GRAPH_AVAILABLE:
            return {"ok": False, "error": "graph msgs not built"}
        req = Relate.Request()
        req.subject_a_id = a_id
        req.subject_b_id = b_id
        req.subject_type = "Person"
        req.relation = relation
        req.description = description
        req.bidirectional = bool(bidirectional)
        resp = self._node._call_graph("relate", Relate,
                                       "/memory/relate", req)
        return {"ok": bool(resp and resp.success)}

    def find_related(self, subject_id: str, relation: str = "",
                     hops: int = 1) -> dict:
        if not GRAPH_AVAILABLE:
            return {"ok": False, "related": []}
        req = FindRelated.Request()
        req.subject_id = subject_id
        req.subject_type = "Person"
        req.relation = relation
        req.hops = int(hops)
        resp = self._node._call_graph("find_related", FindRelated,
                                       "/memory/find_related", req)
        if resp is None:
            return {"ok": False, "related": []}
        return {"ok": True,
                "related": [{"id": i, "name": n}
                            for i, n in zip(resp.related_ids,
                                              resp.related_names)]}

    def cypher(self, query: str, params: Optional[dict] = None) -> dict:
        if not GRAPH_AVAILABLE:
            return {"ok": False, "rows": [], "error": "graph msgs not built"}
        req = CypherQuery.Request()
        req.cypher = query
        req.params_json = json.dumps(params or {})
        resp = self._node._call_graph("cypher", CypherQuery,
                                       "/graph/cypher", req)
        if resp is None:
            return {"ok": False, "rows": []}
        try:
            rows = json.loads(resp.result_json)
        except json.JSONDecodeError:
            rows = []
        return {"ok": bool(resp.success), "message": resp.message,
                "rows": rows}
