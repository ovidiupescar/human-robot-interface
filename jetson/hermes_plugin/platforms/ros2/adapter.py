"""ROS2 perception platform adapter for Hermes.

Inherits from BasePlatformAdapter (gateway/platforms/base.py). Subscribes to
ROS2 perception topics and translates them into MessageEvents dispatched into
a single persistent Hermes session keyed off chat_id="robot-main", so every
voice/vision/sensor event accumulates in the same conversation.

Outbound send() calls are routed through robot_bridge to TTS (the robot speaks).
"""

from __future__ import annotations

import asyncio
import base64
import datetime as _dt
import json
import os
import threading
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
    from gateway.session import Platform, SessionSource
    HERMES_AVAILABLE = True
except ImportError:
    HERMES_AVAILABLE = False
    BasePlatformAdapter = object  # type: ignore
    class MessageEvent:           # type: ignore
        def __init__(self, **kw): self.__dict__.update(kw)
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
        def __init__(self, name): self.name = name

# --- ROS2 imports ---
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, ByteMultiArray, Float32, String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    rclpy = None  # type: ignore

# --- Typed perception messages (bilingual transcripts, language, TTS) ---
try:
    from robot_perception_msgs.msg import (
        LanguagePreference,
        Transcript,
        TtsRequest,
    )
    PERCEPTION_MSGS = True
except ImportError:
    PERCEPTION_MSGS = False

# --- Optional graph msgs ---
try:
    from robot_graph_msgs.msg import (
        IdentifiedSpeech,
        LocationIdentity,
        PersonIdentity,
    )
    GRAPH_MSGS = True
except ImportError:
    GRAPH_MSGS = False

# --- robot_bridge for outbound (TTS) + state caches ---
try:
    from robot_bridge import RobotBridge
    BRIDGE_AVAILABLE = True
except ImportError:
    BRIDGE_AVAILABLE = False


# Stable chat_id so all ROS2 events land in ONE persistent session
CHAT_ID = os.environ.get("ROS2_ROBOT_CHAT_ID", "robot-main")
USER_ID = "physical-world"


# ============================================================
# Internal ROS2 node — collects perception events and emits
# pre-formatted text strings ready to drop into MessageEvent.
# ============================================================

class _PerceptionListener(Node):
    def __init__(self, on_event):
        super().__init__("hermes_ros2_perception_listener")
        self._on_event = on_event

        # Caches
        self._last_person_name = ""
        self._last_person_id = ""
        self._last_location_name = ""
        self._last_addressee = 0.5
        self._addressee_hint = ""
        # Bilingual + prefetch state
        self._current_language = "ro"
        self._language_source = "default"
        self._context_warm: Optional[dict] = None
        self._context_warm_t = 0.0
        self._latest_frame_b64: Optional[str] = None
        self._latest_frame_t = 0.0

        # Subscriptions
        if PERCEPTION_MSGS:
            # Typed transcript (preferred; carries detected language)
            self.create_subscription(Transcript, "/perception/transcript",
                                     self._on_typed_transcript, 10)
            self.create_subscription(LanguagePreference, "/language/current",
                                     self._on_language, 10)
        else:
            # Fallback for older builds
            self.create_subscription(String, "/perception/transcript",
                                     self._on_transcript_legacy, 10)

        self.create_subscription(Bool, "/perception/voice_active",
                                 self._on_voice, 10)
        self.create_subscription(Float32, "/perception/addressee_score",
                                 self._on_addressee, 10)
        self.create_subscription(String, "/perception/addressee_hint",
                                 self._on_addressee_hint, 5)

        # Proactive prefetch + vision capture
        self.create_subscription(String, "/agent/context_warm",
                                 self._on_context_warm, 10)
        self.create_subscription(ByteMultiArray, "/vision/frame_at_utterance",
                                 self._on_frame, 5)

        if GRAPH_MSGS:
            self.create_subscription(IdentifiedSpeech,
                                     "/perception/identified_speech",
                                     self._on_id_speech, 10)
            self.create_subscription(PersonIdentity,
                                     "/perception/identified_person",
                                     self._on_id_person, 10)
            self.create_subscription(LocationIdentity,
                                     "/perception/current_location",
                                     self._on_location, 10)

        # Outbound TTS publisher (typed, language-aware)
        if PERCEPTION_MSGS:
            self._tts_pub = self.create_publisher(TtsRequest, "/tts/say", 10)
        else:
            self._tts_pub = None

    # --- callbacks ---

    def _on_typed_transcript(self, msg):
        # Final transcript with detected language. Use directly when no
        # IdentifiedSpeech arrives (e.g., speaker not yet known).
        if msg.language:
            # Note: actual current language is resolved upstream; this is only
            # the *detected* signal. The authoritative value comes via
            # /language/current.
            pass
        if not GRAPH_MSGS or not self._last_person_id:
            self._emit_voice(text=msg.text, lang=msg.language)

    def _on_transcript_legacy(self, msg: String):
        # Pre-bilingual build
        if not GRAPH_MSGS:
            self._emit_voice(text=msg.data)

    def _on_language(self, msg):
        self._current_language = msg.language or self._current_language
        self._language_source = msg.source or self._language_source

    def _on_context_warm(self, msg: String):
        try:
            self._context_warm = json.loads(msg.data)
            self._context_warm_t = time.time()
        except json.JSONDecodeError:
            pass

    def _on_frame(self, msg):
        try:
            data = bytes(msg.data)
            self._latest_frame_b64 = base64.b64encode(data).decode("ascii")
            self._latest_frame_t = time.time()
        except Exception:
            pass

    def _on_id_speech(self, msg):
        self._last_person_name = msg.speaker.primary_name
        self._last_person_id = msg.speaker.person_id
        self._last_location_name = msg.location.name
        # Note: IdentifiedSpeech message doesn't carry language yet;
        # _current_language reflects the resolver's latest decision.
        self._emit_voice(text=msg.text, lang=self._current_language)

    def _on_id_person(self, msg):
        self._last_person_name = msg.primary_name
        self._last_person_id = msg.person_id

    def _on_location(self, msg):
        if not getattr(msg, "is_unknown", False):
            self._last_location_name = msg.name

    def _on_voice(self, msg: Bool):
        if msg.data:
            self._emit_meta("voice_started")
        # voice_end without transcript: handled by speech_recognizer pushing transcript or not

    def _on_addressee(self, msg: Float32):
        self._last_addressee = float(msg.data)

    def _on_addressee_hint(self, msg: String):
        self._addressee_hint = msg.data

    # --- format & dispatch ---

    def _emit_voice(self, text: str, lang: str = ""):
        if not text:
            return
        ts = self._iso_now()
        rel = "just now"
        spk = self._speaker_tag()
        loc = self._location_tag()
        addr = f"addressee={self._last_addressee:.2f}"
        if self._addressee_hint:
            addr += f", hint:{self._addressee_hint}"
        lang_tag = f", lang={lang}" if lang else ""
        formatted = (f"[{ts} ({rel}) | USER_VOICE from {spk}{loc} | {addr}"
                     f"{lang_tag}]: {text}")
        # Bundle augmentations for the orchestrator to use when calling the LLM
        aug = {
            "current_language": self._current_language,
            "language_source": self._language_source,
            "context_warm": self._fresh_context_warm(),
            "frame_b64": self._fresh_frame(),
        }
        self._on_event(formatted, kind="voice", augmentations=aug)

    def _fresh_context_warm(self) -> Optional[dict]:
        if not self._context_warm:
            return None
        ttl_ms = int(self._context_warm.get("ttl_ms", 0))
        age_ms = (time.time() - self._context_warm_t) * 1000.0
        if ttl_ms > 0 and age_ms > ttl_ms:
            return None
        return self._context_warm

    def _fresh_frame(self) -> Optional[str]:
        if not self._latest_frame_b64:
            return None
        age = time.time() - self._latest_frame_t
        if age > 30.0:
            return None
        return self._latest_frame_b64

    def _emit_meta(self, label: str):
        ts = self._iso_now()
        loc = self._location_tag()
        formatted = f"[{ts} | META {label}{loc}]"
        self._on_event(formatted, kind="meta")

    def _speaker_tag(self) -> str:
        if self._last_person_name:
            return (f"{self._last_person_name}"
                    f"({self._last_person_id})") if self._last_person_id \
                else self._last_person_name
        if self._last_person_id:
            return f"Unknown({self._last_person_id})"
        return "Unknown"

    def _location_tag(self) -> str:
        if self._last_location_name:
            return f" @{self._last_location_name}"
        return ""

    @staticmethod
    def _iso_now() -> str:
        return _dt.datetime.now(_dt.timezone.utc).isoformat(
            timespec="milliseconds").replace("+00:00", "Z")


# ============================================================
# The adapter Hermes instantiates
# ============================================================

class Ros2Adapter(BasePlatformAdapter):
    def __init__(self, config, **kwargs):
        platform = Platform("ros2") if HERMES_AVAILABLE else Platform("ros2")
        super().__init__(config=config, platform=platform) if HERMES_AVAILABLE else None
        self._node: Optional[_PerceptionListener] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._bridge: Optional[Any] = None

    # ---- Hermes lifecycle ----

    async def connect(self) -> bool:
        if not ROS_AVAILABLE:
            return False
        if not rclpy.ok():
            rclpy.init()
        self._loop = asyncio.get_running_loop()
        self._node = _PerceptionListener(self._dispatch_event)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        if BRIDGE_AVAILABLE:
            self._bridge = RobotBridge()
        return True

    async def disconnect(self) -> None:
        try:
            if self._node is not None:
                self._node.destroy_node()
        finally:
            self._node = None

    async def send(self, chat_id: str, content: str,
                   reply_to: Optional[str] = None,
                   metadata: Optional[Dict[str, Any]] = None) -> SendResult:
        if not content:
            return SendResult(success=True, message_id=str(int(time.time() * 1000)))

        # Preferred path: publish typed TtsRequest with explicit language.
        # Falls through to robot_bridge.speak() if the typed publisher isn't
        # available (e.g., perception_msgs missing in environment).
        language = ""
        if metadata and isinstance(metadata, dict):
            language = (metadata.get("language")
                        or metadata.get("response_language") or "")
        if not language and self._node is not None:
            language = getattr(self._node, "_current_language", "") or ""

        if (PERCEPTION_MSGS and self._node is not None
                and self._node._tts_pub is not None):
            try:
                req = TtsRequest()
                req.text = content
                req.language = language
                req.voice = ""  # use default voice for language
                self._node._tts_pub.publish(req)
                return SendResult(success=True,
                                   message_id=str(int(time.time() * 1000)))
            except Exception as e:
                # Fall through to bridge path
                pass

        if self._bridge is None:
            return SendResult(success=False, error="no TTS publisher available",
                               retryable=False)
        try:
            await asyncio.to_thread(self._bridge.speak, content)
            return SendResult(success=True, message_id=str(int(time.time() * 1000)))
        except Exception as e:
            return SendResult(success=False, error=str(e), retryable=True)

    async def send_typing(self, chat_id: str,
                          metadata: Optional[Dict[str, Any]] = None) -> None:
        # Reflex node already switches face to PROCESSING on voice activity.
        # We could also explicitly drive the face to processing here.
        if self._bridge is not None:
            try:
                self._bridge.set_face("processing")
            except Exception:
                pass

    async def send_image(self, chat_id: str, image_url: str,
                         caption: Optional[str] = None,
                         reply_to: Optional[str] = None,
                         metadata: Optional[Dict[str, Any]] = None) -> SendResult:
        # Robot has no image output channel yet.
        return SendResult(success=False, error="no image output on this platform",
                           retryable=False)

    async def get_chat_info(self, chat_id: str) -> Dict[str, Any]:
        return {"name": "Robot physical session", "type": "dm", "chat_id": chat_id}

    # ---- internal ----

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def _dispatch_event(self, formatted_text: str, kind: str = "voice",
                          augmentations: Optional[Dict[str, Any]] = None):
        """Called from ROS thread. Marshal into the asyncio loop.

        `augmentations` rides along on the MessageEvent metadata. Orchestrator
        skills inspect this dict to:
            - augment the system prompt with current_language hint
            - include context_warm facts directly (skip recall round-trip)
            - attach frame_b64 to the LLM call as multimodal input
        """
        if self._loop is None:
            return
        source = self.build_source(
            chat_id=CHAT_ID,
            chat_name="Robot",
            chat_type="dm",
            user_id=USER_ID,
            user_name="Physical World",
        ) if HERMES_AVAILABLE else {
            "chat_id": CHAT_ID, "chat_type": "dm", "user_id": USER_ID,
            "platform": "ros2",
        }
        metadata = {"augmentations": augmentations} if augmentations else None
        evt = MessageEvent(
            text=formatted_text,
            message_type=MessageType.TEXT,
            source=source,
            message_id=str(int(time.time() * 1000)),
            timestamp=_dt.datetime.now(),
            metadata=metadata,
        )
        asyncio.run_coroutine_threadsafe(self.handle_message(evt), self._loop)


# ============================================================
# Plugin registration entry point — Hermes calls this on load
# ============================================================

def register(ctx):
    """Hermes plugin entry point. `ctx` is the platform-registration context.

    Signature confirmed informally from the framework docs; if the real ctx
    API differs (e.g., uses register() with different kwargs), adjust here.
    """
    ctx.register_platform(
        name="ros2",
        label="ROS2",
        adapter_factory=lambda config: Ros2Adapter(config),
    )
