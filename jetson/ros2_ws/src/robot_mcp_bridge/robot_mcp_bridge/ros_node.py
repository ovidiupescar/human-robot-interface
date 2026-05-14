"""Long-lived rclpy node used by the bridge daemon.

This module owns all ROS2 graph access. The MCP tools and the events WS
endpoint both call into the singleton `RosBridge` instance — they never
touch rclpy directly. That single ownership keeps the DDS graph simple
(one node, one set of subscriptions) and lets us shut down cleanly.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_perception_msgs.msg import TtsRequest


class _BridgeNode(Node):
    """Single ROS2 node holding publishers, subscriptions and service clients.

    Methods are kept thin: they translate Python args to ROS messages, fire,
    and return synchronously. The owning RosBridge serializes access from
    HTTP request handlers via short critical sections.
    """

    def __init__(self) -> None:
        super().__init__("robot_mcp_bridge")

        # Publishers
        self._tts_pub = self.create_publisher(TtsRequest, "/tts/say", 10)

    # ---- outbound: TTS ----

    def publish_tts(self, text: str, language: str = "", voice: str = "") -> None:
        msg = TtsRequest()
        msg.text = text
        msg.language = language
        msg.voice = voice
        self._tts_pub.publish(msg)


class RosBridge:
    """Singleton wrapping the rclpy node and its background spin thread."""

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

    # ---- public methods called from MCP tools ----

    def speak(self, text: str, language: str = "", voice: str = "") -> dict[str, Any]:
        """Publish a TTS request. Returns immediately — does not block on synthesis.

        The downstream tts_service node queues the request and audio_player
        plays it through the speakers.
        """
        if not text:
            return {"ok": False, "error": "empty text"}
        self._node.publish_tts(text=text, language=language, voice=voice)
        return {"ok": True, "text": text, "language": language or "default"}
