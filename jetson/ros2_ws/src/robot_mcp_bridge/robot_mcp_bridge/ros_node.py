"""Long-lived rclpy node used by the bridge daemon.

This module owns all ROS2 graph access. The MCP tools and the events WS
endpoint both call into the singleton `RosBridge` instance — they never
touch rclpy directly. That single ownership keeps the DDS graph simple
(one node, one set of subscriptions) and lets us shut down cleanly.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, String

from robot_face_msgs.msg import FaceCommand
from robot_face_msgs.srv import Speak
from robot_perception_msgs.msg import TtsRequest

# Graph msg/srv types are optional — only available once robot_graph_msgs
# is built. Wrap so the daemon still starts on a partial workspace.
try:
    from robot_graph_msgs.msg import LocationIdentity, PersonIdentity
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

    def __init__(self) -> None:
        super().__init__("robot_mcp_bridge")

        # Publishers
        self._tts_pub = self.create_publisher(TtsRequest, "/tts/say", 10)
        self._face_pub = self.create_publisher(FaceCommand, "/face/command", 10)

        # Speech (legacy synchronous API)
        self._speak_cli = self.create_client(Speak, "/speak")

        # Latest transcript cache + wakeup event for `listen()`
        self._last_transcript: Optional[str] = None
        self._transcript_event = threading.Event()
        self.create_subscription(String, "/perception/transcript",
                                 self._on_transcript, 10)

        # Perception state caches (also used to format event payloads later).
        self._last_person: Any = None
        self._last_location: Any = None
        self._last_addressee: float = 0.5

        # Graph service clients — created lazily when first needed so the
        # daemon doesn't pay the setup cost if the user never asks for memory.
        self._graph_clients: dict[str, Any] = {}

        if GRAPH_AVAILABLE:
            self.create_subscription(PersonIdentity,
                                     "/perception/identified_person",
                                     self._on_person, 10)
            self.create_subscription(LocationIdentity,
                                     "/perception/current_location",
                                     self._on_location, 10)
            self.create_subscription(Float32,
                                     "/perception/addressee_score",
                                     self._on_addressee, 10)

    # ---- subscriber callbacks ----

    def _on_transcript(self, msg: String):
        self._last_transcript = msg.data
        self._transcript_event.set()

    def _on_person(self, msg):
        self._last_person = msg

    def _on_location(self, msg):
        self._last_location = msg

    def _on_addressee(self, msg: Float32):
        self._last_addressee = float(msg.data)

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
