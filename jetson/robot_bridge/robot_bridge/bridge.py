"""RobotBridge — singleton facade used by Hermes skill scripts.

All Hermes skills import this and call its methods. It manages a single rclpy
node that talks to the live ROS2 graph (face, speech, knowledge graph services,
journal).
"""

from __future__ import annotations

import gzip
import json
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32, String

from robot_face_msgs.msg import FaceCommand
from robot_face_msgs.srv import Speak

# Optional — only available once robot_graph_msgs is built
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

from .states import STATE_NAME_TO_INT, FaceStateName


class _BridgeNode(Node):
    def __init__(self):
        super().__init__('hermes_robot_bridge')

        # Face
        self._face_pub = self.create_publisher(FaceCommand, '/face/command', 10)

        # Speech
        self._speak_cli = self.create_client(Speak, '/speak')

        # Transcripts
        self._last_transcript: Optional[str] = None
        self._transcript_event = threading.Event()
        self.create_subscription(String, '/perception/transcript',
                                 self._on_transcript, 10)

        # Current state caches
        self._last_person: Optional[Any] = None
        self._last_location: Optional[Any] = None
        self._last_addressee: float = 0.5

        if GRAPH_AVAILABLE:
            self.create_subscription(PersonIdentity,
                                     '/perception/identified_person',
                                     self._on_person, 10)
            self.create_subscription(LocationIdentity,
                                     '/perception/current_location',
                                     self._on_location, 10)
            self.create_subscription(Float32,
                                     '/perception/addressee_score',
                                     self._on_addressee, 10)

            self._identify_voice_cli = None  # built-on-demand
            self._register_person_cli = self.create_client(RegisterPerson,
                                                            '/identity/register_person')
            self._rename_person_cli = self.create_client(RenamePerson,
                                                         '/identity/rename_person')
            self._list_persons_cli = self.create_client(ListPersons,
                                                        '/identity/list_persons')
            self._forget_person_cli = self.create_client(ForgetPerson,
                                                         '/identity/forget_person')

            self._identify_loc_cli = self.create_client(IdentifyLocation,
                                                        '/location/identify')
            self._learn_loc_cli = self.create_client(LearnLocation,
                                                     '/location/learn')
            self._list_loc_cli = self.create_client(ListLocations,
                                                    '/location/list')
            self._set_loc_cli = self.create_client(SetCurrentLocation,
                                                   '/location/set_current')

            self._remember_cli = self.create_client(Remember, '/memory/remember')
            self._recall_cli = self.create_client(Recall, '/memory/recall')
            self._relate_cli = self.create_client(Relate, '/memory/relate')
            self._find_related_cli = self.create_client(FindRelated,
                                                         '/memory/find_related')
            self._cypher_cli = self.create_client(CypherQuery, '/graph/cypher')

    def _on_transcript(self, msg: String):
        self._last_transcript = msg.data
        self._transcript_event.set()

    def _on_person(self, msg):
        self._last_person = msg

    def _on_location(self, msg):
        self._last_location = msg

    def _on_addressee(self, msg: Float32):
        self._last_addressee = float(msg.data)


class RobotBridge:
    """Thread-safe singleton wrapping a long-lived rclpy node."""

    _instance: Optional["RobotBridge"] = None
    _lock = threading.Lock()

    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._init()
        return cls._instance

    def _init(self):
        if not rclpy.ok():
            rclpy.init()
        self._node = _BridgeNode()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin.start()

    # ---- helpers ----

    def _call(self, client, request, timeout: float = 10.0):
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        future = client.call_async(request)
        deadline = time.time() + timeout
        while time.time() < deadline and not future.done():
            time.sleep(0.02)
        return future.result()

    # ---- face ----

    def set_face(self, state: FaceStateName, amplitude: float = 0.0) -> str:
        if state not in STATE_NAME_TO_INT:
            return f"error: unknown state '{state}'"
        m = FaceCommand()
        m.state = STATE_NAME_TO_INT[state]
        m.amplitude = max(0.0, min(1.0, float(amplitude)))
        self._node._face_pub.publish(m)
        return f"face -> {state}"

    # ---- speech ----

    def speak(self, text: str, timeout_seconds: float = 30.0) -> str:
        req = Speak.Request()
        req.text = text
        resp = self._call(self._node._speak_cli, req, timeout=timeout_seconds)
        if resp is None:
            return "error: speak unavailable or timed out"
        return (f"spoke ({resp.duration_seconds:.2f}s)"
                if resp.success else f"error: {resp.message}")

    def listen(self, timeout_seconds: float = 15.0) -> str:
        self._node._transcript_event.clear()
        self._node._last_transcript = None
        if self._node._transcript_event.wait(timeout=timeout_seconds):
            return self._node._last_transcript or ""
        return ""

    # ---- identity ----

    def who_is_here(self) -> dict:
        p = self._node._last_person
        if p is None:
            return {"present": False}
        return {
            "present": True,
            "person_id": p.person_id,
            "name": p.primary_name,
            "voice_confidence": float(p.voice_confidence),
            "is_new": bool(p.is_new),
        }

    def register_person(self, name: str) -> dict:
        if not GRAPH_AVAILABLE:
            return {"success": False, "message": "graph msgs not built"}
        req = RegisterPerson.Request()
        req.name = name
        req.audio_pcm_int16 = []
        req.sample_rate = 16000
        resp = self._call(self._node._register_person_cli, req)
        if resp is None:
            return {"success": False, "message": "service unavailable"}
        return {"success": resp.success, "message": resp.message,
                "person_id": resp.identity.person_id, "name": resp.identity.primary_name}

    def list_persons(self) -> list:
        if not GRAPH_AVAILABLE:
            return []
        resp = self._call(self._node._list_persons_cli, ListPersons.Request())
        if resp is None:
            return []
        return [{"id": p.person_id, "name": p.primary_name} for p in resp.persons]

    def forget_person(self, person_id: str) -> dict:
        if not GRAPH_AVAILABLE:
            return {"success": False, "message": "graph msgs not built"}
        req = ForgetPerson.Request()
        req.person_id = person_id
        resp = self._call(self._node._forget_person_cli, req)
        return {"success": bool(resp and resp.success),
                "message": resp.message if resp else ""}

    # ---- location ----

    def where_am_i(self) -> dict:
        l = self._node._last_location
        if l is None or getattr(l, 'is_unknown', True):
            return {"known": False}
        return {"known": True, "location_id": l.location_id, "name": l.name,
                "parent": l.parent_name, "confidence": float(l.confidence)}

    def learn_location(self, name: str, parent: str = "") -> dict:
        if not GRAPH_AVAILABLE:
            return {"success": False, "message": "graph msgs not built"}
        req = LearnLocation.Request()
        req.name = name
        req.parent_name = parent
        req.sample_count = 5
        resp = self._call(self._node._learn_loc_cli, req)
        return {"success": bool(resp and resp.success),
                "message": resp.message if resp else "",
                "location_id": resp.identity.location_id if resp else ""}

    def set_current_location(self, name: str, parent: str = "") -> dict:
        if not GRAPH_AVAILABLE:
            return {"success": False, "message": "graph msgs not built"}
        req = SetCurrentLocation.Request()
        req.name = name
        req.parent_name = parent
        resp = self._call(self._node._set_loc_cli, req)
        return {"success": bool(resp and resp.success),
                "message": resp.message if resp else "",
                "location_id": resp.identity.location_id if resp else ""}

    def list_locations(self) -> list:
        if not GRAPH_AVAILABLE:
            return []
        resp = self._call(self._node._list_loc_cli, ListLocations.Request())
        if resp is None:
            return []
        return [{"id": l.location_id, "name": l.name, "parent": l.parent_name}
                for l in resp.locations]

    # ---- memory ----

    def remember(self, subject_id: str, subject_type: str, content: str,
                 tags: str = "", source: str = "manual",
                 confidence: float = 1.0) -> dict:
        if not GRAPH_AVAILABLE:
            return {"success": False}
        req = Remember.Request()
        req.subject_id = subject_id
        req.subject_type = subject_type
        req.content = content
        req.tags = tags
        req.source = source
        req.confidence = confidence
        resp = self._call(self._node._remember_cli, req)
        return {"success": bool(resp and resp.success),
                "fact_id": resp.fact_id if resp else ""}

    def recall(self, subject_id: str, subject_type: str = "Person",
               query: str = "", limit: int = 10) -> list:
        if not GRAPH_AVAILABLE:
            return []
        req = Recall.Request()
        req.subject_id = subject_id
        req.subject_type = subject_type
        req.query = query
        req.limit = limit
        resp = self._call(self._node._recall_cli, req)
        if resp is None:
            return []
        out = []
        for fid, content, score in zip(resp.fact_ids, resp.contents, resp.scores):
            out.append({"id": fid, "content": content, "score": float(score)})
        return out

    def relate_persons(self, a_id: str, b_id: str, relation: str,
                       description: str = "", bidirectional: bool = False) -> dict:
        if not GRAPH_AVAILABLE:
            return {"success": False}
        req = Relate.Request()
        req.subject_a_id = a_id
        req.subject_b_id = b_id
        req.subject_type = "Person"
        req.relation = relation
        req.description = description
        req.bidirectional = bidirectional
        resp = self._call(self._node._relate_cli, req)
        return {"success": bool(resp and resp.success)}

    def find_related(self, subject_id: str, relation: str = "",
                     hops: int = 1) -> list:
        if not GRAPH_AVAILABLE:
            return []
        req = FindRelated.Request()
        req.subject_id = subject_id
        req.subject_type = "Person"
        req.relation = relation
        req.hops = hops
        resp = self._call(self._node._find_related_cli, req)
        if resp is None:
            return []
        return [{"id": i, "name": n} for i, n in
                zip(resp.related_ids, resp.related_names)]

    # ---- self ----

    def who_am_i(self) -> dict:
        # Query :Self node via Cypher
        result = self._cypher(
            "MATCH (s:Self) RETURN s.id, s.name, s.owner_id, s.preferences",
            {})
        rows = result.get("rows", [])
        if not rows:
            return {"name": "Hermes", "preferences": "{}"}
        sid, name, owner, prefs = rows[0]
        return {"id": sid, "name": name, "owner_id": owner,
                "preferences": prefs}

    def update_self_preferences(self, prefs: dict) -> dict:
        cur = self.who_am_i()
        try:
            existing = json.loads(cur.get("preferences", "{}"))
        except json.JSONDecodeError:
            existing = {}
        existing.update(prefs)
        result = self._cypher(
            "MERGE (s:Self {id:'self'}) "
            "SET s.preferences=$prefs, s.name=COALESCE(s.name, 'Hermes')",
            {"prefs": json.dumps(existing)},
        )
        return result

    def add_self_fact(self, content: str) -> dict:
        return self.remember("self", "Self", content, source="self-reflection")

    def _cypher(self, cypher: str, params: dict) -> dict:
        if not GRAPH_AVAILABLE:
            return {"success": False, "rows": []}
        req = CypherQuery.Request()
        req.cypher = cypher
        req.params_json = json.dumps(params)
        resp = self._call(self._node._cypher_cli, req)
        if resp is None:
            return {"success": False, "rows": []}
        try:
            rows = json.loads(resp.result_json)
        except json.JSONDecodeError:
            rows = []
        return {"success": resp.success, "message": resp.message, "rows": rows}

    # ---- journal ----

    def read_journal_windows(self, mode: str = "incremental",
                              max_entries: int = 500) -> list:
        """Read journal entries since checkpoint and split into silence-bounded windows.

        For now reads from disk directly (~/robot_data/journal/).
        Production: also call /memorist/read_window service for atomic checkpointing.
        """
        journal_dir = Path.home() / 'robot_data' / 'journal'
        if not journal_dir.exists():
            return []

        checkpoint_file = journal_dir / 'checkpoint.json'
        try:
            checkpoint = json.loads(checkpoint_file.read_text()) if checkpoint_file.exists() else {}
        except json.JSONDecodeError:
            checkpoint = {}

        if mode == "daily":
            # Last full day's file
            target_date = (datetime.now()).strftime('%Y-%m-%d')
            return self._load_day(journal_dir, target_date)

        # Incremental — read today since checkpoint
        last_t = checkpoint.get('last_timestamp', '')
        entries = self._load_day(journal_dir, datetime.now().strftime('%Y-%m-%d'))
        entries = [e for e in entries if e.get('t', '') > last_t]
        return self._bucketize(entries[:max_entries])

    def _load_day(self, journal_dir: Path, date: str) -> list:
        plain = journal_dir / f'{date}.jsonl'
        gz = journal_dir / f'{date}.jsonl.gz'
        if plain.exists():
            opener = lambda: open(plain, 'r', encoding='utf-8')
        elif gz.exists():
            opener = lambda: gzip.open(gz, 'rt', encoding='utf-8')
        else:
            return []
        out = []
        with opener() as f:
            for line in f:
                try:
                    out.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
        return out

    def _bucketize(self, entries: list, gap_seconds: int = 300) -> list:
        if not entries:
            return []
        windows = []
        current = [entries[0]]
        for prev, cur in zip(entries, entries[1:]):
            try:
                dt_prev = datetime.fromisoformat(prev['t'].replace('Z', '+00:00'))
                dt_cur = datetime.fromisoformat(cur['t'].replace('Z', '+00:00'))
            except (KeyError, ValueError):
                current.append(cur)
                continue
            if (dt_cur - dt_prev).total_seconds() > gap_seconds:
                windows.append(current)
                current = []
            current.append(cur)
        if current:
            windows.append(current)
        return [{"start": w[0]['t'], "end": w[-1]['t'], "entries": w}
                for w in windows]

    def forget_day(self, date: str) -> dict:
        """Delete journal file for date + remove Episodes that day from graph."""
        journal_dir = Path.home() / 'robot_data' / 'journal'
        deleted = []
        for path in (journal_dir / f'{date}.jsonl',
                     journal_dir / f'{date}.jsonl.gz'):
            if path.exists():
                path.unlink()
                deleted.append(path.name)
        # Cascade in graph
        result = self._cypher(
            "MATCH (e:Episode) "
            "WHERE date(e.occurred_at) = date($d) "
            "DETACH DELETE e",
            {"d": date},
        )
        return {"deleted_files": deleted, "graph": result}
