"""RobotBridge — thin MCP-client facade for Hermes skill scripts.

Same public API as the old rclpy-based RobotBridge (set_face, speak,
who_is_here, register_person, …). Internally, every method dispatches a
single MCP `tools/call` to the ros2_bridge_daemon running in system
Python 3.10 on http://127.0.0.1:8765/mcp-http/mcp. That keeps rclpy out
of the Hermes Python 3.11 venv where this module is installed.

Each `RobotBridge()` call returns a singleton that reuses one MCP session
across multiple tool invocations from the same skill subprocess. Skill
scripts continue to write:

    from robot_bridge import RobotBridge
    rb = RobotBridge()
    print(rb.speak("hello"))

with no other changes. The return values are now dicts (matching the
daemon's tool surface) rather than the prior bespoke strings; skill
scripts already `print(...)` the return value, so JSON dicts are at
least as readable.
"""

from __future__ import annotations

import asyncio
import os
import threading
from typing import Any, Optional

# We avoid taking a hard dependency on the official mcp client SDK here —
# it pulls in a large async stack we don't need for one-shot tool calls.
# A raw httpx POST against the Streamable HTTP endpoint with the right
# headers is sufficient.
import httpx


MCP_URL = os.environ.get("ROS2_BRIDGE_MCP_URL",
                          "http://127.0.0.1:8765/mcp-http/mcp")
TIMEOUT_S = float(os.environ.get("ROS2_BRIDGE_TIMEOUT_S", "10"))


class _MCPError(Exception):
    pass


class RobotBridge:
    """Thread-safe singleton dispatching tool calls to ros2_bridge_daemon."""

    _instance: Optional["RobotBridge"] = None
    _instance_lock = threading.Lock()

    def __new__(cls) -> "RobotBridge":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._init()
        return cls._instance

    def _init(self) -> None:
        self._session_id: Optional[str] = None
        self._next_id = 1
        self._call_lock = threading.Lock()

    # ---- protocol plumbing ----

    def _next_rpc_id(self) -> int:
        with self._call_lock:
            n = self._next_id
            self._next_id += 1
            return n

    def _post(self, payload: dict, extra_headers: Optional[dict] = None
              ) -> tuple[int, dict, dict]:
        headers = {
            "Content-Type": "application/json",
            # Streamable HTTP servers may reply with either content type;
            # accept both per the MCP spec.
            "Accept": "application/json, text/event-stream",
        }
        if self._session_id:
            headers["Mcp-Session-Id"] = self._session_id
        if extra_headers:
            headers.update(extra_headers)
        with httpx.Client(timeout=TIMEOUT_S) as client:
            resp = client.post(MCP_URL, json=payload, headers=headers)
        # Some servers stash the session id in a response header on init.
        sid = resp.headers.get("Mcp-Session-Id") or resp.headers.get("mcp-session-id")
        if sid:
            self._session_id = sid
        return resp.status_code, dict(resp.headers), self._parse_body(resp)

    @staticmethod
    def _parse_body(resp: httpx.Response) -> dict:
        ct = (resp.headers.get("content-type") or "").lower()
        if "application/json" in ct:
            try:
                return resp.json()
            except Exception:
                return {"raw": resp.text}
        # SSE: pluck the first `data: {...}` line.
        for line in resp.text.splitlines():
            if line.startswith("data: "):
                import json as _json
                try:
                    return _json.loads(line[len("data: "):])
                except Exception:
                    continue
        return {"raw": resp.text}

    def _initialize(self) -> None:
        if self._session_id is not None:
            return
        init_req = {
            "jsonrpc": "2.0",
            "id": self._next_rpc_id(),
            "method": "initialize",
            "params": {
                "protocolVersion": "2025-03-26",
                "capabilities": {},
                "clientInfo": {"name": "robot_bridge",
                                "version": "0.2.0"},
            },
        }
        self._post(init_req)
        # Send the 'initialized' notification (required by spec).
        self._post({
            "jsonrpc": "2.0",
            "method": "notifications/initialized",
            "params": {},
        })

    def _call_tool(self, name: str, arguments: Optional[dict] = None
                    ) -> Any:
        self._initialize()
        rpc_id = self._next_rpc_id()
        envelope = {
            "jsonrpc": "2.0",
            "id": rpc_id,
            "method": "tools/call",
            "params": {"name": name, "arguments": arguments or {}},
        }
        status, _headers, body = self._post(envelope)
        if status >= 400:
            raise _MCPError(f"HTTP {status}: {body}")
        if "error" in body:
            return {"ok": False, "error": body["error"]}
        # MCP tools/call result: {result: {content: [{type:'text',text:JSON}], ...}}
        result = body.get("result") or {}
        content = result.get("content") or []
        if content and isinstance(content[0], dict):
            text = content[0].get("text", "")
            if text:
                import json as _json
                try:
                    return _json.loads(text)
                except _json.JSONDecodeError:
                    return {"ok": True, "raw": text}
        return result

    # ============================================================
    # Public API — same names as the old RobotBridge so existing skill
    # scripts continue to work. Return values are JSON dicts.
    # ============================================================

    # ---- face / speech ----

    def set_face(self, state: str, amplitude: float = 0.0) -> dict:
        return self._call_tool("set_face",
                                 {"state": state, "amplitude": float(amplitude)})

    def speak(self, text: str, timeout_seconds: float = 30.0) -> dict:
        # The fire-and-forget tool. For blocking speak use speak_sync.
        return self._call_tool("speak", {"text": text})

    def speak_sync(self, text: str, timeout_seconds: float = 30.0) -> dict:
        return self._call_tool("speak_sync",
                                 {"text": text,
                                  "timeout_seconds": float(timeout_seconds)})

    def listen(self, timeout_seconds: float = 15.0) -> str:
        result = self._call_tool("listen",
                                   {"timeout_seconds": float(timeout_seconds)})
        return result.get("text", "") if isinstance(result, dict) else str(result)

    # ---- identity ----

    def who_is_here(self) -> dict:
        return self._call_tool("who_is_here", {})

    def register_person(self, name: str) -> dict:
        return self._call_tool("register_person", {"name": name})

    def list_persons(self) -> list:
        result = self._call_tool("list_persons", {})
        return result.get("persons", []) if isinstance(result, dict) else []

    def forget_person(self, person_id: str) -> dict:
        return self._call_tool("forget_person", {"person_id": person_id})

    # ---- location ----

    def where_am_i(self) -> dict:
        return self._call_tool("where_am_i", {})

    def learn_location(self, name: str, parent: str = "") -> dict:
        return self._call_tool("learn_location",
                                 {"name": name, "parent": parent})

    def set_current_location(self, name: str, parent: str = "") -> dict:
        return self._call_tool("set_current_location",
                                 {"name": name, "parent": parent})

    def list_locations(self) -> list:
        result = self._call_tool("list_locations", {})
        return result.get("locations", []) if isinstance(result, dict) else []

    # ---- memory ----

    def remember(self, subject_id: str, subject_type: str, content: str,
                 tags: str = "", source: str = "manual",
                 confidence: float = 1.0) -> dict:
        return self._call_tool("remember",
            {"subject_id": subject_id, "subject_type": subject_type,
             "content": content, "tags": tags, "source": source,
             "confidence": float(confidence)})

    def recall(self, subject_id: str, subject_type: str = "Person",
               query: str = "", limit: int = 10) -> list:
        result = self._call_tool("recall",
            {"subject_id": subject_id, "subject_type": subject_type,
             "query": query, "limit": int(limit)})
        return result.get("facts", []) if isinstance(result, dict) else []

    def relate_persons(self, a_id: str, b_id: str, relation: str,
                       description: str = "",
                       bidirectional: bool = False) -> dict:
        return self._call_tool("relate_persons",
            {"a_id": a_id, "b_id": b_id, "relation": relation,
             "description": description, "bidirectional": bool(bidirectional)})

    def find_related(self, subject_id: str, relation: str = "",
                     hops: int = 1) -> list:
        result = self._call_tool("find_related",
            {"subject_id": subject_id, "relation": relation,
             "hops": int(hops)})
        return result.get("related", []) if isinstance(result, dict) else []

    # ---- self (Cypher-backed convenience methods) ----

    def who_am_i(self) -> dict:
        # The MCP daemon doesn't expose a 'who_am_i' tool; build it via
        # cypher() so the surface stays identical.
        result = self._call_tool("cypher", {
            "query": "MATCH (s:Self) RETURN s.id, s.name, s.owner_id, s.preferences",
            "params": {},
        })
        rows = result.get("rows", []) if isinstance(result, dict) else []
        if not rows:
            return {"name": "Hermes", "preferences": "{}"}
        sid, name, owner, prefs = rows[0]
        return {"id": sid, "name": name, "owner_id": owner,
                "preferences": prefs}

    def update_self_preferences(self, prefs: dict) -> dict:
        import json as _json
        cur = self.who_am_i()
        try:
            existing = _json.loads(cur.get("preferences", "{}"))
        except _json.JSONDecodeError:
            existing = {}
        existing.update(prefs)
        return self._call_tool("cypher", {
            "query": ("MERGE (s:Self {id:'self'}) "
                      "SET s.preferences=$prefs, "
                      "s.name=COALESCE(s.name, 'Hermes')"),
            "params": {"prefs": _json.dumps(existing)},
        })

    def add_self_fact(self, content: str) -> dict:
        return self.remember("self", "Self", content,
                              source="self-reflection")

    # ---- journal (file-based, no rclpy needed) ----
    #
    # The journal files live on the same host the daemon runs on, so we
    # read them directly here. This keeps the call cheap (no MCP round
    # trip for what's essentially a local file read).

    def read_journal_windows(self, mode: str = "incremental",
                              max_entries: int = 500) -> list:
        import gzip
        import json as _json
        from datetime import datetime
        from pathlib import Path

        journal_dir = Path.home() / "robot_data" / "journal"
        if not journal_dir.exists():
            return []

        def load_day(date: str) -> list:
            plain = journal_dir / f"{date}.jsonl"
            gz = journal_dir / f"{date}.jsonl.gz"
            if plain.exists():
                opener = lambda: open(plain, "r", encoding="utf-8")
            elif gz.exists():
                opener = lambda: gzip.open(gz, "rt", encoding="utf-8")
            else:
                return []
            out = []
            with opener() as f:
                for line in f:
                    try:
                        out.append(_json.loads(line))
                    except _json.JSONDecodeError:
                        continue
            return out

        def bucketize(entries: list, gap_seconds: int = 300) -> list:
            if not entries:
                return []
            windows = []
            current = [entries[0]]
            for prev, cur in zip(entries, entries[1:]):
                try:
                    dt_prev = datetime.fromisoformat(
                        prev["t"].replace("Z", "+00:00"))
                    dt_cur = datetime.fromisoformat(
                        cur["t"].replace("Z", "+00:00"))
                except (KeyError, ValueError):
                    current.append(cur)
                    continue
                if (dt_cur - dt_prev).total_seconds() > gap_seconds:
                    windows.append(current)
                    current = []
                current.append(cur)
            if current:
                windows.append(current)
            return [{"start": w[0]["t"], "end": w[-1]["t"],
                      "entries": w} for w in windows]

        if mode == "daily":
            target_date = datetime.now().strftime("%Y-%m-%d")
            return load_day(target_date)

        checkpoint_file = journal_dir / "checkpoint.json"
        try:
            checkpoint = (_json.loads(checkpoint_file.read_text())
                          if checkpoint_file.exists() else {})
        except _json.JSONDecodeError:
            checkpoint = {}
        last_t = checkpoint.get("last_timestamp", "")
        entries = load_day(datetime.now().strftime("%Y-%m-%d"))
        entries = [e for e in entries if e.get("t", "") > last_t]
        return bucketize(entries[:max_entries])

    def forget_day(self, date: str) -> dict:
        from pathlib import Path
        journal_dir = Path.home() / "robot_data" / "journal"
        deleted = []
        for path in (journal_dir / f"{date}.jsonl",
                      journal_dir / f"{date}.jsonl.gz"):
            if path.exists():
                path.unlink()
                deleted.append(path.name)
        result = self._call_tool("cypher", {
            "query": ("MATCH (e:Episode) "
                      "WHERE date(e.occurred_at) = date($d) "
                      "DETACH DELETE e"),
            "params": {"d": date},
        })
        return {"deleted_files": deleted, "graph": result}
