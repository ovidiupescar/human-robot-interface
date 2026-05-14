"""KuzuDB connection + common operations.

Thin facade around kuzu. Keeps Cypher strings central so we can refactor later.
"""

from __future__ import annotations

import os
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Optional

import numpy as np

try:
    import kuzu
except ImportError:
    kuzu = None  # type: ignore

from .schema import init_schema


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def new_id(prefix: str) -> str:
    return f"{prefix}_{uuid.uuid4().hex[:12]}"


class GraphStore:
    """Single-process Kuzu wrapper. Thread-safe via internal lock."""

    def __init__(self, db_path: Optional[str] = None):
        if kuzu is None:
            raise RuntimeError("kuzu not installed: pip install kuzu")
        if db_path is None:
            db_path = str(Path.home() / 'robot_data' / 'graph.kuzu')
        Path(db_path).parent.mkdir(parents=True, exist_ok=True)
        self._db = kuzu.Database(db_path)
        self._conn = kuzu.Connection(self._db)
        self._lock = threading.Lock()
        init_schema(self._conn)

    # ---- raw access ----

    def execute(self, query: str, params: Optional[dict] = None):
        with self._lock:
            return self._conn.execute(query, parameters=params or {})

    # ---- Person ----

    def upsert_person(self, person_id: Optional[str], primary_name: str = "",
                      notes: str = "") -> str:
        pid = person_id or new_id("p")
        # Kuzu requires primary key in CREATE; emulate UPSERT manually
        res = self.execute(
            "MATCH (p:Person {id:$id}) RETURN p.id", {"id": pid}
        )
        exists = res.has_next()
        if exists:
            self.execute(
                "MATCH (p:Person {id:$id}) SET p.last_seen=$now",
                {"id": pid, "now": utc_now()},
            )
        else:
            self.execute(
                "CREATE (p:Person {id:$id, primary_name:$name, notes:$notes, "
                "created_at:$now, last_seen:$now})",
                {"id": pid, "name": primary_name, "notes": notes, "now": utc_now()},
            )
        return pid

    def rename_person(self, person_id: str, new_name: str):
        self.execute(
            "MATCH (p:Person {id:$id}) SET p.primary_name=$name",
            {"id": person_id, "name": new_name},
        )

    def add_alias(self, person_id: str, alias: str, weight: float = 1.0):
        aid = new_id("alias")
        self.execute(
            "CREATE (a:Alias {id:$aid, person_id:$pid, alias:$alias, weight:$w})",
            {"aid": aid, "pid": person_id, "alias": alias, "w": float(weight)},
        )
        self.execute(
            "MATCH (p:Person {id:$pid}), (a:Alias {id:$aid}) "
            "MERGE (p)-[:HAS_ALIAS]->(a)",
            {"pid": person_id, "aid": aid},
        )

    def forget_person(self, person_id: str):
        # Cascade delete via Cypher
        self.execute(
            "MATCH (p:Person {id:$id})-[r*0..]-(x) DETACH DELETE p, x",
            {"id": person_id},
        )

    def list_persons(self) -> list[dict]:
        res = self.execute("MATCH (p:Person) RETURN p.id, p.primary_name")
        return [{"id": row[0], "name": row[1]} for row in _rows(res)]

    # ---- VoiceSample ----

    def add_voice_sample(self, person_id: str, embedding: np.ndarray,
                         model: str) -> str:
        vid = new_id("v")
        emb = embedding.astype(np.float32).tolist()
        self.execute(
            "CREATE (v:VoiceSample {id:$id, person_id:$pid, model:$m, dim:$d, "
            "embedding:$emb, captured_at:$now})",
            {"id": vid, "pid": person_id, "m": model, "d": len(emb),
             "emb": emb, "now": utc_now()},
        )
        self.execute(
            "MATCH (p:Person {id:$pid}), (v:VoiceSample {id:$vid}) "
            "MERGE (p)-[:HAS_VOICE]->(v)",
            {"pid": person_id, "vid": vid},
        )
        return vid

    def all_voice_embeddings(self) -> list[tuple[str, np.ndarray]]:
        """Return (person_id, embedding) for every known voice sample."""
        res = self.execute(
            "MATCH (p:Person)-[:HAS_VOICE]->(v:VoiceSample) "
            "RETURN p.id, v.embedding"
        )
        out: list[tuple[str, np.ndarray]] = []
        for row in _rows(res):
            out.append((row[0], np.asarray(row[1], dtype=np.float32)))
        return out

    # ---- Location ----

    def upsert_location(self, name: str, parent_name: str = "",
                        description: str = "") -> str:
        # one Location per unique name
        res = self.execute(
            "MATCH (l:Location {name:$n}) RETURN l.id", {"n": name}
        )
        rows = list(_rows(res))
        if rows:
            return rows[0][0]
        lid = new_id("loc")
        self.execute(
            "CREATE (l:Location {id:$id, name:$n, description:$d, created_at:$now})",
            {"id": lid, "n": name, "d": description, "now": utc_now()},
        )
        if parent_name:
            self._link_parent_location(lid, parent_name)
        return lid

    def _link_parent_location(self, child_id: str, parent_name: str):
        pid = self.upsert_location(parent_name)
        self.execute(
            "MATCH (c:Location {id:$cid}), (p:Location {id:$pid}) "
            "MERGE (c)-[:PART_OF]->(p)",
            {"cid": child_id, "pid": pid},
        )

    def add_location_sample(self, location_id: str, embedding: np.ndarray,
                            model: str, image_path: str = "") -> str:
        sid = new_id("ls")
        emb = embedding.astype(np.float32).tolist()
        self.execute(
            "CREATE (s:LocationSample {id:$id, location_id:$lid, model:$m, dim:$d, "
            "embedding:$emb, captured_at:$now, image_path:$ip})",
            {"id": sid, "lid": location_id, "m": model, "d": len(emb),
             "emb": emb, "now": utc_now(), "ip": image_path},
        )
        self.execute(
            "MATCH (l:Location {id:$lid}), (s:LocationSample {id:$sid}) "
            "MERGE (l)-[:HAS_SAMPLE]->(s)",
            {"lid": location_id, "sid": sid},
        )
        return sid

    def all_location_embeddings(self) -> list[tuple[str, str, np.ndarray]]:
        res = self.execute(
            "MATCH (l:Location)-[:HAS_SAMPLE]->(s:LocationSample) "
            "RETURN l.id, l.name, s.embedding"
        )
        out = []
        for row in _rows(res):
            out.append((row[0], row[1], np.asarray(row[2], dtype=np.float32)))
        return out

    def list_locations(self) -> list[dict]:
        res = self.execute(
            "MATCH (l:Location) "
            "OPTIONAL MATCH (l)-[:PART_OF]->(p:Location) "
            "RETURN l.id, l.name, COALESCE(p.name, '')"
        )
        return [{"id": r[0], "name": r[1], "parent": r[2]} for r in _rows(res)]

    # ---- Episode / Fact / Event ----

    def create_episode(self, content: str, occurred_at: datetime,
                       duration_s: float = 0.0,
                       embedding: Optional[np.ndarray] = None) -> str:
        eid = new_id("ep")
        emb = (embedding.astype(np.float32).tolist() if embedding is not None
               else [])
        self.execute(
            "CREATE (e:Episode {id:$id, content:$c, occurred_at:$t, duration_s:$d, "
            "embedding:$emb, consolidated:false, created_at:$now})",
            {"id": eid, "c": content, "t": occurred_at, "d": float(duration_s),
             "emb": emb, "now": utc_now()},
        )
        return eid

    def link_episode_location(self, episode_id: str, location_id: str):
        self.execute(
            "MATCH (e:Episode {id:$eid}), (l:Location {id:$lid}) "
            "MERGE (e)-[:OCCURRED_AT]->(l)",
            {"eid": episode_id, "lid": location_id},
        )

    def link_episode_person(self, episode_id: str, person_id: str):
        self.execute(
            "MATCH (e:Episode {id:$eid}), (p:Person {id:$pid}) "
            "MERGE (e)-[:INVOLVES]->(p)",
            {"eid": episode_id, "pid": person_id},
        )

    def create_fact(self, content: str, confidence: float = 1.0,
                    source: str = "manual", tags: str = "",
                    embedding: Optional[np.ndarray] = None,
                    about_person_id: Optional[str] = None,
                    about_location_id: Optional[str] = None,
                    derived_from_episode_id: Optional[str] = None) -> str:
        fid = new_id("f")
        emb = (embedding.astype(np.float32).tolist() if embedding is not None
               else [])
        self.execute(
            "CREATE (f:Fact {id:$id, content:$c, confidence:$conf, source:$s, "
            "tags:$tg, embedding:$emb, created_at:$now, last_referenced:$now})",
            {"id": fid, "c": content, "conf": float(confidence), "s": source,
             "tg": tags, "emb": emb, "now": utc_now()},
        )
        if about_person_id:
            self.execute(
                "MATCH (f:Fact {id:$fid}), (p:Person {id:$pid}) "
                "MERGE (f)-[:ABOUT_PERSON]->(p)",
                {"fid": fid, "pid": about_person_id},
            )
        if about_location_id:
            self.execute(
                "MATCH (f:Fact {id:$fid}), (l:Location {id:$lid}) "
                "MERGE (f)-[:ABOUT_LOCATION]->(l)",
                {"fid": fid, "lid": about_location_id},
            )
        if derived_from_episode_id:
            self.execute(
                "MATCH (f:Fact {id:$fid}), (e:Episode {id:$eid}) "
                "MERGE (f)-[:DERIVED_FROM]->(e)",
                {"fid": fid, "eid": derived_from_episode_id},
            )
        return fid

    def recall_facts_about_person(self, person_id: str, limit: int = 10) -> list[dict]:
        res = self.execute(
            "MATCH (f:Fact)-[:ABOUT_PERSON]->(p:Person {id:$pid}) "
            "RETURN f.id, f.content, f.confidence "
            "ORDER BY f.created_at DESC LIMIT $lim",
            {"pid": person_id, "lim": int(limit)},
        )
        return [{"id": r[0], "content": r[1], "confidence": r[2]} for r in _rows(res)]

    # ---- Relationships between persons ----

    def relate(self, a_id: str, b_id: str, relation: str,
               description: str = "", bidirectional: bool = False):
        rel_table = relation.upper()
        # rel_table must already exist in schema. Generic catch-all = KNOWS with context.
        # To keep things flexible we store unknown relations as KNOWS with context=<relation>.
        if rel_table not in {"KNOWS", "PARENT_OF", "SIBLING_OF",
                              "PARTNER_OF", "COLLEAGUE_OF", "MENTOR_OF"}:
            self.execute(
                "MATCH (a:Person {id:$a}), (b:Person {id:$b}) "
                "MERGE (a)-[:KNOWS {context:$ctx, since:$now}]->(b)",
                {"a": a_id, "b": b_id, "ctx": f"{relation}: {description}",
                 "now": utc_now()},
            )
            if bidirectional:
                self.execute(
                    "MATCH (a:Person {id:$a}), (b:Person {id:$b}) "
                    "MERGE (b)-[:KNOWS {context:$ctx, since:$now}]->(a)",
                    {"a": a_id, "b": b_id, "ctx": f"{relation}: {description}",
                     "now": utc_now()},
                )
            return
        self.execute(
            f"MATCH (a:Person {{id:$a}}), (b:Person {{id:$b}}) "
            f"MERGE (a)-[:{rel_table}]->(b)",
            {"a": a_id, "b": b_id},
        )
        if bidirectional:
            self.execute(
                f"MATCH (a:Person {{id:$a}}), (b:Person {{id:$b}}) "
                f"MERGE (b)-[:{rel_table}]->(a)",
                {"a": a_id, "b": b_id},
            )

    # ---- language preferences ----

    def set_self_default_language(self, language: Optional[str]):
        """Update Self.default_language. Auto-creates the Self node if missing.

        language=None clears the preference.
        """
        # Ensure a Self node exists (singleton id="self")
        self.execute(
            "MERGE (s:Self {id:'self'}) ON CREATE SET s.born_at=$now",
            {"now": utc_now()},
        )
        self.execute(
            "MATCH (s:Self {id:'self'}) SET s.default_language=$lang",
            {"lang": language or ""},
        )

    def get_self_default_language(self) -> Optional[str]:
        res = self.execute(
            "MATCH (s:Self {id:'self'}) RETURN s.default_language"
        )
        for row in _rows(res):
            lang = row[0]
            return lang if lang else None
        return None

    def set_person_language(self, person_id: str, language: Optional[str]):
        self.execute(
            "MATCH (p:Person {id:$id}) SET p.preferred_language=$lang",
            {"id": person_id, "lang": language or ""},
        )

    def get_person_language(self, person_id: str) -> Optional[str]:
        res = self.execute(
            "MATCH (p:Person {id:$id}) RETURN p.preferred_language",
            {"id": person_id},
        )
        for row in _rows(res):
            lang = row[0]
            return lang if lang else None
        return None

    def set_location_language(self, location_id: str, language: Optional[str]):
        self.execute(
            "MATCH (l:Location {id:$id}) SET l.language_override=$lang",
            {"id": location_id, "lang": language or ""},
        )

    def set_event_language(self, event_id: str, language: Optional[str]):
        self.execute(
            "MATCH (e:Event {id:$id}) SET e.language_override=$lang",
            {"id": event_id, "lang": language or ""},
        )

    def find_related_persons(self, person_id: str, relation: str = "",
                              hops: int = 1) -> list[dict]:
        hops = max(1, min(int(hops), 3))
        if relation:
            rel_pat = f":{relation.upper()}"
        else:
            rel_pat = ""
        q = (f"MATCH (p:Person {{id:$pid}})-[r{rel_pat}*1..{hops}]-(o:Person) "
             f"WHERE o.id <> $pid "
             f"RETURN DISTINCT o.id, o.primary_name LIMIT 50")
        res = self.execute(q, {"pid": person_id})
        return [{"id": r[0], "name": r[1]} for r in _rows(res)]


def _rows(result) -> Iterable[tuple]:
    """Iterate Kuzu QueryResult into native python tuples."""
    while result.has_next():
        yield tuple(result.get_next())
