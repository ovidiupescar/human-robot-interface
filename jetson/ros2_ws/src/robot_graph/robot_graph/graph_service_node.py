"""Graph service node — hosts KuzuDB and exposes the full identity/memory/location
service surface to the rest of the system.

Services exposed:
    /identity/identify_voice
    /identity/register_person
    /identity/rename_person
    /identity/list_persons
    /identity/forget_person
    /location/identify        (caller provides JPEG bytes)
    /location/learn
    /location/list
    /location/set_current
    /location/relate
    /memory/remember
    /memory/recall
    /memory/relate
    /memory/find_related
    /graph/cypher
"""

import json
from collections import defaultdict
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from robot_graph_msgs.msg import LocationIdentity, PersonIdentity
from robot_graph_msgs.srv import (
    CypherQuery,
    FindRelated,
    ForgetPerson,
    IdentifyLocation,
    IdentifyVoice,
    LearnLocation,
    ListLocations,
    ListPersons,
    Recall,
    Relate,
    RelateLocations,
    Remember,
    RegisterPerson,
    RenamePerson,
    SetCurrentLocation,
    SetLanguagePreference,
)

from .embeddings import (
    StubFaceEmbedder,
    StubSceneEmbedder,
    StubVoiceEmbedder,
    best_match,
)
from .store import GraphStore


class GraphServiceNode(Node):
    def __init__(self):
        super().__init__('graph_service')

        self.declare_parameter('db_path', '')
        self.declare_parameter('voice_threshold', 0.65)
        self.declare_parameter('scene_threshold', 0.75)

        db_path = self.get_parameter('db_path').value or None
        self.store = GraphStore(db_path=db_path)
        self.voice_embedder = StubVoiceEmbedder()
        self.scene_embedder = StubSceneEmbedder()
        self.face_embedder = StubFaceEmbedder()
        self.voice_threshold = float(self.get_parameter('voice_threshold').value)
        self.scene_threshold = float(self.get_parameter('scene_threshold').value)

        # Identity services
        self.create_service(IdentifyVoice,  '/identity/identify_voice', self.identify_voice)
        self.create_service(RegisterPerson, '/identity/register_person', self.register_person)
        self.create_service(RenamePerson,   '/identity/rename_person',   self.rename_person)
        self.create_service(ListPersons,    '/identity/list_persons',    self.list_persons)
        self.create_service(ForgetPerson,   '/identity/forget_person',   self.forget_person)

        # Location services
        self.create_service(IdentifyLocation, '/location/identify',     self.identify_location)
        self.create_service(LearnLocation,    '/location/learn',        self.learn_location)
        self.create_service(ListLocations,    '/location/list',         self.list_locations)
        self.create_service(SetCurrentLocation, '/location/set_current', self.set_current_location)
        self.create_service(RelateLocations,  '/location/relate',       self.relate_locations)

        # Memory services
        self.create_service(Remember,    '/memory/remember',     self.remember)
        self.create_service(Recall,      '/memory/recall',       self.recall)
        self.create_service(Relate,      '/memory/relate',       self.relate)
        self.create_service(FindRelated, '/memory/find_related', self.find_related)
        self.create_service(CypherQuery, '/graph/cypher',        self.cypher)

        # Preference services
        self.create_service(SetLanguagePreference,
                             '/language/set_preference',
                             self.set_language_preference)

        # Outbound preference broadcast (LanguageResolver consumes this)
        self._pref_pub = self.create_publisher(
            String, '/language/preference_update', 10)

        # Track most recent voice + scene embeddings (for register/learn without sample)
        self._last_voice_emb: Optional[np.ndarray] = None
        self._last_scene_emb: Optional[np.ndarray] = None
        self._current_location_id: Optional[str] = None

        self.get_logger().info('graph_service ready')

    # ---- identity ----

    def identify_voice(self, request, response):
        pcm = bytes(request.audio_pcm_int16)
        emb = self.voice_embedder.embed(pcm, request.sample_rate or 16000)
        self._last_voice_emb = emb
        cands = self.store.all_voice_embeddings()
        pid, score = best_match(emb, cands, self.voice_threshold)
        ident = PersonIdentity()
        ident.voice_confidence = float(score)
        ident.fused_confidence = float(score)
        ident.stamp = self.get_clock().now().to_msg()
        if pid is None:
            # Auto-register unknown
            new_pid = self.store.upsert_person(None,
                primary_name=f"Unknown ({self._short_now()})")
            self.store.add_voice_sample(new_pid, emb, self.voice_embedder.name)
            ident.person_id = new_pid
            ident.primary_name = f"Unknown ({self._short_now()})"
            ident.is_new = True
        else:
            ident.person_id = pid
            ident.primary_name = self._person_name(pid)
            ident.is_new = False
            self.store.add_voice_sample(pid, emb, self.voice_embedder.name)
            # Push their language preference to the resolver
            self._broadcast_person_pref_if_set(pid)
        response.identity = ident
        response.success = True
        response.message = 'ok'
        return response

    def register_person(self, request, response):
        # Use provided audio, or fall back to most recent voice embedding
        if request.audio_pcm_int16:
            emb = self.voice_embedder.embed(bytes(request.audio_pcm_int16),
                                             request.sample_rate or 16000)
        else:
            emb = self._last_voice_emb
        if emb is None:
            response.success = False
            response.message = 'no audio provided and no recent voice sample'
            return response
        # Match first — if known, just add alias
        cands = self.store.all_voice_embeddings()
        pid, score = best_match(emb, cands, self.voice_threshold)
        if pid is None:
            pid = self.store.upsert_person(None, primary_name=request.name)
        else:
            self.store.rename_person(pid, request.name)
        self.store.add_voice_sample(pid, emb, self.voice_embedder.name)
        ident = PersonIdentity()
        ident.person_id = pid
        ident.primary_name = request.name
        ident.voice_confidence = float(score if score else 1.0)
        ident.fused_confidence = float(score if score else 1.0)
        ident.is_new = (score < self.voice_threshold)
        ident.stamp = self.get_clock().now().to_msg()
        response.identity = ident
        response.success = True
        response.message = 'ok'
        return response

    def rename_person(self, request, response):
        self.store.rename_person(request.person_id, request.new_primary_name)
        for alias in request.add_aliases:
            self.store.add_alias(request.person_id, alias)
        response.success = True
        response.message = 'ok'
        return response

    def list_persons(self, request, response):
        out = []
        for p in self.store.list_persons():
            ident = PersonIdentity()
            ident.person_id = p['id']
            ident.primary_name = p['name'] or ''
            out.append(ident)
        response.persons = out
        return response

    def forget_person(self, request, response):
        self.store.forget_person(request.person_id)
        response.success = True
        response.message = 'forgotten'
        return response

    # ---- location ----

    def identify_location(self, request, response):
        jpeg = bytes(request.image_jpeg)
        if not jpeg:
            response.success = False
            response.message = 'no image; scene_recognizer should publish /perception/current_location instead'
            return response
        emb = self.scene_embedder.embed(jpeg)
        self._last_scene_emb = emb
        cands = self.store.all_location_embeddings()
        # cands is (id, name, emb); reshape for best_match
        as_pairs = [(c[0], c[2]) for c in cands]
        names = {c[0]: c[1] for c in cands}
        lid, score = best_match(emb, as_pairs, self.scene_threshold)
        ident = LocationIdentity()
        ident.confidence = float(score)
        ident.stamp = self.get_clock().now().to_msg()
        if lid is None:
            ident.is_unknown = True
        else:
            ident.location_id = lid
            ident.name = names.get(lid, '')
            ident.is_unknown = False
            self._current_location_id = lid
        response.identity = ident
        response.success = True
        response.message = 'ok'
        return response

    def learn_location(self, request, response):
        # Without provided frames, use the last seen scene embedding.
        # In production, scene_recognizer collects N frames and submits via a richer API.
        emb = self._last_scene_emb
        if emb is None:
            response.success = False
            response.message = 'no scene embedding available'
            return response
        lid = self.store.upsert_location(request.name,
                                          parent_name=request.parent_name)
        self.store.add_location_sample(lid, emb, self.scene_embedder.name)
        ident = LocationIdentity()
        ident.location_id = lid
        ident.name = request.name
        ident.parent_name = request.parent_name
        ident.confidence = 1.0
        ident.is_unknown = False
        ident.stamp = self.get_clock().now().to_msg()
        response.identity = ident
        response.success = True
        response.message = 'learned'
        self._current_location_id = lid
        return response

    def list_locations(self, request, response):
        out = []
        for l in self.store.list_locations():
            ident = LocationIdentity()
            ident.location_id = l['id']
            ident.name = l['name'] or ''
            ident.parent_name = l['parent'] or ''
            out.append(ident)
        response.locations = out
        return response

    def set_current_location(self, request, response):
        lid = self.store.upsert_location(request.name,
                                          parent_name=request.parent_name)
        self._current_location_id = lid
        ident = LocationIdentity()
        ident.location_id = lid
        ident.name = request.name
        ident.parent_name = request.parent_name
        ident.confidence = 1.0
        ident.stamp = self.get_clock().now().to_msg()
        response.identity = ident
        response.success = True
        response.message = 'ok'
        return response

    def relate_locations(self, request, response):
        a = self.store.upsert_location(request.location_a_name)
        b = self.store.upsert_location(request.location_b_name)
        rel = request.relation.upper()
        # Only allow known location rels; otherwise no-op
        if rel not in {"PART_OF", "NEAR", "CONNECTS_TO"}:
            response.success = False
            response.message = f'unknown relation: {request.relation}'
            return response
        self.store.execute(
            f"MATCH (a:Location {{id:$a}}), (b:Location {{id:$b}}) "
            f"MERGE (a)-[:{rel}]->(b)",
            {"a": a, "b": b},
        )
        response.success = True
        response.message = 'ok'
        return response

    # ---- memory ----

    def remember(self, request, response):
        about_person = None
        about_location = None
        if request.subject_type == 'Person':
            about_person = request.subject_id
        elif request.subject_type == 'Location':
            about_location = request.subject_id
        fid = self.store.create_fact(
            content=request.content,
            confidence=request.confidence or 1.0,
            source=request.source or 'manual',
            tags=request.tags or '',
            about_person_id=about_person,
            about_location_id=about_location,
        )
        response.fact_id = fid
        response.success = True
        response.message = 'remembered'
        return response

    def recall(self, request, response):
        if request.subject_type == 'Person':
            facts = self.store.recall_facts_about_person(
                request.subject_id, limit=request.limit or 10
            )
        else:
            facts = []
        response.fact_ids = [f['id'] for f in facts]
        response.contents = [f['content'] for f in facts]
        response.scores = [float(f.get('confidence', 1.0)) for f in facts]
        response.success = True
        response.message = 'ok'
        return response

    def relate(self, request, response):
        self.store.relate(
            request.subject_a_id, request.subject_b_id,
            request.relation,
            description=request.description,
            bidirectional=request.bidirectional,
        )
        response.success = True
        response.message = 'ok'
        return response

    def find_related(self, request, response):
        related = self.store.find_related_persons(
            request.subject_id,
            relation=request.relation,
            hops=request.hops or 1,
        )
        response.related_ids = [r['id'] for r in related]
        response.related_names = [r['name'] or '' for r in related]
        response.relation_types = []  # TODO: surface path types
        response.success = True
        response.message = 'ok'
        return response

    def cypher(self, request, response):
        try:
            params = json.loads(request.params_json) if request.params_json else {}
        except json.JSONDecodeError:
            params = {}
        try:
            res = self.store.execute(request.cypher, params)
            rows = []
            while res.has_next():
                rows.append([str(x) for x in res.get_next()])
            response.result_json = json.dumps(rows)
            response.success = True
            response.message = 'ok'
        except Exception as e:
            response.result_json = '[]'
            response.success = False
            response.message = str(e)
        return response

    # ---- language preferences ----

    def set_language_preference(self, request, response):
        scope = request.scope
        scope_id = request.scope_id
        lang = request.language  # "" clears

        if scope not in ('default', 'person', 'location', 'event'):
            response.success = False
            response.message = f'unknown scope: {scope}'
            return response
        if scope != 'default' and not scope_id:
            response.success = False
            response.message = f'scope_id required for scope={scope}'
            return response

        try:
            if scope == 'default':
                self.store.set_self_default_language(lang or None)
            elif scope == 'person':
                self.store.set_person_language(scope_id, lang or None)
            elif scope == 'location':
                self.store.set_location_language(scope_id, lang or None)
            elif scope == 'event':
                self.store.set_event_language(scope_id, lang or None)
        except Exception as e:
            response.success = False
            response.message = f'store error: {e}'
            return response

        # Broadcast to LanguageResolver
        self._broadcast_preference(scope, scope_id, lang or None)

        response.success = True
        response.message = 'ok'
        return response

    def _broadcast_preference(self, scope: str, scope_id: str,
                                language: Optional[str]):
        payload = {
            'scope': scope,
            'id': scope_id,
            'language': language,
        }
        m = String()
        m.data = json.dumps(payload, ensure_ascii=False)
        self._pref_pub.publish(m)

    def _broadcast_person_pref_if_set(self, person_id: str):
        """Called when a person is identified — push their stored preference (if any)
        so the LanguageResolver can apply it without a separate lookup."""
        lang = self.store.get_person_language(person_id)
        if lang:
            self._broadcast_preference('person', person_id, lang)

    # ---- helpers ----

    def _person_name(self, pid: str) -> str:
        res = self.store.execute(
            "MATCH (p:Person {id:$id}) RETURN p.primary_name", {"id": pid}
        )
        while res.has_next():
            return res.get_next()[0] or ''
        return ''

    def _short_now(self) -> str:
        import datetime as _dt
        return _dt.datetime.now().strftime('%Y%m%d-%H%M')


def main(args=None):
    rclpy.init(args=args)
    node = GraphServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
