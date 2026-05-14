"""Language Resolver — single source of truth for current spoken language.

Resolution stack (highest priority first):
    1. Verbal override         — explicit "switch to X" via /agent/intent_shortcut
                                  (action=lang_switch). Persistent across utterances
                                  until another verbal override or context change.
    2. Active event override   — :Event.language_override property when an event is
                                  active. Auto-expires when the event ends.
    3. Active location override— :Location.language_override property for the current
                                  location. Auto-changes when location changes.
    4. Known speaker pref      — :Person.preferred_language for the currently
                                  identified speaker.
    5. Detected language       — most-recent /perception/transcript.language above
                                  confidence floor.
    6. Default                 — default_language parameter (robot's global default).

Publishes /language/current (LanguagePreference) latched whenever the resolved
value changes. Other nodes (tts_service, agent prompt builder) subscribe and use
it as the authoritative current language.

Notes:
    - Verbal override clears on explicit "go back to default" / "switch back" or
      when persistence_seconds elapses (configurable; default infinite within session).
    - Speaker preferences are only applied above identity_confidence_floor.
    - This node owns the resolution logic; storage of preferences lives in
      :Person / :Location / :Event nodes via the graph layer (writes happen
      through robot_graph_service, not here).
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robot_graph_msgs.msg import (
    IdentifiedSpeech,
    LocationIdentity,
    PersonIdentity,
)
from robot_perception_msgs.msg import LanguagePreference, Transcript


class LanguageResolver(Node):

    def __init__(self):
        super().__init__('language_resolver')
        self.declare_parameter('default_language', 'ro')
        self.declare_parameter('allowed_languages', ['ro', 'en'])
        self.declare_parameter('identity_confidence_floor', 0.6)
        self.declare_parameter('detection_confidence_floor', 0.7)

        self._default = self.get_parameter('default_language').value
        self._allowed = list(self.get_parameter('allowed_languages').value)
        self._id_floor = float(
            self.get_parameter('identity_confidence_floor').value)
        self._det_floor = float(
            self.get_parameter('detection_confidence_floor').value)

        # Tier state
        self._verbal_override: str | None = None
        self._event_override: tuple[str, str] | None = None      # (event_id, lang)
        self._location_override: tuple[str, str] | None = None   # (loc_id, lang)
        self._speaker_pref: tuple[str, str] | None = None        # (person_id, lang)
        self._detected_lang: str | None = None

        self._current_resolved: str | None = None

        # Subscriptions
        self.create_subscription(String, '/agent/intent_shortcut',
                                 self._on_shortcut, 20)
        self.create_subscription(Transcript, '/perception/transcript',
                                 self._on_transcript, 10)
        self.create_subscription(IdentifiedSpeech,
                                 '/perception/identified_speech',
                                 self._on_identified_speech, 10)
        self.create_subscription(PersonIdentity,
                                 '/perception/identified_person',
                                 self._on_identified_person, 10)
        self.create_subscription(LocationIdentity,
                                 '/perception/current_location',
                                 self._on_location, 10)

        # Hooks for graph_service to push known prefs (optional integration)
        self.create_subscription(String, '/language/preference_update',
                                 self._on_preference_update, 10)

        # Publisher (latched-like: republish on change)
        self._pub = self.create_publisher(
            LanguagePreference, '/language/current', 10)

        # Emit default at startup
        self._resolve_and_publish()

        self.get_logger().info(
            f'language_resolver ready (default={self._default}, '
            f'allowed={self._allowed})')

    # ---- inputs ----

    def _on_shortcut(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if payload.get('action') != 'lang_switch':
            return
        target = payload.get('target')
        if target in self._allowed:
            self._verbal_override = target
            self.get_logger().info(f'verbal override -> {target}')
            self._resolve_and_publish()

    def _on_transcript(self, msg: Transcript):
        if msg.language and msg.language_confidence >= self._det_floor:
            if msg.language in self._allowed:
                self._detected_lang = msg.language
                self._resolve_and_publish()

    def _on_identified_speech(self, msg: IdentifiedSpeech):
        # Convenience: same handling as identified_person
        if msg.speaker.fused_confidence >= self._id_floor:
            self._handle_speaker_change(msg.speaker)

    def _on_identified_person(self, msg: PersonIdentity):
        if msg.fused_confidence >= self._id_floor:
            self._handle_speaker_change(msg)

    def _handle_speaker_change(self, person: PersonIdentity):
        # NOTE: actual preferred_language fetch happens via graph_service.
        # For now, accept it via /language/preference_update from graph layer.
        # We just remember which person is current; if a pref was pushed for
        # that person, it stays applied.
        if self._speaker_pref and self._speaker_pref[0] != person.person_id:
            self._speaker_pref = None
            self._resolve_and_publish()

    def _on_location(self, msg: LocationIdentity):
        # Drop location override if location changed
        if (self._location_override
                and self._location_override[0] != msg.location_id):
            self._location_override = None
            self._resolve_and_publish()

    def _on_preference_update(self, msg: String):
        """Generic in-bound preference channel from graph_service.

        Expected JSON:
            { "scope": "person"|"location"|"event"|"default",
              "id": "<scope id>",
              "language": "ro"|"en"|null }
        Setting language=null clears the scope.
        """
        try:
            p = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        scope = p.get('scope')
        lang = p.get('language')
        scope_id = p.get('id', '')

        if lang is not None and lang not in self._allowed:
            return

        if scope == 'default':
            if lang in self._allowed:
                self._default = lang
        elif scope == 'person':
            self._speaker_pref = (scope_id, lang) if lang else None
        elif scope == 'location':
            self._location_override = (scope_id, lang) if lang else None
        elif scope == 'event':
            self._event_override = (scope_id, lang) if lang else None
        else:
            return

        self.get_logger().info(
            f'pref update: scope={scope} id={scope_id} lang={lang}')
        self._resolve_and_publish()

    # ---- resolution ----

    def _resolve(self) -> tuple[str, str, str]:
        """Return (language, source, scope_id)."""
        if self._verbal_override:
            return self._verbal_override, 'verbal_override', ''
        if self._event_override:
            return self._event_override[1], 'event', self._event_override[0]
        if self._location_override:
            return (self._location_override[1], 'location',
                    self._location_override[0])
        if self._speaker_pref:
            return self._speaker_pref[1], 'person', self._speaker_pref[0]
        if self._detected_lang:
            return self._detected_lang, 'detected', ''
        return self._default, 'default', ''

    def _resolve_and_publish(self):
        lang, source, scope_id = self._resolve()
        if lang == self._current_resolved:
            return
        self._current_resolved = lang
        m = LanguagePreference()
        m.language = lang
        m.source = source
        m.scope_id = scope_id
        m.stamp = self.get_clock().now().to_msg()
        self._pub.publish(m)
        self.get_logger().info(
            f'language now: {lang} (source={source}, scope={scope_id})')


def main(args=None):
    rclpy.init(args=args)
    node = LanguageResolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
