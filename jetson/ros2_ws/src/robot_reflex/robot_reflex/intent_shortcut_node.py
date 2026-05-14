"""Intent shortcut node — bypass LLM for high-confidence pattern matches.

Bilingual (RO + EN). Watches /perception/transcript_partial and final for
canonical patterns with direct action mappings. On match, publishes a
ready-to-execute action on /agent/intent_shortcut.

Includes verbal language-switch patterns ("switch to English", "vorbește în
română") which the LanguageResolver consumes to update the active language.

Action types:
    stop           — quiet/halt current activity
    greet          — return greeting
    acknowledge    — "you're welcome"
    tell_time      — speak current time
    tell_date      — speak current date
    sleep          — go to standby
    wake           — wake from standby
    lang_switch    — verbal request to switch language (payload includes target)
"""

import json
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robot_perception_msgs.msg import Transcript


# Pattern table: (regex, action, confidence, language hint).
# Language hint is informational; matching is case-insensitive across both.
PATTERNS: list[tuple[re.Pattern, str, float, dict]] = [
    # --- stop / silence ---
    (re.compile(r'\b(stop|quiet|shut up|be quiet|hush)\b', re.I),
     'stop', 0.95, {'lang': 'en'}),
    (re.compile(r'\b(stai|oprește(?:-te)?|opreste(?:-te)?|liniște|liniste|taci|gata)\b', re.I),
     'stop', 0.95, {'lang': 'ro'}),

    # --- greeting ---
    (re.compile(r'\b(hello|hi|hey)\b', re.I),
     'greet', 0.85, {'lang': 'en'}),
    (re.compile(r'\b(salut|bună|buna|servus|noroc)\b', re.I),
     'greet', 0.85, {'lang': 'ro'}),

    # --- thanks ---
    (re.compile(r'\b(thanks|thank you)\b', re.I),
     'acknowledge', 0.90, {'lang': 'en'}),
    (re.compile(r'\b(mulțumesc|multumesc|mersi|mulțam|multam)\b', re.I),
     'acknowledge', 0.90, {'lang': 'ro'}),

    # --- time / date ---
    (re.compile(r"\bwhat (time is it|'s the time)\b", re.I),
     'tell_time', 0.95, {'lang': 'en'}),
    (re.compile(r'\b(cât e ceasul|cat e ceasul|ce or[aă] e)\b', re.I),
     'tell_time', 0.95, {'lang': 'ro'}),
    (re.compile(r"\bwhat (day is it|'s today|date is it)\b", re.I),
     'tell_date', 0.95, {'lang': 'en'}),
    (re.compile(r'\b(ce zi e|ce dat[aă] e|în ce dat[aă] suntem)\b', re.I),
     'tell_date', 0.95, {'lang': 'ro'}),

    # --- sleep / wake ---
    (re.compile(r'\b(go to sleep|sleep now|good night)\b', re.I),
     'sleep', 0.90, {'lang': 'en'}),
    (re.compile(r'\b(culc[aă]-te|noapte bun[aă]|du-te la culcare)\b', re.I),
     'sleep', 0.90, {'lang': 'ro'}),
    (re.compile(r'\b(wake up|wake)\b', re.I),
     'wake', 0.85, {'lang': 'en'}),
    (re.compile(r'\b(trezește-te|trezeste-te|hai trezirea|deșteptarea)\b', re.I),
     'wake', 0.85, {'lang': 'ro'}),

    # --- verbal language switch (persistent override) ---
    (re.compile(r"\b(switch|change) to (english|english language)\b", re.I),
     'lang_switch', 0.95, {'target': 'en'}),
    (re.compile(r"\b(let's |lets )?(speak|talk) in english\b", re.I),
     'lang_switch', 0.92, {'target': 'en'}),
    (re.compile(r'\b(switch|change) to (romanian|romanian language)\b', re.I),
     'lang_switch', 0.95, {'target': 'ro'}),
    (re.compile(r"\b(let's |lets )?(speak|talk) in romanian\b", re.I),
     'lang_switch', 0.92, {'target': 'ro'}),
    (re.compile(r'\b(vorbește|vorbeste|hai s[aă] vorbim) în (român[aă]|romana)\b', re.I),
     'lang_switch', 0.95, {'target': 'ro'}),
    (re.compile(r'\b(vorbește|vorbeste|hai s[aă] vorbim) în englez[aă]\b', re.I),
     'lang_switch', 0.95, {'target': 'en'}),
    (re.compile(r'\b(treci|schimb[aă]) pe (român[aă]|romana)\b', re.I),
     'lang_switch', 0.93, {'target': 'ro'}),
    (re.compile(r'\b(treci|schimb[aă]) pe englez[aă]\b', re.I),
     'lang_switch', 0.93, {'target': 'en'}),
]


class IntentShortcut(Node):

    def __init__(self):
        super().__init__('intent_shortcut')
        self.declare_parameter('min_confidence', 0.85)
        self.declare_parameter('emit_on_partial', True)
        self._min_conf = float(self.get_parameter('min_confidence').value)
        self._emit_on_partial = bool(
            self.get_parameter('emit_on_partial').value)

        self.create_subscription(Transcript, '/perception/transcript_partial',
                                 self._on_partial, 20)
        self.create_subscription(Transcript, '/perception/transcript',
                                 self._on_final, 10)
        self._pub = self.create_publisher(
            String, '/agent/intent_shortcut', 10)

        self._last_fired_action: str | None = None
        self._last_fired_text: str = ''

        self.get_logger().info(
            f'intent_shortcut ready (bilingual, min_conf={self._min_conf})')

    def _on_partial(self, msg: Transcript):
        if self._emit_on_partial:
            self._match_and_emit(msg.text, msg.language, source='partial')

    def _on_final(self, msg: Transcript):
        self._match_and_emit(msg.text, msg.language, source='final')
        self._last_fired_action = None
        self._last_fired_text = ''

    def _match_and_emit(self, text: str, detected_lang: str, source: str):
        if not text:
            return
        for pat, action, conf, meta in PATTERNS:
            if conf < self._min_conf:
                continue
            if not pat.search(text):
                continue
            # Deduplicate within a single utterance
            if (action == self._last_fired_action
                    and text.startswith(self._last_fired_text[:32])):
                return
            self._last_fired_action = action
            self._last_fired_text = text
            payload = {
                'action': action,
                'confidence': conf,
                'source': source,
                'text': text,
                'detected_language': detected_lang,
            }
            payload.update(meta)
            m = String()
            m.data = json.dumps(payload, ensure_ascii=False)
            self._pub.publish(m)
            self.get_logger().info(
                f'shortcut: {action} {meta} ({source}, conf={conf:.2f})')
            return


def main(args=None):
    rclpy.init(args=args)
    node = IntentShortcut()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
