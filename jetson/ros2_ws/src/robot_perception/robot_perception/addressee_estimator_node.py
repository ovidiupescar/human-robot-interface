"""Addressee estimator — fuses signals into a single score in [0,1] of
"is the user talking to the robot right now?"

Subscribes:
  /perception/voice_active     (Bool)
  /perception/wake_word        (String — fired transiently when detected)
  /perception/transcript       (String — to apply direct-address heuristics)
  /audio/level                 (Float32 — proximity proxy)
  /perception/gaze_on_robot    (Bool — vision, future)
  /perception/phone_at_ear     (Bool — vision, future)
  /perception/other_voice      (Bool — diarization, future)
Publishes:
  /perception/addressee_score  (Float32, 0..1)
  /perception/addressee_hint   (String — short tag like "phone_call")
"""

import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


DIRECT_ADDRESS_RE = re.compile(
    r"\b(you|your|yourself|hey|hi|hello|please|tell me|what do you|can you|could you)\b",
    re.IGNORECASE,
)


class AddresseeEstimator(Node):
    def __init__(self):
        super().__init__('addressee_estimator')

        # Tunable weights
        self.declare_parameter('w_wake_word', 0.6)
        self.declare_parameter('w_gaze', 0.3)
        self.declare_parameter('w_address_lex', 0.1)
        self.declare_parameter('w_continuity', 0.2)
        self.declare_parameter('w_phone', -0.4)
        self.declare_parameter('w_other_voice', -0.3)
        self.declare_parameter('continuity_window_s', 30.0)
        self.declare_parameter('wake_word_validity_s', 8.0)

        # Inputs
        self.create_subscription(String, '/perception/wake_word', self._on_wake, 5)
        self.create_subscription(String, '/perception/transcript', self._on_text, 10)
        self.create_subscription(Bool,   '/perception/gaze_on_robot', self._on_gaze, 5)
        self.create_subscription(Bool,   '/perception/phone_at_ear', self._on_phone, 5)
        self.create_subscription(Bool,   '/perception/other_voice', self._on_other_voice, 5)
        self.create_subscription(String, '/agent/last_utterance', self._on_agent_utterance, 5)

        # Outputs
        self._score_pub = self.create_publisher(Float32, '/perception/addressee_score', 10)
        self._hint_pub = self.create_publisher(String, '/perception/addressee_hint', 10)

        # State
        self._wake_at = 0.0
        self._agent_spoke_at = 0.0
        self._gaze = False
        self._phone = False
        self._other_voice = False

        self.create_timer(0.2, self._tick)
        self.get_logger().info('addressee_estimator ready')

    def _on_wake(self, msg: String):
        self._wake_at = time.time()

    def _on_text(self, msg: String):
        # Compute and publish score keyed to this transcript event
        score, hint = self._compute(text=msg.data)
        self._publish(score, hint)

    def _on_gaze(self, msg: Bool):
        self._gaze = bool(msg.data)

    def _on_phone(self, msg: Bool):
        self._phone = bool(msg.data)

    def _on_other_voice(self, msg: Bool):
        self._other_voice = bool(msg.data)

    def _on_agent_utterance(self, msg: String):
        self._agent_spoke_at = time.time()

    def _tick(self):
        # Continuous low-rate update even without transcript event
        score, hint = self._compute(text='')
        self._publish(score, hint)

    def _compute(self, text: str):
        now = time.time()
        w_wake = float(self.get_parameter('w_wake_word').value)
        w_gaze = float(self.get_parameter('w_gaze').value)
        w_lex = float(self.get_parameter('w_address_lex').value)
        w_cont = float(self.get_parameter('w_continuity').value)
        w_phone = float(self.get_parameter('w_phone').value)
        w_other = float(self.get_parameter('w_other_voice').value)
        wake_valid = float(self.get_parameter('wake_word_validity_s').value)
        cont_win = float(self.get_parameter('continuity_window_s').value)

        x = 0.0
        hint = ''

        if (now - self._wake_at) < wake_valid:
            x += w_wake
        if self._gaze:
            x += w_gaze
        if (now - self._agent_spoke_at) < cont_win:
            x += w_cont
        if text and DIRECT_ADDRESS_RE.search(text):
            x += w_lex
        if self._phone:
            x += w_phone
            hint = 'phone_at_ear'
        if self._other_voice:
            x += w_other
            hint = (hint + ',' if hint else '') + 'other_voice'

        # Squash to (0,1)
        score = 1.0 / (1.0 + pow(2.718281828, -x))
        return score, hint

    def _publish(self, score: float, hint: str):
        s = Float32()
        s.data = float(score)
        self._score_pub.publish(s)
        if hint:
            h = String()
            h.data = hint
            self._hint_pub.publish(h)


def main(args=None):
    rclpy.init(args=args)
    node = AddresseeEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
