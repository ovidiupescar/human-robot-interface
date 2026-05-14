"""Reflex node — fast reactions, conversation state machine, barge-in arbiter.

Bypasses Hermes (LLM is too slow for "respond instantly when someone starts
talking"). Hermes overrides reflex outputs whenever it issues a real command.

Responsibilities (v3):
  1. Map perception events to instant face state changes
  2. Hold authoritative conversation-state (IDLE/LISTENING/THINKING/SPEAKING/INTERRUPTED)
  3. Detect barge-in (voice_active during SPEAKING) → publish /control/interrupt
  4. Trigger optional verbal fillers ("hmm let me think...") to hide LLM latency

State transitions:
  IDLE      --voice_active=true  --> LISTENING
  LISTENING --voice_active=false --> THINKING (waits for Hermes response)
  THINKING  --tts started        --> SPEAKING
  SPEAKING  --voice_active=true  --> INTERRUPTED -> LISTENING
  SPEAKING  --tts done           --> IDLE
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from robot_face_msgs.msg import FaceCommand
from robot_control_msgs.msg import ConversationState, Interrupt


class Reflex(Node):

    # Tunables
    DEFAULT_ADDRESSEE_THRESHOLD = 0.4   # below = ignore (likely not addressed)
    FILLER_DELAY_S = 1.5                # how long thinking before filler plays
    FILLER_PHRASES = [
        "hmm let me think",
        "one moment",
        "give me a second",
    ]

    def __init__(self):
        super().__init__('reflex')

        self.declare_parameter('addressee_threshold',
                                self.DEFAULT_ADDRESSEE_THRESHOLD)
        self.declare_parameter('verbal_fillers_enabled', True)

        # Subscriptions
        self.create_subscription(Bool, '/perception/voice_active',
                                 self._on_voice, 10)
        self.create_subscription(Float32, '/perception/addressee_score',
                                 self._on_addressee, 10)
        self.create_subscription(String, '/audio/playback_status',
                                 self._on_playback, 10)

        # Publishers
        self._face_pub = self.create_publisher(FaceCommand, '/face/command', 10)
        self._interrupt_pub = self.create_publisher(Interrupt,
                                                     '/control/interrupt', 10)
        self._state_pub = self.create_publisher(ConversationState,
                                                 '/conversation/state', 10)
        self._filler_pub = self.create_publisher(String,
                                                  '/reflex/filler_request', 10)

        # State
        self._state = ConversationState.IDLE
        self._addressee = 0.5
        self._thinking_since_ms = 0
        self._filler_fired = False

        # Periodic tick for filler timer
        self.create_timer(0.2, self._tick)
        self._publish_state()

        self.get_logger().info('reflex_node v3 ready (state machine + barge-in)')

    # ---- callbacks ----

    def _on_addressee(self, msg: Float32):
        self._addressee = float(msg.data)

    def _on_voice(self, msg: Bool):
        threshold = float(self.get_parameter('addressee_threshold').value)
        addressed = self._addressee >= threshold

        if msg.data:  # voice started
            if not addressed and self._state != ConversationState.SPEAKING:
                # Side conversation (phone, other people) — ignore for face/state
                return

            if self._state == ConversationState.SPEAKING:
                # BARGE-IN
                self._fire_interrupt('user_voice')
                self._transition(ConversationState.INTERRUPTED)
                self._transition(ConversationState.LISTENING)
                self._set_face(FaceCommand.STATE_PROCESSING)
            elif self._state in (ConversationState.IDLE,
                                  ConversationState.THINKING):
                self._transition(ConversationState.LISTENING)
                self._set_face(FaceCommand.STATE_PROCESSING)

        else:  # voice ended
            if self._state == ConversationState.LISTENING:
                self._transition(ConversationState.THINKING)
                self._thinking_since_ms = self._now_ms()
                self._filler_fired = False

    def _on_playback(self, msg: String):
        if msg.data == 'started':
            self._transition(ConversationState.SPEAKING)
            self._set_face(FaceCommand.STATE_SPEAKING, 0.6)
        elif msg.data == 'done':
            if self._state == ConversationState.SPEAKING:
                self._transition(ConversationState.IDLE)
                self._set_face(FaceCommand.STATE_STANDBY)

    def _tick(self):
        # Verbal filler trigger: been thinking too long, fire a filler
        if (self._state == ConversationState.THINKING
                and not self._filler_fired
                and self.get_parameter('verbal_fillers_enabled').value
                and (self._now_ms() - self._thinking_since_ms)
                    >= int(self.FILLER_DELAY_S * 1000)):
            self._fire_filler()

    # ---- helpers ----

    def _transition(self, new_state: int):
        if new_state == self._state:
            return
        self._state = new_state
        self._publish_state()
        names = {0: 'IDLE', 1: 'LISTENING', 2: 'THINKING',
                 3: 'SPEAKING', 4: 'INTERRUPTED'}
        self.get_logger().debug(f'state -> {names.get(new_state, "?")}')

    def _publish_state(self):
        msg = ConversationState()
        msg.state = self._state
        msg.stamp = self.get_clock().now().to_msg()
        self._state_pub.publish(msg)

    def _set_face(self, state: int, amp: float = 0.0):
        cmd = FaceCommand()
        cmd.state = state
        cmd.amplitude = amp
        self._face_pub.publish(cmd)

    def _fire_interrupt(self, reason: str):
        msg = Interrupt()
        msg.reason = reason
        msg.stamp = self.get_clock().now().to_msg()
        msg.tts_active_text = ''
        msg.tts_byte_offset = 0
        self._interrupt_pub.publish(msg)
        self.get_logger().info(f'INTERRUPT: {reason}')

    def _fire_filler(self):
        self._filler_fired = True
        seed = (self._now_ms() // 1000) % len(self.FILLER_PHRASES)
        phrase = self.FILLER_PHRASES[seed]
        msg = String()
        msg.data = phrase
        self._filler_pub.publish(msg)
        self.get_logger().info(f'filler: {phrase}')

    def _now_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000


def main(args=None):
    rclpy.init(args=args)
    node = Reflex()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
