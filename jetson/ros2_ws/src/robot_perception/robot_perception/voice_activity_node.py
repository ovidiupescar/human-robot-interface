"""Voice Activity Detection (VAD) — 2-stage with dip tolerance.

Subscribes:  /audio/level (std_msgs/Float32)        — RMS level (0..1)
Publishes:   /perception/voice_active (std_msgs/Bool) — edge events on change

Algorithm (adapted from Hermes Agent's tools/voice_mode.py AudioRecorder):

  1. SPEECH CONFIRMATION
     A burst above `threshold_rms` does NOT immediately fire voice_active.
     We require `min_speech_duration_s` of sustained audio above threshold
     before declaring speech, tolerating dips up to `max_dip_tolerance_s`
     between syllables. This kills door-clicks, keyboard noise, single
     phonemes the VAD would otherwise treat as utterance starts.

  2. END DETECTION WITH RESUMPTION
     Once speech is confirmed, voice_active stays True until the user has
     been silent for `silence_duration_s`. Brief dips during continued
     speech (within `max_dip_tolerance_s`) do not start the silence
     timer. If audio rises above threshold for `min_speech_duration_s`
     during the silence window, the silence timer cancels — same logic as
     the initial confirmation, so a pause that turns into "...what was I
     saying again" stays one utterance, not two.

Why not the previous one-shot hangover: a 400ms hangover splits any
sentence with a comma-pause into multiple transcripts. Whisper then
transcribes each fragment independently and the agent gets a stream of
short messages, interrupting itself on every one.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class VoiceActivity(Node):
    def __init__(self):
        super().__init__('voice_activity')
        # Energy threshold on /audio/level (RMS 0..1)
        self.declare_parameter('threshold_rms', 0.03)
        # Sustained-energy window before declaring speech start
        self.declare_parameter('min_speech_duration_s', 0.3)
        # How long a sub-threshold dip can last before we treat it as
        # a real pause (used both during start-confirm and during resume
        # tracking inside the silence window).
        self.declare_parameter('max_dip_tolerance_s', 0.3)
        # Sustained silence (after speech confirmed) before voice_active
        # goes False.
        self.declare_parameter('silence_duration_s', 1.2)

        self.threshold = float(self.get_parameter('threshold_rms').value)
        self.min_speech_s = float(
            self.get_parameter('min_speech_duration_s').value)
        self.max_dip_s = float(
            self.get_parameter('max_dip_tolerance_s').value)
        self.silence_s = float(
            self.get_parameter('silence_duration_s').value)

        self.create_subscription(Float32, '/audio/level', self._on_level, 20)
        self._active_pub = self.create_publisher(
            Bool, '/perception/voice_active', 10)

        # State machine
        self._active = False           # current voice_active value
        self._speech_attempt_start = 0  # ms when first above-threshold seen
        self._dip_start = 0             # ms when current sub-threshold dip began
        self._silence_start = 0         # ms when post-speech silence began
        self._resume_start = 0          # ms when above-threshold seen during silence window
        self._resume_dip_start = 0      # ms tracker for resume-window dips

        self.get_logger().info(
            f'VAD threshold={self.threshold} '
            f'min_speech={self.min_speech_s*1000:.0f}ms '
            f'max_dip={self.max_dip_s*1000:.0f}ms '
            f'silence={self.silence_s*1000:.0f}ms')

    def _now_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000

    def _on_level(self, msg: Float32):
        now = self._now_ms()
        above = msg.data > self.threshold

        if not self._active:
            self._handle_inactive(above, now)
        else:
            self._handle_active(above, now)

    def _handle_inactive(self, above: bool, now: int):
        """Looking for a speech START."""
        if above:
            # Reset dip tracker — we're back above threshold
            self._dip_start = 0
            if self._speech_attempt_start == 0:
                self._speech_attempt_start = now
            elif (now - self._speech_attempt_start) >= int(self.min_speech_s * 1000):
                # Sustained energy — confirm speech
                self._set_active(True)
                self._speech_attempt_start = 0
                self._dip_start = 0
        else:
            # Below threshold during a speech attempt: tolerate briefly
            if self._speech_attempt_start > 0:
                if self._dip_start == 0:
                    self._dip_start = now
                elif (now - self._dip_start) >= int(self.max_dip_s * 1000):
                    # Dip lasted too long — abandon this attempt
                    self._speech_attempt_start = 0
                    self._dip_start = 0

    def _handle_active(self, above: bool, now: int):
        """Looking for a speech END (silence_duration of sustained silence)."""
        if above:
            # Audio above threshold while voice_active=True
            self._resume_dip_start = 0
            if self._silence_start > 0:
                # We were in a silence window; check if speech resumed
                if self._resume_start == 0:
                    self._resume_start = now
                elif (now - self._resume_start) >= int(self.min_speech_s * 1000):
                    # Sustained resumed speech — cancel the silence timer
                    self._silence_start = 0
                    self._resume_start = 0
        else:
            # Below threshold while voice_active=True
            if self._silence_start == 0:
                self._silence_start = now
                self._resume_start = 0
                self._resume_dip_start = 0
            else:
                # Dip during a resume attempt — tolerate briefly
                if self._resume_start > 0:
                    if self._resume_dip_start == 0:
                        self._resume_dip_start = now
                    elif (now - self._resume_dip_start) >= int(self.max_dip_s * 1000):
                        # Dip killed the resume — back to silence countdown
                        self._resume_start = 0
                        self._resume_dip_start = 0
                if (now - self._silence_start) >= int(self.silence_s * 1000):
                    # Sustained silence — end utterance
                    self._set_active(False)
                    self._silence_start = 0
                    self._resume_start = 0
                    self._resume_dip_start = 0

    def _set_active(self, val: bool):
        self._active = val
        m = Bool()
        m.data = val
        self._active_pub.publish(m)
        self.get_logger().info(f'voice_active={val}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceActivity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
