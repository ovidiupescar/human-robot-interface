"""Voice Activity Detection (VAD).

Subscribes:  /audio/chunk (std_msgs/ByteMultiArray) — raw PCM int16 mono
             /audio/level (std_msgs/Float32)        — RMS level
Publishes:   /perception/voice_active (std_msgs/Bool) — edge events on change

Simple energy-based VAD. Swap for silero-vad or webrtcvad for production.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, ByteMultiArray, Float32


class VoiceActivity(Node):
    def __init__(self):
        super().__init__('voice_activity')
        self.declare_parameter('threshold_rms', 0.03)
        self.declare_parameter('hangover_ms', 400)
        self.threshold = float(self.get_parameter('threshold_rms').value)
        self.hangover_ms = int(self.get_parameter('hangover_ms').value)

        self.create_subscription(Float32, '/audio/level', self._on_level, 20)
        self._active_pub = self.create_publisher(Bool, '/perception/voice_active', 10)

        self._is_active = False
        self._last_above_ms = 0
        self.get_logger().info(f'VAD threshold={self.threshold} hangover={self.hangover_ms}ms')

    def _on_level(self, msg: Float32):
        now = self.get_clock().now().nanoseconds // 1_000_000
        if msg.data > self.threshold:
            self._last_above_ms = now
            if not self._is_active:
                self._set_active(True)
        else:
            if self._is_active and (now - self._last_above_ms) > self.hangover_ms:
                self._set_active(False)

    def _set_active(self, val: bool):
        self._is_active = val
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
