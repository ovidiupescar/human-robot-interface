"""Vision Capture — grab a single frame at the start of an utterance.

When VAD goes active, capture one JPEG frame from the camera stream and stage it
on /vision/frame_at_utterance for the orchestrator to attach to the next LLM
call (Gemma 4 multimodal). One frame is enough — Gemma describes what's there.

Rationale (from robo-brain v0.2):
    Streaming continuous frames into the LLM is wasteful. The interesting moment
    is "what was visible when the user started speaking" — that's the context
    that grounds the response. One JPEG per utterance, dropped if not consumed
    before the next utterance starts.

Subscribes:
    /camera/image_jpeg          (UInt8MultiArray)  source frames at camera rate
    /perception/voice_active    (Bool)            VAD onset triggers capture

Publishes:
    /vision/frame_at_utterance  (UInt8MultiArray)  the captured JPEG (latched-ish:
                                  republished if not consumed within TTL)

Notes:
    - Holds only the most-recent JPEG in memory (single slot)
    - Throttle: at most one capture per MIN_INTERVAL_S
    - Capture is fire-and-forget; no service call, no LLM round-trip here
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8MultiArray


class VisionCapture(Node):

    MIN_INTERVAL_S = 0.5
    FRAME_TTL_S = 30.0   # discard frames older than this

    def __init__(self):
        super().__init__('vision_capture')
        self.declare_parameter('publish_on_capture', True)

        self.create_subscription(UInt8MultiArray, '/camera/image_jpeg',
                                 self._on_frame, 5)
        self.create_subscription(Bool, '/perception/voice_active',
                                 self._on_voice, 10)
        self._pub = self.create_publisher(
            UInt8MultiArray, '/vision/frame_at_utterance', 5)

        self._latest_frame: bytes | None = None
        self._latest_frame_t = 0.0
        self._last_capture_t = 0.0
        self._lock = threading.Lock()

        self.get_logger().info('vision_capture ready')

    def _on_frame(self, msg: UInt8MultiArray):
        with self._lock:
            self._latest_frame = bytes(msg.data)
            self._latest_frame_t = time.time()

    def _on_voice(self, msg: Bool):
        if not msg.data:
            return
        now = time.time()
        with self._lock:
            if now - self._last_capture_t < self.MIN_INTERVAL_S:
                return
            frame = self._latest_frame
            frame_age = now - self._latest_frame_t
            self._last_capture_t = now

        if frame is None:
            self.get_logger().debug('VAD onset but no frame available')
            return
        if frame_age > self.FRAME_TTL_S:
            self.get_logger().debug(
                f'VAD onset but latest frame too old ({frame_age:.1f}s)')
            return

        from array import array
        out = UInt8MultiArray()
        out.data = array('B', frame)
        self._pub.publish(out)
        self.get_logger().info(
            f'frame captured ({len(frame)} bytes, age {frame_age*1000:.0f}ms)')


def main(args=None):
    rclpy.init(args=args)
    node = VisionCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
