"""Voice identifier — wraps audio buffering and calls /identity/identify_voice.

Subscribes:
    /audio/chunk             (ByteMultiArray)
    /perception/voice_active (Bool) — buffers during active speech
Publishes:
    /perception/identified_person (PersonIdentity) — at end of utterance
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, ByteMultiArray

from robot_graph_msgs.msg import PersonIdentity
from robot_graph_msgs.srv import IdentifyVoice


class VoiceIdentifier(Node):
    def __init__(self):
        super().__init__('voice_identifier')
        self.declare_parameter('sample_rate', 16000)
        self.sr = int(self.get_parameter('sample_rate').value)

        self.create_subscription(ByteMultiArray, '/audio/chunk', self._on_chunk, 100)
        self.create_subscription(Bool, '/perception/voice_active', self._on_voice, 10)
        self._person_pub = self.create_publisher(
            PersonIdentity, '/perception/identified_person', 10)

        self._cli = self.create_client(IdentifyVoice, '/identity/identify_voice')
        self._buf = bytearray()
        self._active = False
        self._lock = threading.Lock()

        self.get_logger().info('voice_identifier ready')

    def _on_chunk(self, msg):
        if not self._active:
            return
        with self._lock:
            self._buf.extend(bytes(msg.data))

    def _on_voice(self, msg: Bool):
        if msg.data and not self._active:
            with self._lock:
                self._buf = bytearray()
                self._active = True
        elif not msg.data and self._active:
            with self._lock:
                self._active = False
                pcm = bytes(self._buf)
                self._buf = bytearray()
            if pcm:
                threading.Thread(target=self._identify, args=(pcm,), daemon=True).start()

    def _identify(self, pcm: bytes):
        if not self._cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning('/identity/identify_voice unavailable')
            return
        req = IdentifyVoice.Request()
        req.audio_pcm_int16 = list(pcm)
        req.sample_rate = self.sr
        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        if resp is None or not resp.success:
            return
        self._person_pub.publish(resp.identity)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceIdentifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
