"""Wake-word detector.

Subscribes:  /audio/chunk (ByteMultiArray, int16 PCM)
Publishes:   /perception/wake_word (String — fires on detection)

Backend: openWakeWord (pip install openwakeword) — light, runs on CPU.
Default model: "hey jarvis" (placeholder). Swap for a custom-trained
"hey hermes" or robot's chosen name later.

Stub mode (when openwakeword not installed): always silent.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, String

try:
    from openwakeword.model import Model as WakeModel
except ImportError:
    WakeModel = None


class WakeWord(Node):
    def __init__(self):
        super().__init__('wake_word')
        self.declare_parameter('model_name', 'hey_jarvis_v0.1')   # placeholder
        self.declare_parameter('threshold', 0.6)
        self.declare_parameter('sample_rate', 16000)

        self.threshold = float(self.get_parameter('threshold').value)
        self.sr = int(self.get_parameter('sample_rate').value)

        self.create_subscription(ByteMultiArray, '/audio/chunk', self._on_chunk, 50)
        self._pub = self.create_publisher(String, '/perception/wake_word', 10)

        self._model = None
        if WakeModel is None:
            self.get_logger().warning(
                'openwakeword not installed — wake word detection disabled')
        else:
            try:
                self._model = WakeModel(
                    wakeword_models=[self.get_parameter('model_name').value],
                    inference_framework='onnx',
                )
                self.get_logger().info('wake word model loaded')
            except Exception as e:
                self.get_logger().error(f'wake model load failed: {e}')

    def _on_chunk(self, msg: ByteMultiArray):
        if self._model is None:
            return
        try:
            pcm = np.frombuffer(bytes(msg.data), dtype=np.int16)
            preds = self._model.predict(pcm)
            for name, score in preds.items():
                if score >= self.threshold:
                    out = String()
                    out.data = name
                    self._pub.publish(out)
                    self.get_logger().info(f'wake word: {name} ({score:.2f})')
        except Exception as e:
            self.get_logger().error(f'wake predict error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = WakeWord()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
