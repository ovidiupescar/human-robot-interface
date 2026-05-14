"""Audio capture node.

Publishes:  /audio/chunk (std_msgs/ByteMultiArray) — raw PCM frames
            /audio/level (std_msgs/Float32)        — RMS level for VAD/UI

TODO: replace dummy sounddevice usage with proper audio_common_msgs/AudioData.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, Float32

try:
    import sounddevice as sd
except ImportError:
    sd = None


class AudioCapture(Node):
    def __init__(self):
        super().__init__('audio_capture')
        self.declare_parameter('device', '')           # ALSA device name; '' = default
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_ms', 20)

        self.sr = int(self.get_parameter('sample_rate').value)
        chunk_ms = int(self.get_parameter('chunk_ms').value)
        self.chunk_samples = int(self.sr * chunk_ms / 1000)
        device = self.get_parameter('device').value or None

        self._chunk_pub = self.create_publisher(ByteMultiArray, '/audio/chunk', 50)
        self._level_pub = self.create_publisher(Float32, '/audio/level', 20)

        if sd is None:
            self.get_logger().error('sounddevice not installed: pip install sounddevice')
            return

        self._stream = sd.InputStream(
            samplerate=self.sr,
            channels=1,
            dtype='int16',
            blocksize=self.chunk_samples,
            device=device,
            callback=self._cb,
        )
        self._stream.start()
        self.get_logger().info(f'capturing @ {self.sr}Hz, {chunk_ms}ms chunks')

    def _cb(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warning(str(status))

        # Publish raw PCM. ROS2 ByteMultiArray.data requires a list/sequence
        # where each element is a `bytes` object of length 1 (rosidl octet
        # array binding). Build it once per chunk.
        raw = indata.tobytes()
        msg = ByteMultiArray()
        msg.data = [bytes((b,)) for b in raw]
        self._chunk_pub.publish(msg)

        # Publish level (RMS, 0..1)
        rms = float(np.sqrt(np.mean(indata.astype(np.float32) ** 2)) / 32768.0)
        lvl = Float32()
        lvl.data = rms
        self._level_pub.publish(lvl)


def main(args=None):
    rclpy.init(args=args)
    node = AudioCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
