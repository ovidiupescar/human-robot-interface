"""Audio capture node.

Publishes:  /audio/chunk (std_msgs/UInt8MultiArray) — raw PCM frames
            /audio/level (std_msgs/Float32)        — RMS level for VAD/UI

TODO: replace dummy sounddevice usage with proper audio_common_msgs/AudioData.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32

try:
    import sounddevice as sd
except ImportError:
    sd = None


class AudioCapture(Node):
    def __init__(self):
        super().__init__('audio_capture')
        # See audio_player_node for the rationale: 'default' goes through
        # ALSA's plug plugin which resamples to the device's native rate.
        # Opening hw:* directly fails with -9997 on USB devices.
        self.declare_parameter('device', 'default')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_ms', 20)

        self.sr = int(self.get_parameter('sample_rate').value)
        chunk_ms = int(self.get_parameter('chunk_ms').value)
        self.chunk_samples = int(self.sr * chunk_ms / 1000)
        device = self.get_parameter('device').value or 'default'

        self._chunk_pub = self.create_publisher(UInt8MultiArray, '/audio/chunk', 50)
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

        # Publish raw PCM via UInt8MultiArray; data accepts array.array('B', ...)
        # directly. Subscribers reconstruct with bytes(msg.data).
        from array import array
        msg = UInt8MultiArray()
        msg.data = array('B', indata.tobytes())
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
