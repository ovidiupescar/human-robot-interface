"""Audio player — streaming ring-buffer playback with barge-in.

Topics:
    /audio/stream        (UInt8MultiArray)  PCM chunks from streaming TTS
    /audio/playback      (UInt8MultiArray)  legacy: full PCM clips
    /control/interrupt   (Interrupt)       flush buffer immediately

Two playback paths:
  1. Streaming (preferred): chunks arrive on /audio/stream, written into a
     ring buffer, played continuously by the output stream callback.
  2. Legacy: full clip on /audio/playback enters the same ring.

Ring buffer drops oldest on overflow (TTS faster than playback rate) and
emits silence on underflow (TTS slower).
"""

import threading

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from robot_control_msgs.msg import Interrupt

try:
    import sounddevice as sd
except ImportError:
    sd = None


class AudioPlayer(Node):

    RING_SECONDS = 4.0  # max buffer depth

    def __init__(self):
        super().__init__('audio_player')
        self.declare_parameter('device', '')
        self.declare_parameter('sample_rate', 22050)
        self.sr = int(self.get_parameter('sample_rate').value)
        self.device = self.get_parameter('device').value or None

        self.create_subscription(UInt8MultiArray, '/audio/stream',
                                 self._on_stream_chunk, 100)
        self.create_subscription(UInt8MultiArray, '/audio/playback',
                                 self._on_full_clip, 10)
        self.create_subscription(Interrupt, '/control/interrupt',
                                 self._on_interrupt, 10)

        self._ring_size = int(self.sr * self.RING_SECONDS)
        self._ring = np.zeros(self._ring_size, dtype=np.int16)
        self._w = 0
        self._r = 0
        self._lock = threading.Lock()

        self._stream = None
        if sd is not None:
            self._stream = sd.OutputStream(
                samplerate=self.sr, channels=1, dtype='int16',
                device=self.device, blocksize=0,
                callback=self._cb,
            )
            self._stream.start()
            self.get_logger().info(f'audio_player v3 ready @ {self.sr}Hz '
                                    f'({self.RING_SECONDS}s ring)')
        else:
            self.get_logger().error('sounddevice not installed; playback disabled')

    # ---- inputs ----

    def _on_stream_chunk(self, msg: UInt8MultiArray):
        pcm = np.frombuffer(bytes(msg.data), dtype=np.int16)
        self._write_ring(pcm)

    def _on_full_clip(self, msg: UInt8MultiArray):
        pcm = np.frombuffer(bytes(msg.data), dtype=np.int16)
        self._write_ring(pcm)

    def _on_interrupt(self, msg: Interrupt):
        with self._lock:
            self._r = self._w  # flush
        self.get_logger().info('audio_player: ring flushed on interrupt')

    # ---- ring buffer ----

    def _write_ring(self, samples: np.ndarray):
        if len(samples) == 0:
            return
        with self._lock:
            n = len(samples)
            if self._w + n <= self._ring_size:
                self._ring[self._w:self._w + n] = samples
            else:
                first = self._ring_size - self._w
                self._ring[self._w:] = samples[:first]
                self._ring[:n - first] = samples[first:]
            self._w = (self._w + n) % self._ring_size
            # Overrun protection: if writer laps reader, drop oldest
            if self._available_locked() < 0:
                self._r = (self._w + 1) % self._ring_size

    def _cb(self, outdata, frames, time_info, status):
        with self._lock:
            avail = self._available_locked()
            if avail >= frames:
                self._read_ring_into(outdata[:, 0], frames)
            elif avail > 0:
                self._read_ring_into(outdata[:avail, 0], avail)
                outdata[avail:, 0] = 0  # silence pad on partial
            else:
                outdata[:, 0] = 0  # underrun = silence

    def _read_ring_into(self, out: np.ndarray, n: int):
        if self._r + n <= self._ring_size:
            out[:] = self._ring[self._r:self._r + n]
        else:
            first = self._ring_size - self._r
            out[:first] = self._ring[self._r:]
            out[first:] = self._ring[:n - first]
        self._r = (self._r + n) % self._ring_size

    def _available_locked(self) -> int:
        return (self._w - self._r) % self._ring_size

    def destroy_node(self):
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
