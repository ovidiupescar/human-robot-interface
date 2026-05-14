"""Audio capture node.

Publishes:  /audio/chunk (std_msgs/UInt8MultiArray) — raw PCM frames
            /audio/level (std_msgs/Float32)        — RMS level for VAD/UI

Subscribes: /audio/playback_status (std_msgs/String)
            Suppresses chunk + level publication while TTS is speaking
            so the speaker's own output, bleeding back into the mic
            (Plantronics is full-duplex but has no hardware AEC exposed
            through the ALSA default device), doesn't get re-transcribed
            as user speech, voice_active-triggered, or worse — registered
            as a brand-new "Unknown" person every utterance. The duck-out
            is released a short hold-over after 'done'/'interrupted' to
            let the speaker's tail decay.
"""

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, UInt8MultiArray

try:
    import sounddevice as sd
except ImportError:
    sd = None


# Time to keep the mic muted after a TTS chunk ends. Covers speaker
# reverb tail + any final samples sitting in the playback ring buffer.
POST_TTS_MUTE_S = 0.6


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

        self._chunk_pub = self.create_publisher(UInt8MultiArray,
                                                  '/audio/chunk', 50)
        self._level_pub = self.create_publisher(Float32, '/audio/level', 20)

        # ---- mic-duck state (set from /audio/playback_status callbacks) ----
        self._mute_until = 0.0          # monotonic-time cutoff
        self._is_speaking = False
        self._lock = threading.Lock()
        self.create_subscription(String, '/audio/playback_status',
                                 self._on_playback_status, 10)

        if sd is None:
            self.get_logger().error(
                'sounddevice not installed: pip install sounddevice')
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
        self.get_logger().info(
            f'capturing @ {self.sr}Hz, {chunk_ms}ms chunks '
            f'(mic ducks during TTS + {POST_TTS_MUTE_S:.1f}s hold-over)')

    def _on_playback_status(self, msg: String) -> None:
        status = msg.data
        with self._lock:
            if status == 'started':
                self._is_speaking = True
            else:  # 'done' or 'interrupted'
                self._is_speaking = False
                self._mute_until = time.monotonic() + POST_TTS_MUTE_S

    def _muted(self) -> bool:
        with self._lock:
            if self._is_speaking:
                return True
            return time.monotonic() < self._mute_until

    def _cb(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warning(str(status))

        if self._muted():
            # Still publish a zero level so the VAD's `level > threshold`
            # state machine collapses immediately rather than coasting on
            # the last value across a TTS burst. Don't publish the chunk
            # itself — anything downstream that buffers PCM (whisper,
            # voice_identifier) should see silence during our turn.
            zero = Float32()
            zero.data = 0.0
            self._level_pub.publish(zero)
            return

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
