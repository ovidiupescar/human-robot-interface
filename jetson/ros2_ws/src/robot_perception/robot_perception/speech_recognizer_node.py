"""Speech-to-Text via faster-whisper — streaming partials + final, with language detection.

Subscribes:  /audio/chunk             (UInt8MultiArray)  PCM16 mono @ sample_rate
             /perception/voice_active (Bool)            gates capture window
Publishes:   /perception/transcript_partial (Transcript)   streaming partials
             /perception/transcript         (Transcript)   final on EOS

Language detection:
    - faster-whisper detects language per utterance when language=None
    - We sample detection on the first ~2s of audio (cheaper than re-detecting per partial)
    - Detected language attaches to every partial and final transcript message
    - If allowed_languages parameter set (e.g., ["ro", "en"]), other languages get clamped
      to the closest allowed one (Whisper sometimes guesses Italian for short Romanian)

The partial path uses greedy decoding (beam_size=1) for speed; the final pass
uses beam_size=3 for quality. Both publish typed Transcript messages.
"""

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8MultiArray

from robot_perception_msgs.msg import Transcript

try:
    from faster_whisper import WhisperModel
except ImportError:
    WhisperModel = None


class SpeechRecognizer(Node):

    PARTIAL_INTERVAL_S = 0.35
    PARTIAL_MIN_AUDIO_S = 0.5
    PARTIAL_MAX_WINDOW_S = 15.0
    LANG_DETECT_AUDIO_S = 2.0    # minimum audio before language detection is trusted

    def __init__(self):
        super().__init__('speech_recognizer')
        # Defaults tuned for Jetson Orin Nano: PyPI ctranslate2 wheels are
        # CPU-only on ARM64 Tegra, so device='cpu' avoids a CUDA-missing
        # crash. 'medium' int8 gives noticeably better Romanian accuracy
        # than 'small' at roughly real-time on Orin CPU. Override via
        # launch params if running on a host with GPU-enabled ctranslate2.
        self.declare_parameter('model_size', 'medium')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('compute_type', 'int8')
        self.declare_parameter('allowed_languages', ['ro', 'en'])
        self.declare_parameter('default_language', 'ro')
        self.declare_parameter('sample_rate', 16000)
        # Fragment filter: anything shorter than these gates does NOT
        # publish a final transcript. Stops "Oh", "Bye", "Mm-hmm",
        # whisper hallucinations on silence, and partial reverb-tail
        # captures from being treated as a user utterance.
        self.declare_parameter('min_final_words', 1)
        self.declare_parameter('min_final_audio_s', 0.4)
        self.declare_parameter(
            'fragment_blacklist',
            [
                # Common whisper hallucinations on silence/reverb.
                'thanks for watching',
                'thanks for watching.',
                'thank you',
                'thank you.',
                'thanks',
                'thanks.',
                'bye',
                'bye.',
                'oh',
                'oh.',
                'um',
                'uh',
                'mm',
                'mm-hmm',
                'mm-hmm.',
            ],
        )

        self.sr = int(self.get_parameter('sample_rate').value)
        self.allowed = list(self.get_parameter('allowed_languages').value)
        self.default_lang = self.get_parameter('default_language').value
        self.min_final_words = int(self.get_parameter('min_final_words').value)
        self.min_final_audio_s = float(
            self.get_parameter('min_final_audio_s').value)
        self.fragment_blacklist = {
            s.strip().lower()
            for s in self.get_parameter('fragment_blacklist').value
        }

        self.create_subscription(UInt8MultiArray, '/audio/chunk',
                                  self._on_chunk, 50)
        self.create_subscription(Bool, '/perception/voice_active',
                                  self._on_voice, 10)
        # Drop any in-flight buffer when TTS starts — otherwise a
        # half-captured user utterance gets concatenated with bot audio
        # leak-through and the final transcript reads as the bot's text.
        self.create_subscription(String, '/audio/playback_status',
                                  self._on_playback_status, 10)
        self._partial_pub = self.create_publisher(
            Transcript, '/perception/transcript_partial', 20)
        self._final_pub = self.create_publisher(
            Transcript, '/perception/transcript', 10)

        self._buf = bytearray()
        self._capturing = False
        self._lock = threading.Lock()

        self._partial_thread: threading.Thread | None = None
        self._partial_stop = threading.Event()
        self._last_partial_text = ''
        self._detected_lang = ''       # latest detected for this utterance
        self._detected_conf = 0.0

        self._model = None
        if WhisperModel is not None:
            try:
                self._model = WhisperModel(
                    self.get_parameter('model_size').value,
                    device=self.get_parameter('device').value,
                    compute_type=self.get_parameter('compute_type').value,
                )
                self.get_logger().info(
                    f'whisper loaded; allowed_languages={self.allowed}')
            except Exception as e:
                self.get_logger().error(f'whisper load failed: {e}')
        else:
            self.get_logger().warning('faster-whisper not installed; STT disabled')

    # ---- audio ingestion ----

    def _on_chunk(self, msg: UInt8MultiArray):
        if not self._capturing:
            return
        with self._lock:
            self._buf.extend(bytes(msg.data))

    def _on_voice(self, msg: Bool):
        if msg.data and not self._capturing:
            self._start_capture()
        elif not msg.data and self._capturing:
            self._stop_capture_and_finalize()

    def _start_capture(self):
        with self._lock:
            self._buf = bytearray()
            self._capturing = True
            self._last_partial_text = ''
            self._detected_lang = ''
            self._detected_conf = 0.0
        self._partial_stop.clear()
        if self._model is not None:
            self._partial_thread = threading.Thread(
                target=self._partial_loop, daemon=True)
            self._partial_thread.start()

    def _stop_capture_and_finalize(self):
        self._partial_stop.set()
        with self._lock:
            self._capturing = False
            pcm = bytes(self._buf)
            self._buf = bytearray()
        audio_s = len(pcm) / (2.0 * self.sr) if pcm else 0.0
        self.get_logger().info(
            f'finalize: buf={len(pcm)} bytes ({audio_s:.2f}s), '
            f'model={"loaded" if self._model else "None"}')
        if pcm and self._model is not None:
            threading.Thread(target=self._final_transcribe,
                             args=(pcm,), daemon=True).start()

    def _on_playback_status(self, msg: String):
        """When TTS starts, drop any in-flight capture and stop the partial
        worker. We do NOT finalize — anything we have so far is contaminated
        by speaker bleed."""
        if msg.data == 'started':
            self._partial_stop.set()
            with self._lock:
                self._capturing = False
                self._buf = bytearray()
                self._last_partial_text = ''
                self._detected_lang = ''
                self._detected_conf = 0.0

    # ---- language clamping ----

    def _clamp_language(self, lang: str) -> str:
        """Force detected language into allowed set, falling back to default."""
        if not self.allowed:
            return lang or self.default_lang
        if lang in self.allowed:
            return lang
        return self.default_lang

    # ---- streaming partials ----

    def _partial_loop(self):
        while not self._partial_stop.is_set():
            time.sleep(self.PARTIAL_INTERVAL_S)
            if self._partial_stop.is_set():
                break

            with self._lock:
                pcm = bytes(self._buf)

            audio_s = len(pcm) / (2.0 * self.sr)
            if audio_s < self.PARTIAL_MIN_AUDIO_S:
                continue

            max_bytes = int(self.PARTIAL_MAX_WINDOW_S * self.sr * 2)
            if len(pcm) > max_bytes:
                pcm = pcm[-max_bytes:]

            try:
                audio = (np.frombuffer(pcm, dtype=np.int16)
                         .astype(np.float32) / 32768.0)
                # Pass language=None for first detection, then lock in
                use_lang = self._detected_lang if self._detected_lang else None
                segments, info = self._model.transcribe(
                    audio, language=use_lang, beam_size=1,
                    condition_on_previous_text=False)
                text = ' '.join(s.text.strip() for s in segments).strip()
                # Update detected language if not locked yet and audio long enough
                if (not self._detected_lang
                        and audio_s >= self.LANG_DETECT_AUDIO_S
                        and hasattr(info, 'language')):
                    self._detected_lang = self._clamp_language(info.language)
                    self._detected_conf = float(
                        getattr(info, 'language_probability', 0.0))
                    self.get_logger().info(
                        f'detected language: {info.language} '
                        f'-> {self._detected_lang} '
                        f'(conf={self._detected_conf:.2f})')
            except Exception as e:
                self.get_logger().warning(f'partial transcribe error: {e}')
                continue

            if text and text != self._last_partial_text:
                self._last_partial_text = text
                self._publish_transcript(
                    self._partial_pub, text,
                    self._detected_lang or self.default_lang,
                    self._detected_conf, is_final=False)

    def _is_fragment(self, text: str, audio_s: float) -> bool:
        """Should this finalized transcript be dropped as noise?

        Reasons covered (lifted from Pipecat's 'Filter Incomplete User Turns'):
          - utterance audio shorter than min_final_audio_s
          - word count below min_final_words
          - text matches a known whisper-on-silence hallucination
            ('thanks for watching', 'bye', 'oh', …)
        """
        normalized = text.strip().lower().rstrip('.!?,;: ')
        if not normalized:
            return True
        if audio_s < self.min_final_audio_s:
            return True
        if normalized in self.fragment_blacklist:
            return True
        words = [w for w in normalized.split() if w]
        if len(words) < self.min_final_words:
            return True
        return False

    def _final_transcribe(self, pcm: bytes):
        try:
            audio = (np.frombuffer(pcm, dtype=np.int16)
                     .astype(np.float32) / 32768.0)
            audio_s = len(audio) / float(self.sr)
            use_lang = self._detected_lang if self._detected_lang else None
            segments, info = self._model.transcribe(
                audio, language=use_lang, beam_size=3)
            text = ' '.join(s.text.strip() for s in segments).strip()
            lang = (self._detected_lang
                    or self._clamp_language(getattr(info, 'language', '')))
            conf = (self._detected_conf
                    or float(getattr(info, 'language_probability', 0.0)))
            if not text:
                self.get_logger().info(
                    f'whisper empty result [{audio_s:.2f}s of audio]')
                return
            if self._is_fragment(text, audio_s):
                self.get_logger().info(
                    f'fragment dropped [{lang}, {audio_s:.2f}s]: {text!r}')
                return
            self._publish_transcript(
                self._final_pub, text, lang, conf, is_final=True)
            self.get_logger().info(f'final [{lang}]: {text}')
        except Exception as e:
            self.get_logger().error(f'final transcribe error: {e}')

    def _publish_transcript(self, pub, text: str, lang: str,
                             conf: float, is_final: bool):
        m = Transcript()
        m.text = text
        m.language = lang
        m.language_confidence = float(conf)
        m.is_final = is_final
        m.stamp = self.get_clock().now().to_msg()
        pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
