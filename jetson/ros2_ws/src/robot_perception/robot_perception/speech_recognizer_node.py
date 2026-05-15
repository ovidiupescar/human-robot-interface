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

import re
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8MultiArray


# Hallucination filter — lifted from Hermes Agent's voice_mode.py.
# Whisper produces these strings deterministically from silence,
# breathing, or low-SNR audio. Keeping the set normalized lowercase
# without trailing punctuation; we strip both before lookup.
_WHISPER_HALLUCINATIONS = {
    "thank you", "thanks for watching", "subscribe to my channel",
    "like and subscribe", "please subscribe", "thank you for watching",
    "bye", "you", "the end",
    # Non-English hallucinations seen on long silence
    "продолжение следует", "sous-titres",
    "sous-titres réalisés par la communauté d'amara.org",
    "sottotitoli creati dalla comunità amara.org",
    "untertitel von stephanie geiges", "amara.org",
    "www.mooji.org", "ご視聴ありがとうございました",
}
_HALLUCINATION_REPEAT_RE = re.compile(
    r'^(?:thank you|thanks|bye|you|ok|okay|the end|\.|\s|,|!)+$',
    flags=re.IGNORECASE,
)


def _has_repeated_phrase(text: str, min_words: int = 3,
                          min_repeats: int = 2) -> bool:
    """Detect whisper's "I'm going to add a little bit of salt to it.
    I'm going to add a little bit of salt." failure mode.

    Whisper, given non-speech audio (TTS bleed, HVAC, ambient hum),
    will sometimes lock onto a phrase and emit it twice or three times.
    The earlier regex only matched short tokens; this catches the
    multi-word case. We strip punctuation, then check whether any
    contiguous window of `min_words` consecutive words appears at
    least `min_repeats` times in the transcript.
    """
    cleaned = re.sub(r'[^\w\s\']', ' ', text.lower())
    words = [w for w in cleaned.split() if w]
    if len(words) < min_words * min_repeats:
        return False
    seen: dict[str, int] = {}
    for i in range(len(words) - min_words + 1):
        gram = ' '.join(words[i:i + min_words])
        seen[gram] = seen.get(gram, 0) + 1
        if seen[gram] >= min_repeats:
            return True
    return False


def _is_whisper_hallucination(text: str) -> bool:
    cleaned = text.strip().lower().rstrip('.!?')
    if not cleaned:
        return True
    if cleaned in _WHISPER_HALLUCINATIONS:
        return True
    if _HALLUCINATION_REPEAT_RE.match(cleaned):
        return True
    if _has_repeated_phrase(text):
        return True
    return False

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
        # Defaults tuned for Jetson Orin Nano with CUDA-built ctranslate2
        # (see ops note: PyPI wheel is CPU-only on ARM64 Tegra and gives
        # ~20s per 1s of audio with the 'medium' model; the from-source
        # CUDA build drops that to under 1s). 'small' with int8_float16
        # is the sweet spot of accuracy vs latency on Orin.
        self.declare_parameter('model_size', 'small.en')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('compute_type', 'int8_float16')
        # English-only mode: Romanian integration deferred. small.en is
        # noticeably more accurate than the multilingual small and skips
        # the language-detection pass entirely.
        self.declare_parameter('allowed_languages', ['en'])
        self.declare_parameter('default_language', 'en')
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
        # ctranslate2 is NOT safe to call concurrently. Serialize ALL
        # whisper transcribe calls (final and partial) through one lock,
        # otherwise concurrent calls hang silently on CPU-only builds.
        self._whisper_lock = threading.Lock()

        self._partial_thread: threading.Thread | None = None
        self._partial_stop = threading.Event()
        self._last_partial_text = ''
        self._detected_lang = ''       # latest detected for this utterance
        self._detected_conf = 0.0

        # Partial decoding is expensive on Jetson CPU (medium int8 ~= 1.5s
        # per call). It also competes with final transcribe for the same
        # CPU, starving the final pass. Off by default; enable via param
        # when the model is small enough to keep up.
        self.declare_parameter('enable_partials', False)
        self._partials_enabled = bool(
            self.get_parameter('enable_partials').value)

        # Pre-roll buffer: keep the last N seconds of audio ALWAYS, so when
        # voice_active goes True we can prepend it to recover the leading
        # ~300ms that the 2-stage VAD swallows during speech confirmation.
        # Without this, an utterance like "what is the database" reaches
        # whisper as just "database" because the VAD took 300ms to confirm
        # speech and speech_recognizer only buffered from that point on.
        self.declare_parameter('preroll_seconds', 0.5)
        self._preroll_s = float(self.get_parameter('preroll_seconds').value)
        self._preroll_max_bytes = int(self._preroll_s * self.sr * 2)
        self._preroll: bytearray = bytearray()

        self._model = None
        if WhisperModel is not None:
            model_size = self.get_parameter('model_size').value
            device = self.get_parameter('device').value
            compute_type = self.get_parameter('compute_type').value
            try:
                self._model = WhisperModel(
                    model_size, device=device, compute_type=compute_type)
                self.get_logger().info(
                    f'whisper loaded: model={model_size} device={device} '
                    f'compute={compute_type} allowed={self.allowed}')
            except Exception as e:
                self.get_logger().warning(
                    f'whisper load failed on device={device}: {e}. '
                    'Falling back to CPU int8.')
                try:
                    # CUDA-built ctranslate2 only ships float32 for CPU.
                    self._model = WhisperModel(
                        model_size, device='cpu', compute_type='float32')
                    self.get_logger().info(
                        f'whisper loaded (CPU fallback): model={model_size}')
                except Exception as e2:
                    self.get_logger().error(
                        f'whisper load failed on CPU too: {e2}')
        else:
            self.get_logger().warning('faster-whisper not installed; STT disabled')

    # ---- audio ingestion ----

    def _on_chunk(self, msg: UInt8MultiArray):
        data = bytes(msg.data)
        with self._lock:
            if self._capturing:
                self._buf.extend(data)
            else:
                # Always maintain the pre-roll ring buffer. When the VAD
                # finally fires voice_active=True, _start_capture seeds
                # _buf with this so the first ~300ms of speech (lost to
                # VAD confirmation latency) is recovered.
                self._preroll.extend(data)
                if len(self._preroll) > self._preroll_max_bytes:
                    overflow = len(self._preroll) - self._preroll_max_bytes
                    del self._preroll[:overflow]

    def _on_voice(self, msg: Bool):
        if msg.data and not self._capturing:
            self._start_capture()
        elif not msg.data and self._capturing:
            self._stop_capture_and_finalize()

    def _start_capture(self):
        with self._lock:
            # Seed the recording buffer with the pre-roll so the first
            # ~300ms of speech that arrived before voice_active=True is
            # not lost. Clear the pre-roll itself — it gets refilled
            # while _capturing is False.
            self._buf = bytearray(self._preroll)
            self._preroll = bytearray()
            self._capturing = True
            self._last_partial_text = ''
            self._detected_lang = ''
            self._detected_conf = 0.0
        self._partial_stop.clear()
        if self._model is not None and self._partials_enabled:
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
                # Also dump the pre-roll: any TTS audio it contains
                # would otherwise be seeded into the next utterance.
                self._preroll = bytearray()
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
                with self._whisper_lock:
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

        Combines:
          - audio length floor (min_final_audio_s)
          - word count floor (min_final_words)
          - project-local blacklist (params)
          - Hermes voice_mode hallucination filter (curated phrase set
            + repetitive-pattern regex; see _is_whisper_hallucination)
        """
        normalized = text.strip().lower().rstrip('.!?,;: ')
        if not normalized:
            return True
        if audio_s < self.min_final_audio_s:
            return True
        if normalized in self.fragment_blacklist:
            return True
        if _is_whisper_hallucination(text):
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
            self.get_logger().info(
                f'whisper transcribe start [{audio_s:.2f}s]')
            with self._whisper_lock:
                # vad_filter: run Silero VAD first to drop silent regions
                #   so whisper never decodes them as hallucinated text.
                # no_repeat_ngram_size: forbid the decoder from emitting
                #   the same trigram twice in a row, which kills the
                #   "I'm going to add a little bit of salt. I'm going to
                #   add a little bit of salt." failure mode at the source.
                # hallucination_silence_threshold: when silence is detected
                #   between segments, skip it instead of letting whisper
                #   fill it with hallucinated text.
                # condition_on_previous_text: off so a noisy first pass
                #   doesn't poison subsequent segments in the same call.
                segments, info = self._model.transcribe(
                    audio, language=use_lang, beam_size=3,
                    vad_filter=True,
                    no_repeat_ngram_size=3,
                    hallucination_silence_threshold=0.5,
                    condition_on_previous_text=False)
                text = ' '.join(s.text.strip() for s in segments).strip()
            self.get_logger().info(
                f'whisper transcribe done [{audio_s:.2f}s]: {text!r}')
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
