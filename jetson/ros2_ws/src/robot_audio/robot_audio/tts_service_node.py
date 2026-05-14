"""TTS service node — bilingual streaming with dual engines.

Engines:
    - Romanian: Piper (ro_RO-mihai-medium)
    - English:  Kokoro-82M (voice af_bella)

Topics:
    /tts/stream_in       (TtsRequest)     text chunks to speak (with language)
    /tts/say             (TtsRequest)     full sentence to speak
    /language/current    (LanguagePreference)  authoritative current language
    /control/interrupt   (Interrupt)      cancel synthesis immediately
    /audio/stream        (UInt8MultiArray) PCM chunks emitted to audio_player
    /audio/playback_status (String)       "started" | "done" | "interrupted"

Service:
    /speak (robot_face_msgs/Speak)        legacy sync speak (uses current language)

Language resolution per request:
    1. If TtsRequest.language is set explicitly, honor it
    2. Else use last value from /language/current
    3. Else fall back to default_language parameter

Voice resolution per request:
    1. If TtsRequest.voice is set, honor it (engine inferred from voice prefix)
    2. Else use the default voice for the resolved language

Streaming model:
    - Sentences arrive on /tts/stream_in or /tts/say
    - Engine synthesizes per-sentence, emitting ~50ms PCM chunks
    - On /control/interrupt: cancel immediately, drop buffered text
    - On language switch mid-buffer: drain current sentence in original language,
      then switch to new engine for next sentence (avoids cracking the voice)

Replace _synthesize_stream() with real Piper + Kokoro streaming when wiring.
"""

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String

from robot_control_msgs.msg import Interrupt
from robot_face_msgs.msg import FaceCommand
from robot_face_msgs.srv import Speak
from robot_perception_msgs.msg import LanguagePreference, TtsRequest

from robot_audio.engines import KokoroEngine, PiperEngine


class TtsService(Node):

    CHUNK_MS = 50

    # Default voice per language
    DEFAULT_VOICES = {
        'ro': 'ro_RO-mihai-medium',  # Piper
        'en': 'af_bella',             # Kokoro
    }

    def __init__(self):
        super().__init__('tts_service')
        self.declare_parameter('sample_rate', 22050)
        self.declare_parameter('drive_face', True)
        self.declare_parameter('default_language', 'ro')
        self.declare_parameter('piper_voice_dir', '/opt/piper/voices')
        self.declare_parameter('kokoro_model_path', '/opt/kokoro/kokoro-v1.0.onnx')

        self.sr = int(self.get_parameter('sample_rate').value)
        self.drive_face = bool(self.get_parameter('drive_face').value)
        self.default_lang = self.get_parameter('default_language').value
        self._current_language = self.default_lang

        # Engines emit at the pipeline rate (self.sr). Piper native is 22050,
        # Kokoro native is 24000 — each resamples as needed so the player and
        # /audio/stream subscribers see a single, consistent rate.
        self._piper = PiperEngine(
            voice_dir=self.get_parameter('piper_voice_dir').value,
            default_voice='ro_RO-mihai-medium',
            output_sample_rate=self.sr)
        self._kokoro = KokoroEngine(
            model_path=self.get_parameter('kokoro_model_path').value,
            default_voice='af_bella',
            output_sample_rate=self.sr)

        # Publishers
        self._stream_pub = self.create_publisher(UInt8MultiArray,
                                                  '/audio/stream', 10)
        self._status_pub = self.create_publisher(String,
                                                  '/audio/playback_status', 10)
        self._face_pub = self.create_publisher(FaceCommand, '/face/command', 10)

        # Subscriptions
        self.create_subscription(TtsRequest, '/tts/stream_in',
                                 self._on_text_chunk, 50)
        self.create_subscription(TtsRequest, '/tts/say', self._on_say, 10)
        self.create_subscription(Interrupt, '/control/interrupt',
                                 self._on_interrupt, 10)
        self.create_subscription(LanguagePreference, '/language/current',
                                 self._on_language_update, 10)

        # Service (legacy sync API)
        self.create_service(Speak, '/speak', self._on_speak)

        # State
        self._buffer: list[TtsRequest] = []
        self._buffer_lock = threading.Lock()
        self._cancel = threading.Event()
        self._synth_thread: threading.Thread | None = None
        self._is_speaking = False

        self.get_logger().info(
            f'tts_service ready (default_lang={self.default_lang}, '
            f'voices={self.DEFAULT_VOICES}, '
            f'piper_available={self._piper.available}, '
            f'kokoro_available={self._kokoro.available})')

    # ---- inputs ----

    def _on_text_chunk(self, msg: TtsRequest):
        with self._buffer_lock:
            self._buffer.append(msg)
        self._kick_synth()

    def _on_say(self, msg: TtsRequest):
        with self._buffer_lock:
            self._buffer.append(msg)
        self._kick_synth()

    def _on_interrupt(self, msg: Interrupt):
        self.get_logger().info(f'interrupt received: {msg.reason}')
        self._cancel.set()
        with self._buffer_lock:
            self._buffer.clear()
        self._emit_status('interrupted')

    def _on_language_update(self, msg: LanguagePreference):
        if msg.language and msg.language != self._current_language:
            self.get_logger().info(
                f'language switch: {self._current_language} -> {msg.language} '
                f'(source={msg.source})')
            self._current_language = msg.language

    def _on_speak(self, request, response):
        """Legacy sync API: enqueue + wait for completion."""
        if not request.text:
            response.success = False
            response.message = 'empty text'
            response.duration_seconds = 0.0
            return response

        req = TtsRequest()
        req.text = request.text
        req.language = ''  # use current
        req.voice = ''
        with self._buffer_lock:
            self._buffer.append(req)
        self._kick_synth()

        t0 = time.time()
        while time.time() - t0 < 30:
            with self._buffer_lock:
                empty = len(self._buffer) == 0
            if empty and not self._is_speaking:
                break
            time.sleep(0.05)

        response.success = True
        response.message = 'ok'
        response.duration_seconds = float(time.time() - t0)
        return response

    # ---- synthesis driver ----

    def _kick_synth(self):
        if self._synth_thread is not None and self._synth_thread.is_alive():
            return
        self._cancel.clear()
        self._synth_thread = threading.Thread(target=self._synth_loop,
                                                daemon=True)
        self._synth_thread.start()

    def _synth_loop(self):
        self._is_speaking = True
        if self.drive_face:
            self._set_face(FaceCommand.STATE_SPEAKING, 0.6)
        self._emit_status('started')

        try:
            while not self._cancel.is_set():
                with self._buffer_lock:
                    if not self._buffer:
                        break
                    req = self._buffer.pop(0)
                self._synthesize_stream(req)
        finally:
            self._is_speaking = False
            if self.drive_face:
                self._set_face(FaceCommand.STATE_STANDBY)
            if not self._cancel.is_set():
                self._emit_status('done')

    def _resolve_language(self, req: TtsRequest) -> str:
        if req.language:
            return req.language
        return self._current_language or self.default_lang

    def _resolve_voice(self, req: TtsRequest, lang: str) -> str:
        if req.voice:
            return req.voice
        return self.DEFAULT_VOICES.get(lang, self.DEFAULT_VOICES['ro'])

    def _pick_engine(self, lang: str, voice: str):
        """Choose Piper for RO/Piper-named voices, Kokoro for EN/Kokoro voices.

        Voice naming convention disambiguates:
            Piper:  "<lang>_<region>-<speaker>-<quality>"  e.g. ro_RO-mihai-medium
            Kokoro: "[ab][fm]_<name>"                       e.g. af_bella, bm_george
        """
        if voice:
            if voice.startswith(('ro_', 'en_US-', 'en_GB-')) and '-' in voice:
                return self._piper
            if len(voice) >= 3 and voice[0] in 'ab' and voice[1] in 'fm' \
                    and voice[2] == '_':
                return self._kokoro
        # Fall back to language-based choice
        if lang == 'en':
            return self._kokoro
        return self._piper

    def _synthesize_stream(self, req: TtsRequest):
        """Stream PCM chunks for req.text via the appropriate engine.

        Output cadence: chunks roughly CHUNK_MS in size at self.sr.

        Sample-rate alignment: each engine resamples its native output to
        self.sr before yielding (Piper 22050 -> sr, Kokoro 24000 -> sr).
        The pipeline downstream sees a single rate and the player needs no
        per-stream re-init.
        """
        text = req.text
        lang = self._resolve_language(req)
        voice = self._resolve_voice(req, lang)
        engine = self._pick_engine(lang, voice)

        self.get_logger().debug(
            f'synth [{lang}/{voice}] via {engine.name} '
            f'@ {engine.output_sample_rate}Hz: {text[:60]}...')

        # Re-slice at the pipeline rate (= engine.output_sample_rate = self.sr)
        target_chunk_bytes = int(engine.output_sample_rate
                                  * self.CHUNK_MS / 1000) * 2  # int16

        cancel_cb = self._cancel.is_set
        buf = bytearray()
        for chunk in engine.synthesize_stream(text, voice=voice,
                                                cancel=cancel_cb):
            if self._cancel.is_set():
                return
            buf.extend(chunk)
            # Re-slice into ~CHUNK_MS chunks
            while len(buf) >= target_chunk_bytes:
                if self._cancel.is_set():
                    return
                slice_ = bytes(buf[:target_chunk_bytes])
                del buf[:target_chunk_bytes]
                self._emit_chunk(slice_)
        # Flush remainder
        if buf and not self._cancel.is_set():
            self._emit_chunk(bytes(buf))

    def _emit_chunk(self, pcm_bytes: bytes):
        # UInt8MultiArray.data accepts array.array('B', ...) directly.
        from array import array
        msg = UInt8MultiArray()
        msg.data = array('B', pcm_bytes)
        self._stream_pub.publish(msg)

    def _emit_status(self, status: str):
        m = String()
        m.data = status
        self._status_pub.publish(m)

    def _set_face(self, state: int, amp: float = 0.0):
        cmd = FaceCommand()
        cmd.state = state
        cmd.amplitude = amp
        self._face_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TtsService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
