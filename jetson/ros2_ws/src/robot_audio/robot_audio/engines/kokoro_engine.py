"""Kokoro-82M TTS engine — English voices.

Wraps either:
    - kokoro-onnx (preferred for Jetson, no torch dep)
    - kokoro (Python pkg, requires torch)

Chosen voice: af_bella by default. Voices live under `model_dir` as ONNX
artifacts (kokoro-v1.0.onnx + voices.json or the multi-voice .onnx).

Output: 24 kHz PCM16 mono (Kokoro's native rate). Caller may resample.
"""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Callable, Iterator, Optional

import numpy as np

try:
    from kokoro_onnx import Kokoro
    KOKORO_AVAILABLE = True
    _KOKORO_BACKEND = 'onnx'
except ImportError:
    try:
        from kokoro import KPipeline as Kokoro  # type: ignore
        KOKORO_AVAILABLE = True
        _KOKORO_BACKEND = 'torch'
    except ImportError:
        Kokoro = None  # type: ignore
        KOKORO_AVAILABLE = False
        _KOKORO_BACKEND = None

from .base import (
    Engine,
    float_to_pcm16_bytes,
    resample_audio,
    silence_chunks,
)


class KokoroEngine(Engine):
    name = "kokoro"
    target_sample_rate = 24000     # Kokoro's native output rate

    # Chunk Kokoro's full waveform output into ~50ms slices for streaming.
    CHUNK_MS = 50

    def __init__(self, model_path: str = '/opt/kokoro/kokoro-v1.0.onnx',
                  voices_path: str = '/opt/kokoro/voices-v1.0.bin',
                  default_voice: str = 'af_bella',
                  output_sample_rate: int = 24000):
        self.available = KOKORO_AVAILABLE
        self._model_path = Path(model_path)
        self._voices_path = Path(voices_path)
        self._default_voice = default_voice
        self.output_sample_rate = int(output_sample_rate)
        self._pipeline: Optional[object] = None
        self._lock = threading.Lock()

    def _ensure_loaded(self):
        if self._pipeline is not None:
            return
        if _KOKORO_BACKEND == 'onnx':
            if not self._model_path.exists():
                raise FileNotFoundError(
                    f"Kokoro model not found: {self._model_path}")
            self._pipeline = Kokoro(str(self._model_path),
                                     str(self._voices_path))
        elif _KOKORO_BACKEND == 'torch':
            self._pipeline = Kokoro(lang_code='a')  # American English

    def synthesize_stream(self, text: str, voice: str = "",
                            cancel: Callable[[], bool] = lambda: False
                            ) -> Iterator[bytes]:
        if not self.available:
            yield from silence_chunks(text, self.output_sample_rate,
                                       cancel=cancel)
            return

        voice_id = voice or self._default_voice
        try:
            with self._lock:
                self._ensure_loaded()
        except Exception:
            yield from silence_chunks(text, self.output_sample_rate,
                                       cancel=cancel)
            return

        try:
            audio = self._synth_full(text, voice_id)
        except Exception:
            yield from silence_chunks(' ', self.output_sample_rate,
                                       cancel=cancel)
            return

        # Resample once at engine exit so the rest of the pipeline sees one rate.
        # resample_audio normalizes int16/float -> float32 in [-1, 1].
        audio_f = resample_audio(audio, self.target_sample_rate,
                                  self.output_sample_rate)

        # Slice into CHUNK_MS PCM16 chunks
        samples_per_chunk = int(self.output_sample_rate * self.CHUNK_MS / 1000)
        for start in range(0, len(audio_f), samples_per_chunk):
            if cancel():
                return
            slice_ = audio_f[start:start + samples_per_chunk]
            yield float_to_pcm16_bytes(slice_)

    def _synth_full(self, text: str, voice_id: str) -> np.ndarray:
        """Produce the full waveform once. Returns float32 [-1,1] or int16 array.

        kokoro-onnx returns (samples float32, sample_rate int).
        kokoro torch returns generator of (graphemes, phonemes, audio).
        """
        if _KOKORO_BACKEND == 'onnx':
            samples, sr = self._pipeline.create(text, voice=voice_id)
            self.target_sample_rate = int(sr) or self.target_sample_rate
            return samples
        # torch backend
        chunks = []
        for _g, _p, audio in self._pipeline(text, voice=voice_id):
            chunks.append(audio.cpu().numpy())
        if not chunks:
            return np.zeros(0, dtype=np.float32)
        return np.concatenate(chunks)
