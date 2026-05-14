"""Piper TTS engine — Romanian (and any other Piper voice).

Streams PCM16 mono chunks from a Piper voice (.onnx model).
"""

from __future__ import annotations

import os
import threading
from pathlib import Path
from typing import Callable, Dict, Iterator, Optional

import numpy as np

try:
    from piper import PiperVoice
    PIPER_AVAILABLE = True
except ImportError:
    PiperVoice = None  # type: ignore
    PIPER_AVAILABLE = False

from .base import (
    Engine,
    float_to_pcm16_bytes,
    resample_audio,
    silence_chunks,
)


# How much audio to accumulate before running a resample pass.
# Larger = better quality at chunk boundaries, but more latency.
# 200ms gives ~9 chunks of 22050 sr * 50ms each; quality is excellent
# and the added latency only affects the first chunk of the utterance.
_RESAMPLE_BUFFER_MS = 200


class PiperEngine(Engine):
    """Lazy-loaded Piper voices keyed by voice id (e.g., 'ro_RO-mihai-medium').

    Voice models are loaded on first use. Each .onnx file lives in
    `voice_dir/{voice_id}.onnx` and its config in `voice_dir/{voice_id}.onnx.json`.
    """

    name = "piper"
    target_sample_rate = 22050     # Piper voices we ship are 22050 native

    def __init__(self, voice_dir: str = '/opt/piper/voices',
                  default_voice: str = 'ro_RO-mihai-medium',
                  use_cuda: bool = True,
                  output_sample_rate: int = 22050):
        self.available = PIPER_AVAILABLE
        self._voice_dir = Path(voice_dir)
        self._default_voice = default_voice
        self._use_cuda = use_cuda
        self.output_sample_rate = int(output_sample_rate)
        self._voices: Dict[str, object] = {}
        self._lock = threading.Lock()

    def _voice_path(self, voice_id: str) -> Path:
        return self._voice_dir / f"{voice_id}.onnx"

    def _load_voice(self, voice_id: str):
        if voice_id in self._voices:
            return self._voices[voice_id]
        path = self._voice_path(voice_id)
        if not path.exists():
            raise FileNotFoundError(
                f"Piper voice not found: {path}. "
                f"Download from rhasspy/piper-voices on HF and place under "
                f"{self._voice_dir}/")
        voice = PiperVoice.load(str(path), use_cuda=self._use_cuda)
        self._voices[voice_id] = voice
        # Update target SR from the loaded voice config
        try:
            self.target_sample_rate = voice.config.sample_rate
        except AttributeError:
            pass
        return voice

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
                pv = self._load_voice(voice_id)
        except Exception:
            yield from silence_chunks(text, self.output_sample_rate,
                                       cancel=cancel)
            return

        needs_resample = self.target_sample_rate != self.output_sample_rate

        try:
            if not needs_resample:
                # Fast path: Piper native SR matches pipeline SR. Pass through.
                for audio_chunk in pv.synthesize_stream_raw(text):
                    if cancel():
                        return
                    yield audio_chunk
            else:
                # Resample path: buffer ~200ms of audio at native SR, then
                # resample the batch once to avoid boundary artifacts.
                yield from self._stream_with_resample(pv, text, cancel)
        except Exception:
            yield from silence_chunks(' ', self.output_sample_rate,
                                       cancel=cancel)

    def _stream_with_resample(self, pv, text: str,
                                cancel: Callable[[], bool]
                                ) -> Iterator[bytes]:
        """Buffered streaming resample: collect ~200ms at native SR, resample,
        emit. Latency added = one buffer (~200ms) plus engine synthesis time.
        """
        buffer_bytes = bytearray()
        flush_size = int(self.target_sample_rate
                          * _RESAMPLE_BUFFER_MS / 1000) * 2  # int16

        for audio_chunk in pv.synthesize_stream_raw(text):
            if cancel():
                return
            buffer_bytes.extend(audio_chunk)
            while len(buffer_bytes) >= flush_size:
                batch = bytes(buffer_bytes[:flush_size])
                del buffer_bytes[:flush_size]
                yield self._resample_batch(batch)
                if cancel():
                    return

        # Flush remainder
        if buffer_bytes and not cancel():
            yield self._resample_batch(bytes(buffer_bytes))

    def _resample_batch(self, pcm_bytes: bytes) -> bytes:
        import numpy as np
        samples = np.frombuffer(pcm_bytes, dtype=np.int16)
        resampled = resample_audio(samples, self.target_sample_rate,
                                    self.output_sample_rate)
        return float_to_pcm16_bytes(resampled)
