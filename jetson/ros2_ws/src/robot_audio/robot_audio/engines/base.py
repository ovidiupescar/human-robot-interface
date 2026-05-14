"""Engine base + helpers."""

from __future__ import annotations

from math import gcd
from typing import Callable, Iterator, Optional

import numpy as np

try:
    from scipy.signal import resample_poly
    _SCIPY_AVAILABLE = True
except ImportError:
    _SCIPY_AVAILABLE = False


class Engine:
    """Base class for TTS engines.

    `target_sample_rate`  : the engine's native output rate
    `output_sample_rate`  : the rate the engine must emit at; defaults to native.
                            If different from native, the engine resamples its
                            output before yielding so the pipeline sees one rate.
    """

    name: str = "base"
    target_sample_rate: int = 22050
    output_sample_rate: int = 22050
    available: bool = False

    def set_output_sample_rate(self, sr: int):
        self.output_sample_rate = int(sr)

    def synthesize_stream(self, text: str, voice: str = "",
                            cancel: Callable[[], bool] = lambda: False
                            ) -> Iterator[bytes]:
        """Yield PCM16 mono chunks at `output_sample_rate`.

        `cancel` is a callable polled between chunks; truthy return aborts.
        """
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Resampling
# ---------------------------------------------------------------------------

def _reduce_ratio(up: int, down: int) -> tuple[int, int]:
    g = gcd(up, down)
    return up // g, down // g


def resample_audio(samples: np.ndarray, src_sr: int, dst_sr: int) -> np.ndarray:
    """Resample mono audio. Returns float32 in [-1, 1] regardless of input dtype.

    Uses scipy.signal.resample_poly when available (polyphase, high quality).
    Falls back to numpy linear interpolation otherwise — acceptable for speech
    but introduces mild aliasing at large ratio changes.
    """
    if samples.size == 0 or src_sr == dst_sr:
        if samples.dtype == np.int16:
            return samples.astype(np.float32) / 32768.0
        return samples.astype(np.float32, copy=False)

    if samples.dtype == np.int16:
        x = samples.astype(np.float32) / 32768.0
    else:
        x = samples.astype(np.float32, copy=False)

    if _SCIPY_AVAILABLE:
        up, down = _reduce_ratio(dst_sr, src_sr)
        return resample_poly(x, up, down).astype(np.float32, copy=False)

    # Fallback: linear interpolation
    duration_s = len(x) / float(src_sr)
    n_out = int(round(duration_s * dst_sr))
    if n_out <= 0:
        return np.zeros(0, dtype=np.float32)
    src_t = np.linspace(0.0, duration_s, num=len(x), endpoint=False)
    dst_t = np.linspace(0.0, duration_s, num=n_out, endpoint=False)
    return np.interp(dst_t, src_t, x).astype(np.float32)


def float_to_pcm16_bytes(x: np.ndarray) -> bytes:
    """Convert float32 [-1, 1] to PCM16 bytes."""
    clipped = np.clip(x, -1.0, 1.0)
    pcm = (clipped * 32767.0).astype(np.int16)
    return pcm.tobytes()


def silence_chunks(text: str, sample_rate: int, chunk_ms: int = 50,
                    cancel: Callable[[], bool] = lambda: False
                    ) -> Iterator[bytes]:
    """Fallback used when an engine's backing library is missing.

    Emits PCM16 silence at `sample_rate` pacing approximate speech duration.
    """
    seconds = max(0.5, 0.06 * len(text))
    chunks = max(1, int(seconds * 1000 / chunk_ms))
    per_chunk_samples = int(sample_rate * chunk_ms / 1000)
    silence = np.zeros(per_chunk_samples, dtype=np.int16).tobytes()
    import time as _time
    for _ in range(chunks):
        if cancel():
            return
        yield silence
        _time.sleep(chunk_ms / 1000.0)
