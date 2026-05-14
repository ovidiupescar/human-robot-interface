"""TTS engine implementations.

Each engine exposes:
    name: str
    available: bool                     # True if backing lib is importable
    target_sample_rate: int             # native SR (caller may resample)
    synthesize_stream(text) -> Iterator[bytes]   # PCM16 mono chunks

The Engine.silence_fallback() helper produces silence-pacing chunks for use
when the backing lib is not installed, so the rest of the pipeline can still
exercise the topology.
"""

from .base import (
    Engine,
    float_to_pcm16_bytes,
    resample_audio,
    silence_chunks,
)
from .piper_engine import PiperEngine
from .kokoro_engine import KokoroEngine

__all__ = [
    'Engine',
    'PiperEngine',
    'KokoroEngine',
    'float_to_pcm16_bytes',
    'resample_audio',
    'silence_chunks',
]
