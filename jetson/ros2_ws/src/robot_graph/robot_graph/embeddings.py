"""Pluggable embedding backends with stub defaults.

Real models (ECAPA-TDNN, CLIP, ArcFace) plug in by replacing the constructor
of the appropriate class. The interface stays stable so the rest of the system
doesn't change when models swap.
"""

from __future__ import annotations

import hashlib
from typing import Protocol

import numpy as np


class VoiceEmbedder(Protocol):
    name: str
    dim: int
    def embed(self, pcm_int16: bytes, sample_rate: int) -> np.ndarray: ...


class SceneEmbedder(Protocol):
    name: str
    dim: int
    def embed(self, image_jpeg: bytes) -> np.ndarray: ...


class FaceEmbedder(Protocol):
    name: str
    dim: int
    def embed(self, image_jpeg: bytes) -> np.ndarray: ...


# ----------- Stub implementations -----------
# Deterministic hash-based "embeddings" so the rest of the system works without
# real models. Same input -> same output. Replace before deploying for real.

def _hash_embed(data: bytes, dim: int, salt: str = "") -> np.ndarray:
    h = hashlib.sha256(salt.encode() + data).digest()
    # Repeat hash to fill dim
    raw = b''
    while len(raw) < dim * 4:
        h = hashlib.sha256(h).digest()
        raw += h
    arr = np.frombuffer(raw[:dim * 4], dtype=np.uint32).astype(np.float32)
    arr = arr / 4294967295.0 * 2.0 - 1.0  # [-1, 1]
    n = np.linalg.norm(arr) + 1e-9
    return (arr / n).astype(np.float32)


class StubVoiceEmbedder:
    name = "stub-voice-256"
    dim = 256

    def embed(self, pcm_int16: bytes, sample_rate: int) -> np.ndarray:
        return _hash_embed(pcm_int16, self.dim, salt="voice")


class StubSceneEmbedder:
    name = "stub-scene-512"
    dim = 512

    def embed(self, image_jpeg: bytes) -> np.ndarray:
        return _hash_embed(image_jpeg, self.dim, salt="scene")


class StubFaceEmbedder:
    name = "stub-face-512"
    dim = 512

    def embed(self, image_jpeg: bytes) -> np.ndarray:
        return _hash_embed(image_jpeg, self.dim, salt="face")


# ----------- Real implementations (deferred) -----------
# TODO: class EcapaTdnnEmbedder using speechbrain
# TODO: class ClipImageEmbedder using open_clip
# TODO: class ArcFaceEmbedder using insightface


# ----------- Matching utilities -----------

def cosine(a: np.ndarray, b: np.ndarray) -> float:
    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    return float(np.dot(a, b) / (na * nb))


def best_match(query: np.ndarray, candidates: list[tuple[str, np.ndarray]],
               threshold: float) -> tuple[str | None, float]:
    """Return (id, score) of best match or (None, score) below threshold."""
    if not candidates:
        return None, 0.0
    sims = [(cid, cosine(query, emb)) for cid, emb in candidates]
    sims.sort(key=lambda x: x[1], reverse=True)
    top_id, top_score = sims[0]
    if top_score < threshold:
        return None, top_score
    return top_id, top_score
