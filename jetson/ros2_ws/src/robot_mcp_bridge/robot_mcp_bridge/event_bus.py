"""In-process pub/sub for perception events.

Used to fan perception updates from the rclpy executor threads (where the
subscription callbacks run) out to every WebSocket client connected to
`/events`. The bus is intentionally minimal:

  * `publish(event)` is non-blocking; it enqueues into per-client asyncio
    queues from any thread via `loop.call_soon_threadsafe`.
  * Disconnected clients are reaped when the queue read raises.
  * No buffering across reconnects — perception is realtime, and a stale
    `voice_active=True` frame from 5 seconds ago is worse than missing it.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any, Optional

log = logging.getLogger("robot_mcp_bridge.events")


class EventBus:
    def __init__(self) -> None:
        # Each subscriber owns a queue. We hold a set of (queue, loop).
        self._subscribers: set[tuple[asyncio.Queue, asyncio.AbstractEventLoop]] = set()
        self._lock = asyncio.Lock()

    async def subscribe(self) -> asyncio.Queue:
        """Register a new WebSocket subscriber. Returns its queue."""
        q: asyncio.Queue = asyncio.Queue(maxsize=256)
        loop = asyncio.get_running_loop()
        async with self._lock:
            self._subscribers.add((q, loop))
        log.info("subscriber added; total=%d", len(self._subscribers))
        return q

    async def unsubscribe(self, queue: asyncio.Queue) -> None:
        async with self._lock:
            self._subscribers = {(q, l) for (q, l) in self._subscribers
                                  if q is not queue}
        log.info("subscriber removed; total=%d", len(self._subscribers))

    def publish(self, event: dict[str, Any]) -> None:
        """Broadcast an event. Safe to call from any thread, including
        rclpy executor threads.
        """
        for queue, loop in list(self._subscribers):
            try:
                loop.call_soon_threadsafe(self._enqueue, queue, event)
            except RuntimeError:
                # Loop closed (subscriber going away). Skip.
                pass

    @staticmethod
    def _enqueue(queue: asyncio.Queue, event: dict[str, Any]) -> None:
        try:
            queue.put_nowait(event)
        except asyncio.QueueFull:
            # Drop oldest, then push new. Realtime > completeness.
            try:
                queue.get_nowait()
                queue.put_nowait(event)
            except (asyncio.QueueEmpty, asyncio.QueueFull):
                pass


# Single global instance shared by the rclpy node and the daemon's
# WebSocket route. Created in build_app() so the lifespan owns it.
_INSTANCE: Optional[EventBus] = None


def get_bus() -> EventBus:
    global _INSTANCE
    if _INSTANCE is None:
        _INSTANCE = EventBus()
    return _INSTANCE
