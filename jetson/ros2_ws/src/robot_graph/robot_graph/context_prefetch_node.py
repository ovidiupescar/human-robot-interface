"""Context Prefetch — proactively warm the memory context before STT finishes.

Idea (from robo-brain v0.2): waiting for the final transcript to start the
memory lookup is wasted time. The signals that *trigger* a likely query —
voice activity onset, face detected, identified speaker — fire 1–2s before
the transcript is ready. Kick off a recall query as soon as those signals
arrive, parallel with STT. By the time the orchestrator wants context, it's
already on /agent/context_warm.

Inputs:
    /perception/voice_active         (Bool)            VAD onset
    /perception/identified_person    (PersonIdentity)  current speaker
    /perception/current_location     (LocationIdentity) current place

Output:
    /agent/context_warm  (String, JSON)   recalled facts + metadata
        {
          "stamp_ms": int,
          "person_id": str|null,
          "person_name": str|null,
          "location_id": str|null,
          "facts": [{"id": str, "content": str, "score": float}, ...],
          "ttl_ms": int          # caller should treat as fresh within this window
        }

Strategy:
    - Cooldown: do not fire more than once per COOLDOWN_S
    - Subject preference: identified person > location > general "recent"
    - Result TTL: orchestrator may reuse for TTL_S; afterwards the prefetch
      becomes stale and should be re-fired (or fall back to on-demand recall)
    - Failure is silent: if /memory/recall is slow or unreachable, the
      orchestrator can still do its own recall (this is a *latency
      optimization*, not a correctness requirement)
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from robot_graph_msgs.msg import LocationIdentity, PersonIdentity
from robot_graph_msgs.srv import Recall


class ContextPrefetch(Node):

    COOLDOWN_S = 1.5          # min interval between prefetches
    TTL_S = 5.0               # how long the warm context stays fresh
    RECALL_TIMEOUT_S = 1.0    # wait this long for graph service
    RECALL_LIMIT = 8          # facts per prefetch
    IDENT_CONF_FLOOR = 0.55

    def __init__(self):
        super().__init__('context_prefetch')

        self.create_subscription(Bool, '/perception/voice_active',
                                  self._on_voice_active, 10)
        self.create_subscription(PersonIdentity,
                                  '/perception/identified_person',
                                  self._on_person, 10)
        self.create_subscription(LocationIdentity,
                                  '/perception/current_location',
                                  self._on_location, 10)

        self._pub = self.create_publisher(String, '/agent/context_warm', 10)
        self._client = self.create_client(Recall, '/memory/recall')

        # State for subject selection
        self._current_person: PersonIdentity | None = None
        self._current_location: LocationIdentity | None = None
        self._last_fire_t = 0.0
        self._lock = threading.Lock()

        self.get_logger().info('context_prefetch ready')

    def _on_person(self, msg: PersonIdentity):
        if msg.fused_confidence >= self.IDENT_CONF_FLOOR:
            self._current_person = msg

    def _on_location(self, msg: LocationIdentity):
        if not msg.is_unknown:
            self._current_location = msg

    def _on_voice_active(self, msg: Bool):
        if not msg.data:
            return
        now = time.time()
        with self._lock:
            if now - self._last_fire_t < self.COOLDOWN_S:
                return
            self._last_fire_t = now

        threading.Thread(target=self._prefetch, daemon=True).start()

    def _prefetch(self):
        subject_id, subject_type, label = self._pick_subject()
        if not subject_id:
            return

        if not self._client.wait_for_service(timeout_sec=0.2):
            self.get_logger().debug('graph service not ready, skipping')
            return

        req = Recall.Request()
        req.subject_id = subject_id
        req.subject_type = subject_type
        req.query = ''           # recent-only on prefetch; semantic later if needed
        req.limit = self.RECALL_LIMIT
        req.mode = 'recent'

        future = self._client.call_async(req)
        # Block this worker thread up to RECALL_TIMEOUT_S
        t0 = time.time()
        while not future.done() and (time.time() - t0) < self.RECALL_TIMEOUT_S:
            time.sleep(0.02)

        if not future.done():
            self.get_logger().debug(
                f'recall timeout for {subject_type}:{subject_id}')
            return

        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warning(f'recall failed: {e}')
            return

        if not resp.success:
            return

        facts = [
            {'id': fid, 'content': content, 'score': float(score)}
            for fid, content, score in zip(
                resp.fact_ids, resp.contents, resp.scores)
        ]

        payload = {
            'stamp_ms': int(time.time() * 1000),
            'subject_type': subject_type,
            'subject_id': subject_id,
            'subject_label': label,
            'person_id': (self._current_person.person_id
                          if self._current_person else None),
            'person_name': (self._current_person.primary_name
                            if self._current_person else None),
            'location_id': (self._current_location.location_id
                            if self._current_location else None),
            'location_name': (self._current_location.name
                              if self._current_location else None),
            'facts': facts,
            'ttl_ms': int(self.TTL_S * 1000),
        }
        m = String()
        m.data = json.dumps(payload, ensure_ascii=False)
        self._pub.publish(m)
        self.get_logger().info(
            f'prefetch -> {subject_type}:{subject_id} ({len(facts)} facts)')

    def _pick_subject(self) -> tuple[str, str, str]:
        """Return (id, type, label). Empty id means no usable subject."""
        if self._current_person and self._current_person.person_id:
            return (self._current_person.person_id, 'Person',
                    self._current_person.primary_name or '')
        if self._current_location and self._current_location.location_id:
            return (self._current_location.location_id, 'Location',
                    self._current_location.name or '')
        return ('', '', '')


def main(args=None):
    rclpy.init(args=args)
    node = ContextPrefetch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
