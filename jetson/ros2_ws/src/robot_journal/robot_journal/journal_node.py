"""Journal node — append-only daily JSONL of everything.

Subscribes to a curated set of topics and writes a normalized JSON event
per message. Files rotate at local midnight. Yesterday's file gzips on rotation.

Layout:
    ~/robot_data/journal/
        2026-05-11.jsonl       (today, open for append)
        2026-05-10.jsonl.gz    (gzipped after rotation)
        checkpoint.json        (memorist's last-read position)
        index.json             (per-day stats; written nightly)

Anything in the system that wants to be on the record publishes a
robot_graph_msgs/JournalEntry to /journal/append. This node also opportunistically
subscribes to a few canonical topics so writers don't have to wrap each one.
"""

import gzip
import json
import os
import queue
import shutil
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, ByteMultiArray, Float32, String

from robot_face_msgs.msg import FaceCommand, FaceState
from robot_graph_msgs.msg import (
    IdentifiedSpeech,
    JournalEntry,
    LocationIdentity,
    PersonIdentity,
)


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def _today_str() -> str:
    return datetime.now().strftime('%Y-%m-%d')


class JournalNode(Node):
    """Async append-only journal.

    Publishers never block on disk I/O. Events go into an in-RAM queue;
    a background writer thread drains the queue, flushing every 500ms or
    every N events, whichever comes first.

    Worst case on hard power loss: lose up to ~500ms of unflushed events.
    Acceptable for a journal that's already best-effort.
    """

    FLUSH_INTERVAL_S = 0.5
    FLUSH_BATCH = 10
    QUEUE_MAX = 10000  # safety bound; drops oldest on overflow

    def __init__(self):
        super().__init__('journal')
        self.declare_parameter('journal_dir', str(Path.home() / 'robot_data' / 'journal'))
        self._dir = Path(self.get_parameter('journal_dir').value)
        self._dir.mkdir(parents=True, exist_ok=True)

        self._file_lock = threading.Lock()  # only used during rotation
        self._current_day = _today_str()
        self._fp = open(self._dir / f'{self._current_day}.jsonl', 'a', encoding='utf-8')

        # Async write pipeline
        self._queue: "queue.Queue[dict]" = queue.Queue(maxsize=self.QUEUE_MAX)
        self._stop = threading.Event()
        self._writer = threading.Thread(target=self._writer_loop,
                                         name='journal_writer', daemon=True)
        self._writer.start()

        # Explicit channel for nodes that want full control of the record
        self.create_subscription(JournalEntry, '/journal/append', self._on_entry, 100)

        # Convenience subscriptions — common things to record without ceremony
        self.create_subscription(String, '/perception/transcript', self._on_transcript, 50)
        self.create_subscription(Bool,   '/perception/voice_active', self._on_voice_active, 50)
        self.create_subscription(IdentifiedSpeech, '/perception/identified_speech',
                                 self._on_identified_speech, 50)
        self.create_subscription(PersonIdentity, '/perception/identified_person',
                                 self._on_identified_person, 20)
        self.create_subscription(LocationIdentity, '/perception/current_location',
                                 self._on_current_location, 20)
        self.create_subscription(FaceCommand, '/face/command', self._on_face_cmd, 20)
        self.create_subscription(FaceState,   '/face/state',   self._on_face_state, 5)
        self.create_subscription(String,      '/audio/playback_status',
                                 self._on_playback_status, 20)

        # Rotate every minute
        self.create_timer(60.0, self._maybe_rotate)
        self.get_logger().info(f'journal writing to {self._dir}')

    # ---- write helpers ----

    def _write(self, kind: str, source: str, payload: dict):
        """Non-blocking enqueue. Returns in microseconds — publishers never wait."""
        record = {'t': _now_iso(), 'kind': kind, 'source': source, **payload}
        try:
            self._queue.put_nowait(record)
        except queue.Full:
            # Backpressure: drop oldest to make room. Loss > stall.
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(record)
            except queue.Full:
                self.get_logger().warning('journal queue full; event dropped')

    def _writer_loop(self):
        """Background thread: drain queue, batch-write, flush periodically."""
        last_flush = time.monotonic()
        batch: list[str] = []
        while not self._stop.is_set():
            try:
                record = self._queue.get(timeout=self.FLUSH_INTERVAL_S)
                batch.append(json.dumps(record, ensure_ascii=False))
            except queue.Empty:
                pass

            now = time.monotonic()
            should_flush = (
                len(batch) >= self.FLUSH_BATCH
                or (batch and (now - last_flush) >= self.FLUSH_INTERVAL_S)
            )
            if not should_flush:
                continue

            with self._file_lock:
                try:
                    self._fp.write('\n'.join(batch) + '\n')
                    self._fp.flush()
                except OSError as e:
                    # Don't crash the writer; log and keep going
                    self.get_logger().error(f'journal write failed: {e}')
            batch.clear()
            last_flush = now

        # Drain on shutdown
        remaining = []
        try:
            while True:
                remaining.append(json.dumps(self._queue.get_nowait(),
                                             ensure_ascii=False))
        except queue.Empty:
            pass
        if remaining:
            with self._file_lock:
                try:
                    self._fp.write('\n'.join(remaining) + '\n')
                    self._fp.flush()
                except OSError:
                    pass

    # ---- subscription callbacks ----

    def _on_entry(self, msg: JournalEntry):
        try:
            payload = json.loads(msg.payload_json) if msg.payload_json else {}
        except json.JSONDecodeError:
            payload = {'raw': msg.payload_json}
        self._write(msg.kind, msg.source or 'unknown', payload)

    def _on_transcript(self, msg: String):
        self._write('perception.voice', 'speech_recognizer', {'text': msg.data})

    def _on_voice_active(self, msg: Bool):
        self._write('perception.voice_activity', 'voice_activity',
                    {'active': bool(msg.data)})

    def _on_identified_speech(self, msg: IdentifiedSpeech):
        self._write('perception.voice', 'identity_fusion', {
            'text': msg.text,
            'speaker_id': msg.speaker.person_id,
            'speaker_name': msg.speaker.primary_name,
            'voice_conf': msg.speaker.voice_confidence,
            'face_conf': msg.speaker.face_confidence,
            'fused_conf': msg.speaker.fused_confidence,
            'is_new_speaker': msg.speaker.is_new,
            'location_id': msg.location.location_id,
            'location_name': msg.location.name,
            'location_conf': msg.location.confidence,
        })

    def _on_identified_person(self, msg: PersonIdentity):
        self._write('perception.person_present', 'identity_fusion', {
            'person_id': msg.person_id,
            'name': msg.primary_name,
            'voice_conf': msg.voice_confidence,
            'face_conf': msg.face_confidence,
            'fused_conf': msg.fused_confidence,
            'is_new': msg.is_new,
        })

    def _on_current_location(self, msg: LocationIdentity):
        self._write('perception.location', 'scene_recognizer', {
            'location_id': msg.location_id,
            'name': msg.name,
            'parent': msg.parent_name,
            'confidence': msg.confidence,
            'is_unknown': msg.is_unknown,
        })

    def _on_face_cmd(self, msg: FaceCommand):
        self._write('actuator.face', 'face_bridge',
                    {'state': int(msg.state), 'amplitude': float(msg.amplitude)})

    def _on_face_state(self, msg: FaceState):
        self._write('actuator.face_state', 'face_bridge',
                    {'state': int(msg.state), 'amplitude': float(msg.amplitude)})

    def _on_playback_status(self, msg: String):
        self._write('actuator.audio', 'audio_player', {'status': msg.data})

    # ---- rotation ----

    def _maybe_rotate(self):
        today = _today_str()
        if today == self._current_day:
            return
        # Coordinate with writer: file_lock ensures no torn write across rotation
        with self._file_lock:
            self._fp.flush()
            self._fp.close()
            old_path = self._dir / f'{self._current_day}.jsonl'
            if old_path.exists():
                gz_path = old_path.with_suffix(old_path.suffix + '.gz')
                with open(old_path, 'rb') as src, gzip.open(gz_path, 'wb') as dst:
                    shutil.copyfileobj(src, dst)
                old_path.unlink()
            self._current_day = today
            self._fp = open(self._dir / f'{self._current_day}.jsonl', 'a', encoding='utf-8')
        self.get_logger().info(f'rotated journal: now writing {self._current_day}.jsonl')

    def destroy_node(self):
        # Stop writer, drain remaining, close file
        self._stop.set()
        if self._writer.is_alive():
            self._writer.join(timeout=2.0)
        try:
            self._fp.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JournalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
