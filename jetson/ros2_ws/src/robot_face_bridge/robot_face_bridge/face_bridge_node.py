"""Serial JSON bridge between ROS2 and ESP32 face board.

Subscribes:  /face/command  (robot_face_msgs/FaceCommand)
Publishes:   /face/state    (robot_face_msgs/FaceState)
Services:    /face/set_state (robot_face_msgs/SetFaceState)

ESP32 firmware accepts JSON lines: {"state": N, "amp": F}

The node tolerates the serial port being absent at startup or going away
during runtime. It logs a single warning per connection attempt cycle and
keeps trying until the ESP32 is plugged in.
"""

import json
import threading

import rclpy
import serial
from rclpy.node import Node

from robot_face_msgs.msg import FaceCommand, FaceState
from robot_face_msgs.srv import SetFaceState


class FaceBridge(Node):
    RECONNECT_INTERVAL_S = 2.0

    def __init__(self):
        super().__init__('face_bridge')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self._port = self.get_parameter('serial_port').value
        self._baud = int(self.get_parameter('baud_rate').value)

        self._serial: serial.Serial | None = None
        self._serial_lock = threading.Lock()
        self._last_open_failed = False

        # State cache (valid even when disconnected)
        self._current_state = FaceCommand.STATE_STANDBY
        self._current_amp = 0.0

        self._state_pub = self.create_publisher(FaceState, '/face/state', 10)
        self.create_subscription(FaceCommand, '/face/command', self._on_command, 10)
        self.create_service(SetFaceState, '/face/set_state', self._on_set_state)

        self._reader_alive = True
        self._reader = threading.Thread(target=self._read_serial_loop, daemon=True)
        self._reader.start()

        self.create_timer(self.RECONNECT_INTERVAL_S, self._ensure_connected)
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info(
            f'face_bridge ready; will connect to {self._port} when available')
        self._ensure_connected()

    def _ensure_connected(self):
        with self._serial_lock:
            if self._serial is not None and self._serial.is_open:
                return
            try:
                self._serial = serial.Serial(self._port, self._baud, timeout=0.1)
                self._last_open_failed = False
                self.get_logger().info(f'connected to {self._port} @ {self._baud}')
            except serial.SerialException as e:
                if not self._last_open_failed:
                    self.get_logger().warning(
                        f'cannot open {self._port}: {e}; will keep retrying every '
                        f'{self.RECONNECT_INTERVAL_S:.1f}s')
                    self._last_open_failed = True
                self._serial = None

    def _on_command(self, msg: FaceCommand):
        self._send(msg.state, msg.amplitude)

    def _on_set_state(self, request, response):
        ok = self._send(request.state, request.amplitude)
        response.success = ok
        response.message = 'ok' if ok else 'serial disconnected or write failed'
        return response

    def _send(self, state: int, amp: float) -> bool:
        self._current_state = state
        self._current_amp = amp
        payload = json.dumps({'state': int(state), 'amp': float(amp)}) + '\n'
        with self._serial_lock:
            if self._serial is None or not self._serial.is_open:
                return False
            try:
                self._serial.write(payload.encode('utf-8'))
                return True
            except serial.SerialException as e:
                self.get_logger().error(f'serial write failed: {e}; dropping connection')
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
                return False

    def _read_serial_loop(self):
        buf = b''
        while self._reader_alive:
            with self._serial_lock:
                ser = self._serial
            if ser is None or not ser.is_open:
                # No connection; back off briefly. _ensure_connected will retry.
                threading.Event().wait(0.5)
                continue
            try:
                chunk = ser.read(64)
            except serial.SerialException:
                with self._serial_lock:
                    if self._serial is ser:
                        try:
                            self._serial.close()
                        except Exception:
                            pass
                        self._serial = None
                continue
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('utf-8', errors='replace').strip()
                if text:
                    self.get_logger().debug(f'esp32: {text}')

    def _publish_state(self):
        msg = FaceState()
        msg.state = self._current_state
        msg.amplitude = self._current_amp
        msg.stamp = self.get_clock().now().to_msg()
        self._state_pub.publish(msg)

    def destroy_node(self):
        self._reader_alive = False
        with self._serial_lock:
            try:
                if self._serial is not None:
                    self._serial.close()
            except Exception:
                pass
            self._serial = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
