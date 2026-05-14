"""Serial JSON bridge between ROS2 and ESP32 face board.

Subscribes:  /face/command  (robot_face_msgs/FaceCommand)
Publishes:   /face/state    (robot_face_msgs/FaceState)
Services:    /face/set_state (robot_face_msgs/SetFaceState)

ESP32 firmware accepts JSON lines: {"state": N, "amp": F}
"""

import json
import threading

import rclpy
import serial
from rclpy.node import Node

from robot_face_msgs.msg import FaceCommand, FaceState
from robot_face_msgs.srv import SetFaceState


class FaceBridge(Node):
    def __init__(self):
        super().__init__('face_bridge')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        self.get_logger().info(f'opening {port} @ {baud}')
        try:
            self._serial = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f'serial open failed: {e}')
            raise

        # State cache
        self._current_state = FaceCommand.STATE_STANDBY
        self._current_amp = 0.0

        # Pub / sub / srv
        self._state_pub = self.create_publisher(FaceState, '/face/state', 10)
        self.create_subscription(FaceCommand, '/face/command', self._on_command, 10)
        self.create_service(SetFaceState, '/face/set_state', self._on_set_state)

        # Reader thread for ESP32 logs / acks
        self._reader_alive = True
        self._reader = threading.Thread(target=self._read_serial, daemon=True)
        self._reader.start()

        # Heartbeat state publication @ 1Hz
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info('face_bridge ready')

    # ---- command handlers ----

    def _on_command(self, msg: FaceCommand):
        self._send(msg.state, msg.amplitude)

    def _on_set_state(self, request, response):
        ok = self._send(request.state, request.amplitude)
        response.success = ok
        response.message = 'ok' if ok else 'serial write failed'
        return response

    # ---- serial I/O ----

    def _send(self, state: int, amp: float) -> bool:
        payload = json.dumps({'state': int(state), 'amp': float(amp)}) + '\n'
        try:
            self._serial.write(payload.encode('utf-8'))
            self._current_state = state
            self._current_amp = amp
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'serial write: {e}')
            return False

    def _read_serial(self):
        buf = b''
        while self._reader_alive:
            try:
                chunk = self._serial.read(64)
                if not chunk:
                    continue
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    text = line.decode('utf-8', errors='replace').strip()
                    if text:
                        self.get_logger().debug(f'esp32: {text}')
            except serial.SerialException:
                self._reader_alive = False
                break

    def _publish_state(self):
        msg = FaceState()
        msg.state = self._current_state
        msg.amplitude = self._current_amp
        msg.stamp = self.get_clock().now().to_msg()
        self._state_pub.publish(msg)

    def destroy_node(self):
        self._reader_alive = False
        try:
            self._serial.close()
        except Exception:
            pass
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
