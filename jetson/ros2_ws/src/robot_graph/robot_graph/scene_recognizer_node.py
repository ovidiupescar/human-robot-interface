"""Scene recognizer — visual location identification.

Camera-gated: only runs when camera frames arrive on /camera/image_jpeg.
For now this is a stub that publishes "unknown" location at 0.1Hz so the rest
of the pipeline works without a camera.

Subscribes:
    /camera/image_jpeg  (std_msgs/ByteMultiArray) — TODO when camera node wired
Publishes:
    /perception/current_location (LocationIdentity)
    /perception/scene_changed   (std_msgs/Bool)
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from robot_graph_msgs.msg import LocationIdentity


class SceneRecognizer(Node):
    def __init__(self):
        super().__init__('scene_recognizer')

        self._loc_pub = self.create_publisher(
            LocationIdentity, '/perception/current_location', 10)
        self._change_pub = self.create_publisher(Bool, '/perception/scene_changed', 10)

        # TODO: subscribe to /camera/image_jpeg when camera_driver is added.
        # self.create_subscription(ByteMultiArray, '/camera/image_jpeg',
        #                          self._on_frame, 5)

        # For now: publish "unknown" every 10s so downstream knows we're alive.
        self.create_timer(10.0, self._publish_unknown)
        self.get_logger().info(
            'scene_recognizer running in STUB mode (no camera). '
            'Wire /camera/image_jpeg to enable.')

    def _publish_unknown(self):
        msg = LocationIdentity()
        msg.is_unknown = True
        msg.confidence = 0.0
        msg.stamp = self.get_clock().now().to_msg()
        self._loc_pub.publish(msg)

    # TODO: _on_frame(): embed frame, compare vs known centroids, publish best match
    # TODO: detect scene change (cosine drop vs last frame), publish /scene_changed


def main(args=None):
    rclpy.init(args=args)
    node = SceneRecognizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
