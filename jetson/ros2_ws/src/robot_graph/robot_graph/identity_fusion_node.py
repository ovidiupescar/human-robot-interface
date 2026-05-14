"""Identity fusion — combines transcripts, speaker identity, and current location
into a single IdentifiedSpeech event the rest of the system can consume.

Subscribes:
    /perception/transcript          (String)
    /perception/identified_person   (PersonIdentity)
    /perception/current_location    (LocationIdentity)
    /perception/addressee_score     (Float32)
Publishes:
    /perception/identified_speech   (IdentifiedSpeech)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from robot_graph_msgs.msg import (
    IdentifiedSpeech,
    LocationIdentity,
    PersonIdentity,
)


class IdentityFusion(Node):
    def __init__(self):
        super().__init__('identity_fusion')

        self.create_subscription(String, '/perception/transcript', self._on_text, 20)
        self.create_subscription(PersonIdentity, '/perception/identified_person',
                                 self._on_person, 20)
        self.create_subscription(LocationIdentity, '/perception/current_location',
                                 self._on_location, 20)
        self.create_subscription(Float32, '/perception/addressee_score',
                                 self._on_addressee, 20)

        self._pub = self.create_publisher(
            IdentifiedSpeech, '/perception/identified_speech', 10)

        # Latest values
        self._last_person: PersonIdentity = PersonIdentity()
        self._last_location: LocationIdentity = LocationIdentity()
        self._last_location.is_unknown = True
        self._last_addressee: float = 0.5

        self.get_logger().info('identity_fusion ready')

    def _on_person(self, msg: PersonIdentity):
        self._last_person = msg

    def _on_location(self, msg: LocationIdentity):
        self._last_location = msg

    def _on_addressee(self, msg: Float32):
        self._last_addressee = float(msg.data)

    def _on_text(self, msg: String):
        out = IdentifiedSpeech()
        out.text = msg.data
        out.speaker = self._last_person
        out.location = self._last_location
        out.start_stamp = self.get_clock().now().to_msg()
        out.end_stamp = out.start_stamp
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = IdentityFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
