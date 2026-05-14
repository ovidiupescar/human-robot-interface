"""robot_bridge — clean Python facade over ROS2 for Hermes skills.

Usage:
    from robot_bridge import RobotBridge
    rb = RobotBridge()
    rb.set_face("speaking", 0.5)
    rb.speak("Hello world")
    text = rb.listen(timeout=8.0)
"""

from .bridge import RobotBridge
from .states import FaceStateName, STATE_NAME_TO_INT

__all__ = ["RobotBridge", "FaceStateName", "STATE_NAME_TO_INT"]
