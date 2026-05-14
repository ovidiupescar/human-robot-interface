"""Composable perception executor — single-process group.

Runs VAD, wake-word, speech recognizer, and addressee estimator in one
MultiThreadedExecutor under one process. This eliminates inter-process IPC
serialization between hot-path perception stages and shares a single Python
interpreter (one CUDA context, one model cache).

Rationale:
    Audio chunks arrive at 20ms cadence. Routing them through separate ROS2
    nodes in separate processes costs ~1-3ms per hop in serialization and
    copy. Co-locating in a MultiThreadedExecutor lets ROS2 intraprocess
    communication pass message pointers without copy.

To use this in place of the four individual `Node(...)` entries in
robot.launch.py, run:
    ros2 run robot_perception perception_executor

Note: rclpy doesn't have true component containers like rclcpp, but a single
process with one MultiThreadedExecutor is the practical equivalent for the
Python stack.
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_perception.voice_activity_node import VoiceActivity
from robot_perception.speech_recognizer_node import SpeechRecognizer
from robot_perception.wake_word_node import WakeWord
from robot_perception.addressee_estimator_node import AddresseeEstimator
from robot_perception.language_resolver_node import LanguageResolver
from robot_perception.vision_capture_node import VisionCapture


def main(args=None):
    rclpy.init(args=args)

    nodes = [
        VoiceActivity(),
        SpeechRecognizer(),
        WakeWord(),
        AddresseeEstimator(),
        LanguageResolver(),
        VisionCapture(),
    ]

    executor = MultiThreadedExecutor(num_threads=6)
    for n in nodes:
        executor.add_node(n)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
