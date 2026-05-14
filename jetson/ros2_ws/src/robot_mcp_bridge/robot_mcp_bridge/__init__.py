"""Bridge daemon between the ROS2 graph and Hermes Agent.

Runs under system Python 3.10 (Hermes Humble's rclpy ABI) and exposes ROS2
actions as MCP tools over HTTP+SSE so Hermes (Python 3.11) can drive the
robot without ever importing rclpy.
"""
