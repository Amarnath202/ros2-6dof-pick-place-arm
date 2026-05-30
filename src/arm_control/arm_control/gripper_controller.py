#!/usr/bin/env python3
# ============================================================
# arm_control/arm_control/gripper_controller.py
#
# Standalone ROS2 node for direct gripper control.
# Subscribes to /gripper/command (Float32) for simple open/close.
# Also provides a ROS2 service for gripper actions.
# Publishes to the position controller topic.
#
# Topics:
#   Subscribe: /gripper/command  (std_msgs/Float32, 0.0=close, 1.0=open)
#   Subscribe: /gripper/width    (std_msgs/Float32, width in meters)
#   Publish:   /gripper_controller/commands (std_msgs/Float64MultiArray)
#
# Services:
#   /gripper/open   (std_srvs/Trigger)
#   /gripper/close  (std_srvs/Trigger)
# ============================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray
from std_srvs.srv import Trigger
import time


class GripperControllerNode(Node):
    """
    Gripper control node.
    Bridges high-level gripper commands to ros2_control position commands.
    """

    # Constants
    MAX_OPENING = 0.05   # meters — maximum finger travel (50mm per finger = 80mm total gap)
    MIN_OPENING = 0.002  # meters — minimum to avoid singularity

    def __init__(self):
        super().__init__('gripper_controller_node')

        self.get_logger().info('Initializing Gripper Controller Node...')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('gripper_open_position', 0.05)
        self.declare_parameter('gripper_close_position', 0.002)
        self.declare_parameter('command_topic', '/gripper_controller/commands')

        self.open_pos = self.get_parameter('gripper_open_position').value
        self.close_pos = self.get_parameter('gripper_close_position').value
        command_topic = self.get_parameter('command_topic').value

        # ── Current state ────────────────────────────────────────────────
        self._current_position = self.open_pos  # start open
        self._is_open = True

        # ── Publisher: to ros2_control position controller ───────────────
        self._cmd_pub = self.create_publisher(
            Float64MultiArray,
            command_topic,
            10
        )

        # ── Subscribers ──────────────────────────────────────────────────
        # Simple 0/1 command (0=close, 1=open)
        self._cmd_sub = self.create_subscription(
            Float32,
            '/gripper/command',
            self._command_callback,
            10
        )

        # Width command in meters
        self._width_sub = self.create_subscription(
            Float32,
            '/gripper/width',
            self._width_callback,
            10
        )

        # ── Services ─────────────────────────────────────────────────────
        self._open_srv = self.create_service(
            Trigger,
            '/gripper/open',
            self._open_service_callback
        )

        self._close_srv = self.create_service(
            Trigger,
            '/gripper/close',
            self._close_service_callback
        )

        # ── Initial state: open gripper ──────────────────────────────────
        # Wait briefly for publishers to connect, then open
        self._init_done = False
        self._init_timer = self.create_timer(1.0, self._initial_open)

        self.get_logger().info('Gripper Controller Node ready.')
        self.get_logger().info(f'  Publish to /gripper/command (0=close, 1=open)')
        self.get_logger().info(f'  Publish to /gripper/width (meters)')
        self.get_logger().info(f'  Call /gripper/open or /gripper/close service')

    def _initial_open(self):
        """Send open command on startup (called once via timer)."""
        if self._init_done:
            return
        self._init_done = True
        self.open_gripper()
        # Cancel the repeating timer after first successful call
        self._init_timer.cancel()

    def open_gripper(self) -> bool:
        """Open the gripper to maximum position."""
        self.get_logger().info('Gripper: OPEN')
        self._current_position = self.open_pos
        self._is_open = True
        self._publish_command(self.open_pos)
        return True

    def close_gripper(self) -> bool:
        """Close the gripper to minimum position (grasp)."""
        self.get_logger().info('Gripper: CLOSE')
        self._current_position = self.close_pos
        self._is_open = False
        self._publish_command(self.close_pos)
        return True

    def set_width(self, width_meters: float) -> bool:
        """
        Set gripper to a specific total gap width.

        Args:
            width_meters: Total gap between fingers (0.0 to 0.08 m)

        Returns:
            True if command was sent
        """
        # Each finger moves half the total gap
        finger_pos = width_meters / 2.0
        finger_pos = max(self.MIN_OPENING, min(self.MAX_OPENING, finger_pos))

        self.get_logger().info(f'Gripper width: {width_meters*1000:.1f}mm '
                               f'(finger pos: {finger_pos*1000:.1f}mm)')

        self._current_position = finger_pos
        self._is_open = (finger_pos > self.MIN_OPENING)
        self._publish_command(finger_pos)
        return True

    def _publish_command(self, finger_position: float):
        """
        Publish position command to ros2_control.

        Args:
            finger_position: Position for each finger (0.0 to 0.04 m)
        """
        msg = Float64MultiArray()
        # [left_finger_joint, right_finger_joint]
        msg.data = [finger_position, finger_position]
        self._cmd_pub.publish(msg)

    def _command_callback(self, msg: Float32):
        """Handle simple 0/1 open/close commands."""
        if msg.data >= 0.5:
            self.open_gripper()
        else:
            self.close_gripper()

    def _width_callback(self, msg: Float32):
        """Handle width commands in meters."""
        self.set_width(msg.data)

    def _open_service_callback(self, request, response):
        """Service handler: open gripper."""
        success = self.open_gripper()
        response.success = success
        response.message = 'Gripper opened' if success else 'Failed to open gripper'
        return response

    def _close_service_callback(self, request, response):
        """Service handler: close gripper."""
        success = self.close_gripper()
        response.success = success
        response.message = 'Gripper closed' if success else 'Failed to close gripper'
        return response

    @property
    def is_open(self) -> bool:
        """Return True if gripper is open."""
        return self._is_open

    @property
    def current_position(self) -> float:
        """Return current finger position in meters."""
        return self._current_position


def main(args=None):
    rclpy.init(args=args)
    node = GripperControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
