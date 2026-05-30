#!/usr/bin/env python3
# ============================================================
# arm_vision/arm_vision/camera_tf_broadcaster.py
#
# Camera TF Broadcaster Node.
#
# For a USB/real camera (not Gazebo simulated camera),
# this node broadcasts the static TF transform from
# robot base (base_link) to camera frame (camera_link).
#
# For Gazebo simulation: the TF is already published via
# robot_state_publisher from the URDF fixed joints.
# This node is only needed for REAL hardware setups.
#
# PUBLISHED TF:
#   world -> camera_mount_frame (static broadcast)
#
# PARAMETERS:
#   camera_frame: TF frame name for camera
#   parent_frame: Parent frame (default: base_link)
#   x, y, z: Camera position relative to parent (meters)
#   roll, pitch, yaw: Camera orientation relative to parent (radians)
#   is_wrist_mounted: If true, camera moves with robot (already in URDF)
# ============================================================

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import math


class CameraTFBroadcasterNode(Node):
    """
    Broadcasts static TF from robot frame to camera frame.
    Used for external/overhead cameras not in the URDF.
    """

    def __init__(self):
        super().__init__('camera_tf_broadcaster_node')

        self.get_logger().info('=== Camera TF Broadcaster Node Starting ===')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('camera_frame', 'overhead_camera_link')
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('x', 0.5)     # 0.5m in front of robot
        self.declare_parameter('y', 0.0)     # centered
        self.declare_parameter('z', 1.2)     # 1.2m above ground (overhead)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 1.5708)   # pi/2 = pointing down
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('is_wrist_mounted', True)  # already in URDF

        self.camera_frame = self.get_parameter('camera_frame').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.z = self.get_parameter('z').value
        self.roll = self.get_parameter('roll').value
        self.pitch = self.get_parameter('pitch').value
        self.yaw = self.get_parameter('yaw').value
        self.is_wrist = self.get_parameter('is_wrist_mounted').value

        # ── TF Broadcaster ───────────────────────────────────────────────
        if not self.is_wrist:
            # Only broadcast for non-URDF cameras
            self._static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
            self._broadcast_static_tf()
            self.get_logger().info(
                f'Broadcasting static TF: '
                f'{self.parent_frame} -> {self.camera_frame}'
            )
        else:
            self.get_logger().info(
                'Wrist-mounted camera: TF published via robot_state_publisher'
            )
            self.get_logger().info(
                'Camera TF chain: world -> ... -> link6 -> camera_link -> camera_optical_frame'
            )

        # ── Optical frame broadcaster ─────────────────────────────────────
        # Always broadcast the optical frame transform
        # (ROS convention: Z forward, X right, Y down)
        self._opt_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._broadcast_optical_tf()

        self.get_logger().info('Camera TF Broadcaster ready.')

    def _broadcast_static_tf(self):
        """Broadcast static transform for overhead camera."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.camera_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z

        # Convert RPY to quaternion
        q = self._rpy_to_quat(self.roll, self.pitch, self.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self._static_broadcaster.sendTransform(t)

    def _broadcast_optical_tf(self):
        """
        Broadcast the transform from camera body frame to optical frame.
        Standard ROS convention:
          camera_link: Z=forward, X=right, Y=down (for Gazebo)
          camera_optical_frame: Z=forward, X=right, Y=down (for ROS image)

        For a camera body frame (X=forward):
          Rotation: -90° around Z, then -90° around X
          RPY: (-pi/2, 0, -pi/2)
        """
        # This is only for overhead cameras — wrist camera optical frame
        # is already defined in the URDF
        if not self.is_wrist:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.camera_frame
            t.child_frame_id = self.camera_frame.replace('_link', '_optical_frame')

            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0

            # Standard camera -> optical frame rotation
            q = self._rpy_to_quat(-math.pi/2, 0, -math.pi/2)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self._opt_broadcaster.sendTransform(t)

    def _rpy_to_quat(self, roll: float, pitch: float, yaw: float):
        """Convert roll-pitch-yaw to quaternion [x, y, z, w]."""
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = CameraTFBroadcasterNode()
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
