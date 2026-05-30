#!/usr/bin/env python3
# ============================================================
# arm_vision/arm_vision/coord_transformer.py
#
# Coordinate Transformation Node.
#
# Converts detected object 3D coordinates from camera frame
# to world/robot-base frame using TF2 transforms.
#
# PIPELINE:
#   YOLO detects object at (x, y, z) in camera_optical_frame
#   -> TF2 transform: camera_optical_frame -> world
#   -> Object position in world frame published
#   -> pick_place_node uses world-frame position for IK
#
# SUBSCRIBED TOPICS:
#   /detected_object_pose  (geometry_msgs/PoseStamped, camera frame)
#
# PUBLISHED TOPICS:
#   /arm/target_pose       (geometry_msgs/PoseStamped, world frame)
#   /arm/target_point      (geometry_msgs/PointStamped, world frame)
#
# PARAMETERS:
#   target_frame: Frame to transform into (default: 'world')
#   source_frame: Frame poses come from (default: 'camera_optical_frame')
#   table_z_correction: Override Z with known table height (m)
# ============================================================

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException


class CoordTransformerNode(Node):
    """
    Transforms object poses from camera frame to world frame.

    Upgraded to support:
      - Custom QoS (Reliable, Transient Local, History Depth 10)
      - Continuous 5Hz republishing until confirmation receipt
      - Startup synchronization (do not publish until subscribers exist)
      - Explicit receipt confirmation via /arm/target_pose/ack
    """

    def __init__(self):
        super().__init__('coord_transformer_node')

        self.get_logger().info('=== Coordinate Transformer Node Starting ===')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('source_frame', 'camera_optical_frame')
        self.declare_parameter('table_z', 0.75)     # Known table height
        self.declare_parameter('use_table_z', True)  # Override z with table height
        self.declare_parameter('z_offset', 0.06)    # Object half-height above table

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.table_z = self.get_parameter('table_z').value
        self.use_table_z = self.get_parameter('use_table_z').value
        self.z_offset = self.get_parameter('z_offset').value

        # ── Pipeline Synchronization and Republishing Control ───────────
        self._latest_target_pose = None
        self._receipt_confirmed = False
        self._last_busy_state = 'false' # Start 'false' to prevent premature startup reset

        # ── TF2 setup ────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ──────────────────────────────────────────────────
        self._pose_sub = self.create_subscription(
            PoseStamped,
            '/detected_object_pose',
            self._pose_callback,
            10
        )

        # Subscribe to pick_place busy flag to know when to reset
        self._busy_sub = self.create_subscription(
            String,
            '/pick_place/busy',
            self._busy_callback,
            10
        )

        # Target Pose QoS configuration (Reliable, Transient Local, Depth 10)
        self.target_pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to acknowledgment topic for receipt confirmation
        self._ack_sub = self.create_subscription(
            String,
            '/arm/target_pose/ack',
            self._ack_callback,
            self.target_pose_qos
        )

        # ── Publishers ───────────────────────────────────────────────────
        self._world_pose_pub = self.create_publisher(
            PoseStamped,
            '/arm/target_pose',
            self.target_pose_qos
        )

        self._status_pub = self.create_publisher(
            String, '/tf_transform/status', 10
        )

        # ── Timer for Continuous Republishing ───────────────────────────
        timer_period = 0.2  # 5Hz
        self._publish_timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f'Transforming from {self.source_frame} -> {self.target_frame}'
        )
        self.get_logger().info('Continuous mode: will republish target pose at 5Hz until confirmation')

    def _busy_callback(self, msg: String):
        """
        When pick_place transitions from busy to IDLE, reset synchronization state
        so we can receive and publish a new target pose.
        """
        current_busy = msg.data
        if self._last_busy_state == 'true' and current_busy == 'false':
            self._latest_target_pose = None
            self._receipt_confirmed = False
            self.get_logger().info('Pick-place IDLE — reset target pose and confirmation state.')
        
        self._last_busy_state = current_busy

    def _ack_callback(self, msg: String):
        """
        Callback when receipt acknowledgment is received from pick_place_node.
        """
        if msg.data == 'received' and not self._receipt_confirmed:
            self._receipt_confirmed = True
            self.get_logger().info('✔ Receipt acknowledgment received from pick_place_node! Stopping 5Hz republishing.')

    def _pose_callback(self, msg: PoseStamped):
        """
        Transform pose from camera frame to world frame.
        Saves target pose for continuous timer-based republishing.
        """
        # Only take a new target if we don't have a confirmed or currently active one
        if self._latest_target_pose is not None or self._receipt_confirmed:
            return

        try:
            # Wait for transform to be available
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,  # source: camera_optical_frame
                rclpy.time.Time(),    # use latest available transform
                timeout=Duration(seconds=1.0)
            )
        except TransformException as e:
            self.get_logger().warn(
                f'Transform lookup failed '
                f'({msg.header.frame_id} -> {self.target_frame}): {e}',
                throttle_duration_sec=2.0
            )
            return

        # Apply transform to the pose
        try:
            world_pose = tf2_geometry_msgs.do_transform_pose(
                msg.pose, transform
            )
        except Exception as e:
            self.get_logger().warn(f'Transform failed: {e}')
            return

        # Build output message
        world_pose_stamped = PoseStamped()
        world_pose_stamped.header.frame_id = self.target_frame
        world_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        world_pose_stamped.pose = world_pose

        # Override Z with known table height if requested
        if self.use_table_z:
            world_pose_stamped.pose.position.z = (
                self.table_z + self.z_offset
            )

        # Keep orientation as identity (gripper always points down)
        world_pose_stamped.pose.orientation.x = 0.0
        world_pose_stamped.pose.orientation.y = 0.0
        world_pose_stamped.pose.orientation.z = 0.0
        world_pose_stamped.pose.orientation.w = 1.0

        # Save the pose to be continuously published by the timer
        self._latest_target_pose = world_pose_stamped

        self.get_logger().info(
            f'★ Object detected & transformed: '
            f'({world_pose_stamped.pose.position.x:.3f}, '
            f'{world_pose_stamped.pose.position.y:.3f}, '
            f'{world_pose_stamped.pose.position.z:.3f}) '
            f'in frame: {world_pose_stamped.header.frame_id}'
        )

    def _timer_callback(self):
        """
        Timer loop executing at 5Hz to republish target pose until receipt is confirmed.
        Includes startup synchronization to ensure active subscribers are present.
        """
        if self._latest_target_pose is None or self._receipt_confirmed:
            return

        # ── Startup Synchronization Wait ──
        # Do not publish target pose until a subscriber has registered to /arm/target_pose
        sub_count = self._world_pose_pub.get_subscription_count()
        if sub_count == 0:
            self.get_logger().warn(
                'Waiting for subscriber on /arm/target_pose before publishing...',
                throttle_duration_sec=3.0
            )
            return

        # Update message timestamp to current time for planner freshness
        self._latest_target_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Publish
        self._world_pose_pub.publish(self._latest_target_pose)
        self.get_logger().info(
            f'Republishing target pose at 5Hz (Subscribers: {sub_count}): '
            f'({self._latest_target_pose.pose.position.x:.3f}, '
            f'{self._latest_target_pose.pose.position.y:.3f}, '
            f'{self._latest_target_pose.pose.position.z:.3f})',
            throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = CoordTransformerNode()
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
