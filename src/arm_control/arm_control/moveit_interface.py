#!/usr/bin/env python3
# ============================================================
# arm_control/arm_control/moveit_interface.py
#
# MoveIt2 Python interface wrapper using pure ROS2 actions/services.
# Avoids dependency on compiled C++ moveit_py bindings.
# ============================================================

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import numpy as np

# MoveGroup actions and planning services
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    PlanningOptions,
    RobotState
)
from shape_msgs.msg import SolidPrimitive

# Geometry messages
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header

# Control messages for gripper
from control_msgs.action import GripperCommand

# scipy for rotation math
try:
    from scipy.spatial.transform import Rotation
except ImportError:
    import subprocess
    subprocess.run(['pip3', 'install', 'scipy'], check=True)
    from scipy.spatial.transform import Rotation


class ArmMoveItInterface:
    """
    High-level MoveIt2 interface for the 6-DOF arm using ROS2 action client.
    """

    def __init__(self, node: Node):
        self.parent_node = node

        # Get parent node's use_sim_time parameter to ensure proper time sync
        use_sim_time = False
        try:
            use_sim_time = node.get_parameter('use_sim_time').value
        except Exception:
            pass

        from rclpy.parameter import Parameter
        # Create an isolated node for synchronous action client spinning.
        # Set use_global_arguments=False to prevent node name collisions in the ROS2 graph.
        self.node = rclpy.create_node(
            'moveit_interface_node',
            use_global_arguments=False,
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)
            ]
        )
        self.logger = self.node.get_logger()

        # ── Joint State Subscriber ──────────────────────────────────────
        self.logger.info('Subscribing to /joint_states...')
        from sensor_msgs.msg import JointState
        self._joint_state = None
        self._joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
        )

        # ── Initialize ROS2 clients ─────────────────────────────────────
        self.logger.info('Initializing Action Client for /move_action...')
        self._action_client = ActionClient(self.node, MoveGroup, 'move_action')

        self.logger.info('Initializing Action Client for /execute_trajectory...')
        self._execute_client = ActionClient(self.node, ExecuteTrajectory, 'execute_trajectory')

        self.logger.info('Initializing Service Client for /apply_planning_scene...')
        self._scene_client = self.node.create_client(ApplyPlanningScene, 'apply_planning_scene')

        self.logger.info('Initializing Service Client for /compute_cartesian_path...')
        self._cartesian_client = self.node.create_client(
            GetCartesianPath, 'compute_cartesian_path'
        )
        
        self.logger.info('Initializing Service Client for /get_planning_scene...')
        from moveit_msgs.srv import GetPlanningScene
        self._get_scene_client = self.node.create_client(GetPlanningScene, 'get_planning_scene')

        # ── Gripper publisher ───────────────────────────────────────────
        self._gripper_pub = node.create_publisher(
            __import__('std_msgs.msg', fromlist=['Float64MultiArray']).Float64MultiArray,
            '/gripper_controller/commands',
            10
        )

        self.logger.info('MoveGroup Action interface ready.')

    def _joint_state_cb(self, msg):
        """Callback to store the current joint state."""
        self._joint_state = msg

    def go_home(self) -> bool:
        """Move arm to home position (elbow-up/wrist-down)."""
        self.logger.info('Moving to home position...')
        home_joints = {
            'joint1': 0.0,
            'joint2': -0.55,
            'joint3': 1.10,
            'joint4': 0.0,
            'joint5': 0.45,
            'joint6': 0.0
        }
        return self.move_to_joints(home_joints)

    def go_ready(self) -> bool:
        """Move arm to ready position."""
        self.logger.info('Moving to ready position...')
        ready_joints = {
            'joint1': 0.0,
            'joint2': -0.5236,
            'joint3': 1.0472,
            'joint4': 0.0,
            'joint5': -0.5236,
            'joint6': 0.0
        }
        return self.move_to_joints(ready_joints)

    def go_folded(self) -> bool:
        """Move arm to folded position."""
        self.logger.info('Moving to folded position...')
        folded_joints = {
            'joint1': 0.0,
            'joint2': 1.5708,
            'joint3': -2.0944,
            'joint4': 0.0,
            'joint5': -1.5708,
            'joint6': 0.0
        }
        return self.move_to_joints(folded_joints)

    def move_to_joints(self, joint_map: dict) -> bool:
        """Move arm to specific joint positions."""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3

        constraints = Constraints()
        for name, val in joint_map.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.request.start_state.is_diff = True

        return self._send_and_execute_goal(goal_msg)

    def _get_grasp_quaternion(self):
        """
        Quaternion for a downward-facing grasp (gripper Z-axis pointing down).
        roll=pi about X: (w=0, x=1, y=0, z=0)
        """
        return 0.0, 1.0, 0.0, 0.0

    def move_to_pose(
        self,
        x: float, y: float, z: float,
        roll: float = 3.14159, pitch: float = 0.0, yaw: float = 0.0,
        frame_id: str = 'world',
        velocity_scale: float = 1.0,
        acceleration_scale: float = 1.0,
        constrain_orientation: bool = False
    ) -> bool:
        """
        Move end-effector (tool0) to a Cartesian pose.

        When constrain_orientation=True, a path constraint is added to keep
        the gripper pointing downward throughout the entire trajectory,
        preventing wrist flipping.
        """
        qx, qy, qz, qw = self._get_grasp_quaternion()
        self.logger.info(
            f'Planning path to pose: ({x:.3f}, {y:.3f}, {z:.3f}) '
            f'quat=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}) '
            f'vel={velocity_scale:.2f}'
        )

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 8.0
        goal_msg.request.max_velocity_scaling_factor = velocity_scale
        goal_msg.request.max_acceleration_scaling_factor = acceleration_scale

        # ── Goal Constraints: position only ─────────────────────────────
        # No orientation constraint — OMPL plans freely, and orientation
        # is enforced by the IK solution at the Cartesian waypoint stage.
        constraints = Constraints()

        # Position Constraint
        pos_con = PositionConstraint()
        pos_con.header.frame_id = frame_id
        pos_con.link_name = 'tool0'
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.015, 0.015, 0.015]  # 1.5 cm goal tolerance
        pos_con.constraint_region.primitives.append(primitive)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        pos_con.constraint_region.primitive_poses.append(pose)
        constraints.position_constraints.append(pos_con)

        # Orientation Constraint
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = frame_id
        ori_con.link_name = 'tool0'
        
        # Use scipy instead of tf_transformations to avoid the broken
        # transforms3d dependency that references the removed np.float alias.
        _r = Rotation.from_euler('xyz', [roll, pitch, yaw])
        q = _r.as_quat()  # returns [x, y, z, w]
        ori_con.orientation.x = float(q[0])
        ori_con.orientation.y = float(q[1])
        ori_con.orientation.z = float(q[2])
        ori_con.orientation.w = float(q[3])
        
        ori_con.absolute_x_axis_tolerance = 0.05
        ori_con.absolute_y_axis_tolerance = 0.05
        ori_con.absolute_z_axis_tolerance = 0.05
        ori_con.weight = 1.0
        constraints.orientation_constraints.append(ori_con)

        goal_msg.request.goal_constraints.append(constraints)

        # ── Path Constraints: lock orientation during entire trajectory ───
        if constrain_orientation:
            path_constraints = Constraints()
            path_ori = OrientationConstraint()
            path_ori.header.frame_id = frame_id
            path_ori.link_name = 'tool0'
            path_ori.orientation.x = qx
            path_ori.orientation.y = qy
            path_ori.orientation.z = qz
            path_ori.orientation.w = qw
            # Relaxed tolerance for path
            path_ori.absolute_x_axis_tolerance = 1.0
            path_ori.absolute_y_axis_tolerance = 1.0
            path_ori.absolute_z_axis_tolerance = 1.0
            path_ori.weight = 1.0
            path_constraints.orientation_constraints.append(path_ori)
            goal_msg.request.path_constraints = path_constraints
            self.logger.info('  Path constraint: orientation locked (downward)')

        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.request.start_state.is_diff = True

        return self._send_and_execute_goal(goal_msg)

    def move_cartesian(
        self,
        x: float, y: float, z: float,
        roll: float = 3.14159, pitch: float = 0.0, yaw: float = 0.0,
        frame_id: str = 'world',
        max_step: float = 0.01,
        velocity_scale: float = 0.03,
        acceleration_scale: float = 0.03,
        avoid_collisions: bool = False
    ) -> bool:
        """
        Move tool0 in a straight line (Cartesian path) to the target pose.
        Used for final grasp approach to ensure precise straight-line descent.
        Orientation is locked to the target quaternion throughout.

        Falls back to regular move_to_pose if the Cartesian path service
        is unavailable or the path fraction is too low.
        """
        qx, qy, qz, qw = self._get_grasp_quaternion()
        self.logger.info(
            f'Cartesian path to: ({x:.3f}, {y:.3f}, {z:.3f}) '
            f'quat=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}) '
            f'step={max_step:.4f}m vel={velocity_scale:.2f}'
        )

        if not self._cartesian_client.wait_for_service(timeout_sec=3.0):
            self.logger.error('Cartesian path service not available')
            return False

        # Build target waypoint with locked orientation
        waypoint = Pose()
        waypoint.position.x = x
        waypoint.position.y = y
        waypoint.position.z = z
        waypoint.orientation.x = qx
        waypoint.orientation.y = qy
        waypoint.orientation.z = qz
        waypoint.orientation.w = qw

        # No path orientation constraints — the waypoint already carries the
        # correct orientation (qx=1, roll=π downward). Adding a path constraint
        # causes the trajectory post-processing validator to reject valid plans
        # when intermediate IK solutions cannot perfectly maintain orientation.

        # Ensure joint state is populated
        if self._joint_state is None:
            self.logger.info('Waiting for joint state to be populated...')
            import time
            start_t = time.time()
            while self._joint_state is None and (time.time() - start_t) < 5.0:
                time.sleep(0.05)

        req = GetCartesianPath.Request()
        req.header.frame_id = frame_id
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.group_name = 'arm'
        req.link_name = 'tool0'
        req.waypoints = [waypoint]
        req.max_step = max_step
        req.avoid_collisions = avoid_collisions
        req.start_state.is_diff = True

        future = self._cartesian_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=5.0):
            self.logger.error('Cartesian path computation timed out')
            return False

        result = future.result()
        if result is None:
            return False

        fraction = result.fraction
        self.logger.info(f'Cartesian path fraction: {fraction:.2f}')

        if fraction < 0.80:
            self.logger.warn(
                f'Low Cartesian path fraction ({fraction:.2f}). '
                f'Falling back to joint-space move_to_pose.'
            )
            return self.move_to_pose(
                x, y, z, roll, pitch, yaw,
                frame_id=frame_id,
                velocity_scale=velocity_scale,
                acceleration_scale=acceleration_scale,
                constrain_orientation=False
            )

        # Scale trajectory velocity
        trajectory = result.solution
        self._scale_trajectory_velocity(trajectory, velocity_scale, acceleration_scale)

        num_points = len(trajectory.joint_trajectory.points)
        self.logger.info(f"Cartesian trajectory computed with {num_points} points")
        
        # Execute trajectory via ExecuteTrajectory action
        return self._execute_trajectory(trajectory)

    def _scale_trajectory_velocity(
        self, trajectory, velocity_scale: float, acceleration_scale: float = 0.1
    ):
        """
        Scale trajectory timing to slow down execution.
        Stretches time_from_start and scales velocities/accelerations.
        """
        if velocity_scale <= 0.0 or velocity_scale >= 1.0:
            velocity_scale = 1.0
        if acceleration_scale <= 0.0 or acceleration_scale >= 1.0:
            acceleration_scale = 1.0

        scale = min(velocity_scale, acceleration_scale)
        if scale >= 1.0:
            return

        time_stretch = 1.0 / scale  # e.g. 0.08 -> stretch 12.5x

        for point in trajectory.joint_trajectory.points:
            # Convert to total nanoseconds (Python int, no overflow risk)
            total_ns = (
                int(point.time_from_start.sec) * 1_000_000_000
                + int(point.time_from_start.nanosec)
            )
            # Stretch the time
            total_ns = int(total_ns * time_stretch)
            # Split back into sec + nanosec (both within valid ranges)
            point.time_from_start.sec = int(total_ns // 1_000_000_000)
            point.time_from_start.nanosec = int(total_ns % 1_000_000_000)

            # Scale velocities and accelerations
            if point.velocities:
                point.velocities = [
                    v / time_stretch for v in point.velocities
                ]
            if point.accelerations:
                point.accelerations = [
                    a / (time_stretch * time_stretch)
                    for a in point.accelerations
                ]

    def _execute_trajectory(self, trajectory) -> bool:
        """Execute a pre-planned trajectory via the ExecuteTrajectory action."""
        self.logger.info("Sending Cartesian trajectory")
        if not self._execute_client.wait_for_server(timeout_sec=5.0):
            self.logger.error('ExecuteTrajectory action server not available!')
            return False

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory

        send_goal_future = self._execute_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_goal_future, timeout_sec=5.0):
            self.logger.error("Send goal future did not complete")
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.logger.error('ExecuteTrajectory goal rejected.')
            return False

        self.logger.info("Cartesian execution ACCEPTED")
        self.logger.info("Waiting for Cartesian execution result")
        
        get_result_future = goal_handle.get_result_async()
        if not self._wait_for_future(get_result_future, timeout_sec=300.0):
            self.logger.error("Cartesian execution exceeded 300 seconds timeout")
            return False

        result_msg = get_result_future.result()
        if result_msg and result_msg.result.error_code.val == 1:
            self.logger.info("Cartesian trajectory SUCCESS")
            return True
        else:
            code = (
                result_msg.result.error_code.val
                if result_msg else 'UNKNOWN'
            )
            self.logger.error(
                f'Cartesian trajectory execution failed with code: {code}'
            )
            return False

    def _send_and_execute_goal(self, goal_msg) -> bool:
        """Internal helper to send action goal and block until finished."""
        self.logger.info("Sending MoveGroup goal...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.logger.error("Action server /move_action not available!")
            return False

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        self.logger.info("Waiting for MoveGroup goal acceptance...")
        if not self._wait_for_future(send_goal_future, timeout_sec=10.0):
            self.logger.error("Send goal future did not complete")
            return False
            
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.logger.error("MoveGroup goal rejected.")
            return False

        self.logger.info("MoveGroup goal ACCEPTED")
        self.logger.info("Waiting for trajectory result...")
        
        get_result_future = goal_handle.get_result_async()
        if not self._wait_for_future(get_result_future, timeout_sec=60.0):
            self.logger.error("Get result future did not complete")
            return False
            
        result_msg = get_result_future.result()
        if result_msg and result_msg.result.error_code.val == 1:
            self.logger.info("Trajectory execution SUCCESS")
            return True
        else:
            code = result_msg.result.error_code.val if result_msg else "UNKNOWN"
            self.logger.error(f"Execution failed with code: {code}")
            return False

    def open_gripper(self, opening: float = 0.04) -> bool:
        """Open gripper to specified total width."""
        self.logger.info(f'Opening gripper to {opening*1000:.0f}mm...')
        finger_pos = opening / 2.0
        from std_msgs.msg import Float64MultiArray
        msg = Float64MultiArray()
        msg.data = [finger_pos, finger_pos]
        self._gripper_pub.publish(msg)
        return True

    def close_gripper(self, force: float = 10.0) -> bool:
        """Close gripper fully."""
        self.logger.info('Closing gripper...')
        from std_msgs.msg import Float64MultiArray
        msg = Float64MultiArray()
        msg.data = [0.018, 0.018]
        self._gripper_pub.publish(msg)
        return True

    def set_gripper_width(self, width: float) -> bool:
        """Set finger gap. width=0.0 means fully closed (maximum grip force)."""
        finger_pos = width / 2.0
        finger_pos = max(0.0, min(0.04, finger_pos))  # Clamp to joint limits
        from std_msgs.msg import Float64MultiArray
        msg = Float64MultiArray()
        msg.data = [finger_pos, finger_pos]
        self._gripper_pub.publish(msg)
        return True

    def add_collision_box(
        self,
        name: str,
        size: tuple,
        position: tuple,
        frame_id: str = 'world'
    ) -> bool:
        """Add collision box via ApplyPlanningScene service."""
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive

        co = CollisionObject()
        co.id = name
        co.header.frame_id = frame_id
        co.header.stamp = self.node.get_clock().now().to_msg()

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)
        co.primitives.append(box)

        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = 1.0
        co.primitive_poses.append(pose)

        co.operation = CollisionObject.ADD
        return self._apply_scene_diff(co)

    def remove_collision_object(self, name: str) -> bool:
        """Remove collision object."""
        from moveit_msgs.msg import CollisionObject
        co = CollisionObject()
        co.id = name
        co.operation = CollisionObject.REMOVE
        return self._apply_scene_diff(co)

    def attach_object(self, object_name: str, link_name: str = 'tool0') -> bool:
        """
        Attach a world collision object to a robot link.

        Critically: the object must be REMOVED from world.collision_objects
        at the same time it is added to robot_state.attached_collision_objects.
        If both coexist, MoveIt sees the attached geometry intersecting the world
        geometry at the same pose, causing immediate collision that blocks planning.
        """
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

        # 1. Fetch the current object geometry from the planning scene so we
        #    can include it in the AttachedCollisionObject (required by MoveIt).
        from moveit_msgs.srv import GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents
        import time

        obj_shapes = []
        obj_poses = []
        if self._get_scene_client.wait_for_service(timeout_sec=3.0):
            get_req = GetPlanningScene.Request()
            get_req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            future = self._get_scene_client.call_async(get_req)
            t0 = time.time()
            while not future.done() and (time.time() - t0) < 3.0:
                time.sleep(0.01)
            if future.done() and future.result():
                for co in future.result().scene.world.collision_objects:
                    if co.id == object_name:
                        obj_shapes = co.primitives
                        obj_poses = co.primitive_poses
                        break

        # 2. Build the AttachedCollisionObject
        aco = AttachedCollisionObject()
        aco.link_name = link_name
        aco.object.id = object_name
        aco.object.header.frame_id = link_name
        aco.object.operation = CollisionObject.ADD
        # Copy geometry if available (lets MoveIt track the attached body)
        if obj_shapes:
            aco.object.primitives = obj_shapes
            # Use identity pose (object is now at link origin offset by grasp)
            from geometry_msgs.msg import Pose
            p = Pose()
            p.orientation.w = 1.0
            aco.object.primitive_poses = [p]
        # Allow the attached cube to touch the gripper links without collision
        aco.touch_links = [
            'left_finger_link', 'right_finger_link',
            'gripper_base_link', 'tool0'
        ]

        # 3. Build the world CollisionObject REMOVE message
        world_co = CollisionObject()
        world_co.id = object_name
        world_co.operation = CollisionObject.REMOVE

        # 4. Apply both atomically in one scene diff:
        #    - Remove from world
        #    - Add to robot state
        req = ApplyPlanningScene.Request()
        req.scene.world.collision_objects.append(world_co)
        req.scene.robot_state.attached_collision_objects.append(aco)
        req.scene.robot_state.is_diff = True
        req.scene.is_diff = True
        self.logger.info(f'Attaching {object_name} to {link_name} (removing from world)')
        return self._call_scene_service(req)

    def detach_object(self, object_name: str) -> bool:
        """Detach object from end effector and remove from scene."""
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
        aco = AttachedCollisionObject()
        aco.object.id = object_name
        aco.object.operation = CollisionObject.REMOVE

        req = ApplyPlanningScene.Request()
        req.scene.robot_state.attached_collision_objects.append(aco)
        req.scene.robot_state.is_diff = True
        req.scene.is_diff = True
        return self._call_scene_service(req)

    def _apply_scene_diff(self, co) -> bool:
        """Helper to apply a collision object diff to planning scene."""
        req = ApplyPlanningScene.Request()
        req.scene.world.collision_objects.append(co)
        req.scene.is_diff = True
        return self._call_scene_service(req)

    def allow_collision(self, object_name: str, link_names: list) -> bool:
        """Add AllowedCollisionMatrix entries to allow object to touch links safely."""
        from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents, AllowedCollisionEntry
        import time
        
        if not self._get_scene_client.wait_for_service(timeout_sec=5.0):
            self.logger.error("Service /get_planning_scene not available!")
            return False
            
        # 1. Fetch current ACM
        get_req = GetPlanningScene.Request()
        get_req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self._get_scene_client.call_async(get_req)
        while not future.done() and rclpy.ok():
            time.sleep(0.01)
        res = future.result()
        if not res:
            return False
            
        acm = res.scene.allowed_collision_matrix
        
        # 2. Add object if not present
        if object_name not in acm.entry_names:
            acm.entry_names.append(object_name)
            # Add a new row (AllowedCollisionEntry) for the new object
            new_entry = AllowedCollisionEntry()
            new_entry.enabled = [False] * len(acm.entry_names)
            acm.entry_values.append(new_entry)
            # Add a new column (False) to all existing rows
            for entry in acm.entry_values[:-1]:
                entry.enabled.append(False)
                
        obj_idx = acm.entry_names.index(object_name)
        
        # 3. Enable collision between object and specified links
        for link in link_names:
            if link in acm.entry_names:
                link_idx = acm.entry_names.index(link)
                acm.entry_values[obj_idx].enabled[link_idx] = True
                acm.entry_values[link_idx].enabled[obj_idx] = True
            else:
                self.logger.warn(f"Link {link} not found in ACM!")
                
        # 4. Apply updated ACM
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.allowed_collision_matrix = acm
        return self._call_scene_service(req)

    def _call_scene_service(self, req) -> bool:
        """Call apply_planning_scene service synchronously."""
        if not self._scene_client.wait_for_service(timeout_sec=5.0):
            self.logger.error("Service /apply_planning_scene not available!")
            return False

        future = self._scene_client.call_async(req)
        import time
        while not future.done() and rclpy.ok():
            time.sleep(0.01)
        res = future.result()
        return res is not None and res.success

    def _wait_for_future(self, future, timeout_sec=None) -> bool:
        """
        Wait for a future to complete by checking its status.
        Since the node is spun by a MultiThreadedExecutor in the main thread,
        we do NOT call spin_until_future_complete (which would conflict and crash).
        Instead, we just sleep and let the executor handle the callbacks.
        """
        import time
        start_t = time.time()
        while not future.done() and rclpy.ok():
            if timeout_sec is not None and (time.time() - start_t) > timeout_sec:
                return False
            time.sleep(0.01)
        return future.done()
