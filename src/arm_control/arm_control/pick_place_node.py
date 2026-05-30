#!/usr/bin/env python3
# ============================================================
# arm_control/arm_control/pick_place_node.py
#
# Complete Pick-and-Place pipeline node with state machine.
#
# PIPELINE:
#   1. Subscribe to /arm/target_pose (from vision pipeline)
#   2. Latch FIRST valid detection, ignore all subsequent
#   3. Execute pick-and-place state machine
#   4. Reset cleanly and wait for next trigger
#
# STATE MACHINE:
#   WAITING_FOR_TRIGGER -> MOVING_TO_PREGRASP -> DESCENDING -> GRASPING ->
#   LIFTING -> PLACING -> COMPLETE -> WAITING_FOR_TRIGGER
#   Any state -> FAILED -> WAITING_FOR_TRIGGER (on error/timeout)
#
# SUBSCRIBED TOPICS:
#   /arm/target_pose       (geometry_msgs/PoseStamped, world frame)
#   /pick_place/trigger    (std_msgs/String, "start" or "home")
#
# PUBLISHED TOPICS:
#   /arm/status            (std_msgs/String, state machine status)
#   /pick_place/busy       (std_msgs/String, "true"/"false")
#
# PARAMETERS:
#   place_x, place_y, place_z — target place position
#   approach_height  — how high above object to approach (m)
#   grasp_z_offset   — additional Z offset at grasp (m)
#   planning_timeout — max seconds per MoveIt plan (s)
# ============================================================

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import time
import threading
from enum import Enum, auto
from typing import Optional

from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import String, Float32, Float64MultiArray
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Import the MoveIt2 interface
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
from moveit_interface import ArmMoveItInterface


class PipelineState(Enum):
    """State machine states for pick-and-place."""
    WAITING_FOR_TRIGGER = auto()
    HOMING = auto()
    OPENING_GRIPPER = auto()
    MOVING_TO_PREGRASP = auto()
    DESCENDING = auto()
    FINAL_ALIGN = auto()
    TOUCHDOWN = auto()
    GRASPING = auto()
    LIFTING = auto()
    MOVING_TO_PLACE = auto()
    PLACING = auto()
    RELEASING = auto()
    RETRACTING = auto()
    RETURNING_HOME = auto()
    COMPLETE = auto()
    FAILED = auto()


class PickPlaceNode(Node):
    """
    Pick-and-Place pipeline orchestrator with state machine.

    Waits for a manual trigger before executing.
    Only caches the latest detection while waiting.
    """

    def __init__(self):
        super().__init__('pick_place_node')

        self.get_logger().info('=== Pick-and-Place Node Starting ===')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('place_x', 0.18)
        self.declare_parameter('place_y', -0.05)
        self.declare_parameter('place_z', 0.40)
        self.declare_parameter('approach_height', 0.25)
        self.declare_parameter('grasp_z_offset', 0.027)    # 0.352 - 0.325 = 0.027
        self.declare_parameter('grasp_x_offset', 0.0)
        self.declare_parameter('grasp_y_offset', 0.0)
        self.declare_parameter('use_moveit', True)
        self.declare_parameter('gripper_open_width', 0.10)
        self.declare_parameter('gripper_close_width', 0.068)   # 68mm command -> 48mm physical gap (2mm squeeze on 50mm cube)
        self.declare_parameter('slow_approach_height', 0.10)
        self.declare_parameter('final_align_height', 0.05)
        self.declare_parameter('planning_timeout', 30.0)
        # Auto-start delay: seconds after startup before auto-triggering pipeline.
        # Set to 0.0 to disable auto-start (manual trigger only).
        self.declare_parameter('auto_start_delay', 5.0)

        self.place_x = self.get_parameter('place_x').value
        self.place_y = self.get_parameter('place_y').value
        self.place_z = self.get_parameter('place_z').value
        self.approach_height = self.get_parameter('approach_height').value
        self.grasp_z_offset = self.get_parameter('grasp_z_offset').value
        self.grasp_x_offset = self.get_parameter('grasp_x_offset').value
        self.grasp_y_offset = self.get_parameter('grasp_y_offset').value
        self.use_moveit = self.get_parameter('use_moveit').value
        self.gripper_open_width = self.get_parameter('gripper_open_width').value
        self.gripper_close_width = self.get_parameter('gripper_close_width').value
        self.slow_approach_height = self.get_parameter('slow_approach_height').value
        self.final_align_height = self.get_parameter('final_align_height').value
        self.planning_timeout = self.get_parameter('planning_timeout').value
        self.auto_start_delay = self.get_parameter('auto_start_delay').value

        self.get_logger().info('=' * 60)
        if self.gripper_close_width == 0.0:
            self.get_logger().info(
                f'Gripper close width: {self.gripper_close_width*1000:.1f}mm '
                f'(FULL CLOSE — max grip force mode)'
            )
        elif 0.0 < self.gripper_close_width < 0.005:
            self.get_logger().warn(
                f'Gripper close width is very small: {self.gripper_close_width*1000:.1f}mm — '
                f'check gripper_close_width parameter'
            )
        else:
            self.get_logger().info(
                f'Gripper close width: {self.gripper_close_width*1000:.1f}mm'
            )
        self.get_logger().info('=' * 60)

        # ── State machine ────────────────────────────────────────────────
        self._state = PipelineState.WAITING_FOR_TRIGGER
        self._lock = threading.Lock()
        self._exec_thread: Optional[threading.Thread] = None
        self._grip_hold_active = False
        self._grip_hold_thread: Optional[threading.Thread] = None
        
        self.test_pick_pose = {
            "x": 0.30,
            "y": 0.10,
            "z": 0.325
        }

        self._last_log_time = 0.0

        # Callback group for concurrent execution
        self._cb_group = ReentrantCallbackGroup()

        # ── Publishers ───────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/arm/status', 10)
        self._marker_pub = self.create_publisher(Marker, 'grasp_marker', 10)
        self._busy_pub = self.create_publisher(String, '/pick_place/busy', 10)

        self._gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10
        )

        # Manual trigger.
        # Trigger with: ros2 topic pub -t 5 /pick_place/trigger std_msgs/msg/String "{data: 'start'}"
        # Use explicit RELIABLE QoS to match ros2 topic pub defaults exactly.
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        trigger_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._trigger_sub = self.create_subscription(
            String,
            '/pick_place/trigger',
            self._trigger_callback,
            trigger_qos,
            callback_group=self._cb_group
        )

        # ── Status timer (1 Hz) ──────────────────────────────────────────
        self._status_timer = self.create_timer(
            1.0, self._publish_status, callback_group=self._cb_group
        )

        # ── Initialize MoveIt2 Interface ──────────────────────────────────
        self.moveit = ArmMoveItInterface(self)

        # One-shot startup timer to add collision objects once spinning
        self._startup_timer = self.create_timer(
            1.5, self._on_startup, callback_group=self._cb_group
        )

        # Auto-start timer (wall clock, so it fires even with sim_time issues)
        self._auto_start_timer = None
        if self.auto_start_delay > 0.0:
            self._auto_start_timer = self.create_timer(
                self.auto_start_delay, self._auto_start_callback,
                callback_group=self._cb_group
            )

        self.get_logger().info('Pick-Place Node ready — state: WAITING_FOR_TRIGGER')
        self.get_logger().info(f'Place target: ({self.place_x}, {self.place_y}, {self.place_z})')
        self.get_logger().info(
            f'Auto-start in {self.auto_start_delay:.1f}s '
            f'(or publish "start" to /pick_place/trigger)'
        )
        print(f'[pick_place_node] READY — auto-start in {self.auto_start_delay:.1f}s', flush=True)

    # ── Startup ──────────────────────────────────────────────────────────

    def _on_startup(self):
        """Add collision objects to the planning scene once spinning."""
        self._startup_timer.destroy()
        # NOTE: We deliberately do NOT add the worktable as a MoveIt collision
        # object. The table caused link2 to appear in collision at the home
        # start state, which blocked every planning attempt.
        self.get_logger().info('Startup complete (no persistent collision objects added).')
        print('[pick_place_node] Startup complete — executor is spinning OK', flush=True)

    def _auto_start_callback(self):
        """Auto-trigger the pipeline once after the configured delay."""
        if self._auto_start_timer is not None:
            self._auto_start_timer.destroy()
            self._auto_start_timer = None

        self.get_logger().info(
            f'[AUTO-START] Triggering pipeline after {self.auto_start_delay:.1f}s delay'
        )
        print('[pick_place_node] AUTO-START firing now!', flush=True)

        with self._lock:
            if self._state != PipelineState.WAITING_FOR_TRIGGER:
                self.get_logger().warn(
                    f'Auto-start skipped — state={self._state.name}'
                )
                return
            self._state = PipelineState.HOMING

        self._start_pipeline_thread()

    # ── Status Publishing ────────────────────────────────────────────────

    def _publish_status(self):
        """Publish current state machine status."""
        msg = String()
        msg.data = self._state.name
        self._status_pub.publish(msg)

        busy_msg = String()
        busy_msg.data = 'true' if self._state != PipelineState.WAITING_FOR_TRIGGER else 'false'
        self._busy_pub.publish(busy_msg)



    # ── Manual Trigger ───────────────────────────────────────────────────

    def _trigger_callback(self, msg: String):
        """
        Manual trigger. Publish "start" to /pick_place/trigger.
        Reliable trigger cmd: ros2 topic pub -t 5 /pick_place/trigger std_msgs/msg/String "{data: 'start'}"
        """
        print(f'[pick_place_node] _trigger_callback fired: data="{msg.data}"', flush=True)
        self.get_logger().info(f'Trigger message received: "{msg.data}"')

        if msg.data == 'start':
            # Cancel auto-start timer if still pending
            if self._auto_start_timer is not None:
                self._auto_start_timer.destroy()
                self._auto_start_timer = None

            with self._lock:
                if self._state != PipelineState.WAITING_FOR_TRIGGER:
                    self.get_logger().warn(
                        f'Cannot start — state={self._state.name}'
                    )
                    return
                self._state = PipelineState.HOMING

            self.get_logger().info('Manual trigger received — starting pipeline')
            print('[pick_place_node] Pipeline starting!', flush=True)
            self._start_pipeline_thread()

        elif msg.data == 'home':
            if self._state == PipelineState.WAITING_FOR_TRIGGER:
                self.moveit.go_home()

        elif msg.data == 'reset':
            self._reset_to_waiting()
            self.get_logger().info('Manual reset to WAITING_FOR_TRIGGER')

    # ── Thread Management ────────────────────────────────────────────────

    def _start_pipeline_thread(self):
        """Start the pipeline execution in a daemon thread."""
        self._exec_thread = threading.Thread(
            target=self._execute_pipeline_safe,
            daemon=True
        )
        self._exec_thread.start()

    def _execute_pipeline_safe(self):
        """Wrapper that catches all exceptions and always resets."""
        try:
            self._execute_pick_place_pipeline()
        except Exception as e:
            self.get_logger().error(f'Pipeline EXCEPTION: {e}')
            import traceback
            traceback.print_exc()
            self._set_state(PipelineState.FAILED)
            # Try to go home on failure
            try:
                self.get_logger().info('Safety: returning to HOME after failure...')
                self.moveit.go_home()
            except Exception:
                pass
        finally:
            self._reset_to_waiting()

    def _reset_to_waiting(self):
        """Reset all state back to WAITING_FOR_TRIGGER."""
        # Stop grip reinforcement if active
        self._grip_hold_active = False
        if self._grip_hold_thread:
            self._grip_hold_thread.join(timeout=2.0)
            self._grip_hold_thread = None
            
        # Ensure object is detached and removed
        self.moveit.detach_object('target_cube')
        self.moveit.remove_collision_object('target_cube')

        with self._lock:
            if self._state == PipelineState.COMPLETE:
                self.get_logger().info('Task completed successfully.')
            self._state = PipelineState.WAITING_FOR_TRIGGER
        self.get_logger().info('Pipeline state → WAITING_FOR_TRIGGER (ready for next trigger)')

    def _set_state(self, state: PipelineState):
        """Update state machine and publish."""
        with self._lock:
            self._state = state
        self.get_logger().info(f'State → {state.name}')
        # Publish immediately
        msg = String()
        msg.data = state.name
        self._status_pub.publish(msg)

        busy_msg = String()
        busy_msg.data = 'true' if state != PipelineState.WAITING_FOR_TRIGGER else 'false'
        self._busy_pub.publish(busy_msg)

    # ── Main Pipeline ────────────────────────────────────────────────────

    def _execute_pick_place_pipeline(self):
        """
        Full pick-and-place state machine execution.

        Runs in a separate thread. Each MoveIt call is wrapped
        with a timeout check. On any failure, state goes to FAILED
        and the finally block in _execute_pipeline_safe resets to IDLE.
        """
        # Ensure object is added to the scene for planning
        self.moveit.add_collision_box(
            name='target_cube',
            size=(0.05, 0.05, 0.05),
            position=(self.test_pick_pose["x"], self.test_pick_pose["y"], self.test_pick_pose["z"]),
            frame_id='world'
        )
        
        self.get_logger().info('Allowing collision between gripper and target_cube')
        self.moveit.allow_collision(
            'target_cube', 
            ['left_finger_link', 'right_finger_link', 'gripper_base_link', 'tool0']
        )


        target_x = self.test_pick_pose["x"]
        target_y = self.test_pick_pose["y"]
        target_z = self.test_pick_pose["z"]

        # ── Gripper geometry & Table Avoidance ───────────────────────────
        # Finger box: 0.04 × 0.02 × 0.10 m, origin at +0.05m Z from finger joint
        # tool0      at gripper_base_link + 0.09m Z
        # → Finger spans from: tool0_z - 0.05m  to  tool0_z + 0.05m
        #
        # Table surface is at Z = 0.300
        # Cube is at Z = 0.300 to 0.350 (center 0.325)
        # 
        # CRITICAL: We MUST keep the bottom of the fingers above the table.
        # finger_bottom = tool0_z - 0.050 > 0.300  →  tool0_z > 0.350
        # If we set tool0_z = 0.355, finger bottom is 0.305 (5mm above table).
        # The fingers will grip the top 4.5cm of the cube.
        # ─────────────────────────────────────────────────────────────────
        cube_top_z    = target_z + 0.025  # 0.350
        finger_half   = 0.050             # half-height of finger box
        
        # Grasp: 5mm above table to avoid violent Gazebo collision
        grasp_z       = 0.355             
        
        # Final alignment: just above the cube
        final_align_z = 0.380

        # Slow approach: safe distance
        slow_approach_z = 0.420

        # Pre-grasp: high above
        pre_grasp_z   = 0.500

        lift_z        = 0.500
        place_z       = self.place_z  # 0.475


        self.get_logger().info('=' * 60)
        self.get_logger().info('  PICK-AND-PLACE PIPELINE STARTING (MANUAL MODE)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Cube center z  : {target_z:.4f}')
        self.get_logger().info(f'  Cube top z     : {cube_top_z:.4f}')
        self.get_logger().info(f'  Grasp target z : {grasp_z:.4f}')
        self.get_logger().info('------------------------------------------------------------')
        self.get_logger().info(f'  Cube center    : ({target_x:.4f}, {target_y:.4f}, {target_z:.4f})')
        self.get_logger().info(f'  Pre-grasp (z)  : {pre_grasp_z:.4f}')
        self.get_logger().info(f'  Slow appr. (z) : {slow_approach_z:.4f}')
        self.get_logger().info(f'  Gripper open   : {self.gripper_open_width*1000:.1f}mm')
        self.get_logger().info(f'  Gripper close  : {self.gripper_close_width*1000:.1f}mm')
        self.get_logger().info(f'  Place target   : ({self.place_x:.3f}, {self.place_y:.3f}, {place_z:.3f})')
        self.get_logger().info('=' * 60)

        # Publish debug marker
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'grasp'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(target_x)
        marker.pose.position.y = float(target_y)
        marker.pose.position.z = float(grasp_z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self._marker_pub.publish(marker)

        # ── STEP 1: HOME ─────────────────────────────────────────────
        self._set_state(PipelineState.HOMING)
        self.get_logger().info('[1/13] Moving to HOME...')
        self._move_with_timeout(lambda: self.moveit.go_home(), 'HOME')

        # ── STEP 2: OPEN GRIPPER ─────────────────────────────────────
        self._set_state(PipelineState.OPENING_GRIPPER)
        self.get_logger().info(f'[2/13] Opening gripper to {self.gripper_open_width*1000:.0f}mm...')
        self.moveit.open_gripper(self.gripper_open_width)
        self._wait(1.0)

        # ── STEP 3: PRE-GRASP (transit — NO path orientation constraint) ──
        self._set_state(PipelineState.MOVING_TO_PREGRASP)
        self.get_logger().info('Executing PRE-GRASP')
        self.get_logger().info(
            f'[3/13] Moving to PRE-GRASP: '
            f'({target_x:.4f}, {target_y:.4f}, {pre_grasp_z:.4f})'
        )
        self._move_with_timeout(
            lambda: self.moveit.move_to_pose(
                target_x, target_y, pre_grasp_z,
                constrain_orientation=False  # transit: only goal orientation matters
            ),
            'PRE-GRASP'
        )

        # ── STEP 4: SLOW APPROACH ─────────────
        self._set_state(PipelineState.DESCENDING)
        self.get_logger().info(
            f'[4/13] Slow approach to: '
            f'({target_x:.4f}, {target_y:.4f}, {slow_approach_z:.4f}) [Cartesian, vel=0.08]'
        )
        self._move_with_timeout(
            lambda: self.moveit.move_cartesian(
                target_x, target_y, slow_approach_z,
                velocity_scale=0.20, acceleration_scale=0.20,
                avoid_collisions=False
            ),
            'SLOW_APPROACH'
        )

        # ── STEP 5: FINAL ALIGNMENT (Cartesian) ─────────────
        self._set_state(PipelineState.FINAL_ALIGN)
        self.get_logger().info(
            f'[5/13] Final alignment to: '
            f'({target_x:.4f}, {target_y:.4f}, {final_align_z:.4f}) [vel=0.10]'
        )
        self._move_with_timeout(
            lambda: self.moveit.move_cartesian(
                target_x, target_y, final_align_z,
                velocity_scale=0.10, acceleration_scale=0.10,
                avoid_collisions=False
            ),
            'FINAL_ALIGN'
        )

        # ── STEP 6: TOUCHDOWN (Cartesian, very slow) ──────────
        self._set_state(PipelineState.TOUCHDOWN)
        self.get_logger().info('Executing FINAL GRASP')
        self.get_logger().info(
            f'[6/13] Touchdown descent to GRASP: '
            f'({target_x:.4f}, {target_y:.4f}, {grasp_z:.4f}) [straight-line, vel=0.10]'
        )
        self._move_with_timeout(
            lambda: self.moveit.move_cartesian(
                target_x, target_y, grasp_z,
                velocity_scale=0.10, acceleration_scale=0.10,
                avoid_collisions=False
            ),
            'TOUCHDOWN'
        )
        self.get_logger().info(f'  ✓ Grasp position reached at z={grasp_z:.4f}')

        # Settling delay for Gazebo physics to stabilize before closing
        self.get_logger().info('  [Waiting 0.5s for contact physics to settle...]')
        self._wait(0.5)

        # ── STEP 7: CLOSE GRIPPER ────────────────────────────────────
        self._set_state(PipelineState.GRASPING)
        self.get_logger().info(
            f'[7/13] Closing gripper to {self.gripper_close_width*1000:.1f}mm in steps...'
        )
        
        # Gradual close: ramp from open width down to gripper_close_width.
        #
        # Gripper geometry (from URDF):
        #   finger joint travel: [0..0.040] m (prismatic, ±Y direction)
        #   finger Y half-thickness: 0.010 m
        #   inner-face gap = 2 * (joint_pos - 0.010)
        #
        # For a 50 mm cube:
        #   just-touching: 2*(joint_pos - 0.010) = 0.050  → joint_pos = 0.035 m
        #   total width at just-touching = 2*0.035 = 0.070 m
        #   gripper_close_width = 0.044 m → joint_pos = 0.022 m
        #   inner-face gap = 2*(0.022-0.010) = 0.024 m  (13 mm squeeze per finger)
        #   The controller cannot reach 0.022 (blocked by cube) so it builds
        #   PID integral — producing sustained grip force without penetration.
        close_w = self.gripper_close_width  # 0.044 m default
        # Ramp: open → just-touching → final close setpoint
        for w in [0.08, 0.075, 0.070, close_w + 0.010, close_w]:
            self.moveit.set_gripper_width(max(w, close_w))
            self._wait(0.35)
            
        self.get_logger().info('  ✓ Gripper closed — holding for physics stabilization...')
        # 1.0 s lets Gazebo contact forces settle so the cube
        # is firmly clamped before we start moving the arm.
        self._wait(1.0)
        
        self.get_logger().info('  ✓ Attaching target_cube to tool0 in MoveIt planning scene')
        self.moveit.attach_object('target_cube', 'tool0')
        # Wait for MoveIt planning scene to propagate the attach —
        # without this delay the lift planner starts with the old (duplicate)
        # collision state and gets only 16% Cartesian fraction.
        self._wait(1.5)

        # ── START CONTINUOUS GRIP REINFORCEMENT ──────────────────────
        # Re-publish the close command at 10Hz during the entire lift/transport
        # to prevent the position controller from relaxing
        self._grip_hold_active = True
        self._grip_hold_thread = threading.Thread(
            target=self._grip_reinforcement_loop,
            daemon=True
        )
        self._grip_hold_thread.start()
        self.get_logger().info('  ✓ Continuous grip reinforcement STARTED')

        # ── STEP 8: SMOOTH LIFT ───────────
        self._set_state(PipelineState.LIFTING)
        
        self.get_logger().info('Executing LIFT')
        self.get_logger().info(
            f'[8/13] Smooth lift: z={grasp_z:.4f} -> {lift_z:.4f} (vel=0.05)'
        )
        self._move_with_timeout(
            lambda: self.moveit.move_cartesian(
                target_x, target_y, lift_z,
                velocity_scale=0.05, acceleration_scale=0.05,
                avoid_collisions=False
            ),
            'LIFT_FINAL'
        )
        
        self.get_logger().info(f'  ✓ Lift complete at z={lift_z:.4f}')

        # ── STEP 9: MOVE TO PRE-PLACE (transit — joint-space, higher clearance) ──
        self._set_state(PipelineState.MOVING_TO_PLACE)
        # Use a generous pre-place height so the cube clears obstacles during transit
        pre_place_z = self.place_z + 0.08   # 80 mm above place point
        self.get_logger().info(
            f'[9/13] Moving to PRE-PLACE: '
            f'({self.place_x:.3f}, {self.place_y:.3f}, {pre_place_z:.3f})'
        )
        # Joint-space transit (no path constraint) — just get near the place zone
        self._move_with_timeout(
            lambda: self.moveit.move_to_pose(
                self.place_x, self.place_y, pre_place_z,
                constrain_orientation=False,
                velocity_scale=0.15, acceleration_scale=0.15
            ),
            'PRE-PLACE'
        )
        # Short pause so physics settles after transit sway
        self._wait(0.3)

        # ── STEP 10: PLACE DESCENT (Cartesian, slow straight-line) ─────
        self._set_state(PipelineState.PLACING)
        self.get_logger().info('Executing PLACE')
        self.get_logger().info(
            f'[10/13] Cartesian descent to PLACE: '
            f'({self.place_x:.3f}, {self.place_y:.3f}, {place_z:.3f}) [Cartesian, vel=0.06]'
        )
        # Cartesian straight-line descent — avoids sideways push on the cube
        self._move_with_timeout(
            lambda: self.moveit.move_cartesian(
                self.place_x, self.place_y, place_z,
                velocity_scale=0.06, acceleration_scale=0.06,
                avoid_collisions=False
            ),
            'PLACE_DESCENT'
        )

        # Let cube settle on pad before releasing grip
        self.get_logger().info('  [Waiting 0.8s for cube to settle on pad...]')
        self._wait(0.8)

        # ── STEP 11: RELEASE ─────────────────────────────────────────
        self._set_state(PipelineState.RELEASING)
        self.get_logger().info('[11/13] Stopping grip reinforcement and opening gripper (RELEASING)...')
        # Stop continuous grip reinforcement BEFORE opening
        self._grip_hold_active = False
        if self._grip_hold_thread:
            self._grip_hold_thread.join(timeout=2.0)
            self._grip_hold_thread = None

        self.get_logger().info('  ✓ Detaching target_cube from tool0 in MoveIt planning scene')
        self.moveit.detach_object('target_cube')

        # Open gripper gently (gradual, not sudden snap)
        self.moveit.open_gripper(self.gripper_open_width)
        # Wait for gripper to open and cube to be fully released
        self._wait(1.0)

        # Cartesian retract straight up — avoids nudging the placed cube
        self._set_state(PipelineState.RETRACTING)
        self.get_logger().info('[12/13] Retracting straight up...')
        self._move_with_timeout(
            lambda: self.moveit.move_cartesian(
                self.place_x, self.place_y, pre_place_z,
                velocity_scale=0.10, acceleration_scale=0.10,
                avoid_collisions=False
            ),
            'RETRACT'
        )

        # ── STEP 13: RETURN HOME ─────────────────────────────────────
        self._set_state(PipelineState.RETURNING_HOME)
        self.get_logger().info('[13/13] Returning to HOME...')
        self._move_with_timeout(lambda: self.moveit.go_home(), 'RETURN_HOME')

        # ── COMPLETE ─────────────────────────────────────────────────
        self._set_state(PipelineState.COMPLETE)
        self.get_logger().info('=' * 60)
        self.get_logger().info('  PICK-AND-PLACE COMPLETE ✓')
        self.get_logger().info('=' * 60)
        # _reset_to_waiting() is called by the finally block in _execute_pipeline_safe

    # ── Helpers ──────────────────────────────────────────────────────────

    def _move_with_timeout(self, move_fn, label: str):
        """
        Execute a MoveIt motion synchronously.
        MoveIt natively handles timeouts and action results.
        """
        start = time.time()
        
        try:
            result = move_fn()
        except Exception as e:
            self.get_logger().error(f'Motion error in {label}: {e}')
            raise e

        elapsed = time.time() - start

        if result is False:
            self.get_logger().error(f'Motion FAILED: {label} (returned False)')
            raise RuntimeError(f'Motion failed: {label}')

        self.get_logger().info(f'  ✓ {label} completed ({elapsed:.1f}s)')
        self.get_logger().info(f'{label} motion confirmed complete')

    def _wait(self, seconds: float):
        """Wait for specified time."""
        time.sleep(seconds)

    def _grip_reinforcement_loop(self):
        """
        Continuously re-publish the gripper close command at 10Hz.
        
        This is CRITICAL for maintaining grip during lift. A position
        controller in Gazebo can relax once position is 'reached' (even
        if blocked by cube). By continuously re-commanding the closed
        position, we keep the PID integral term active and the controller
        continuously pushing the fingers into the cube.
        """
        from std_msgs.msg import Float64MultiArray
        rate_hz = 10.0
        period = 1.0 / rate_hz
        close_pos = self.gripper_close_width / 2.0  # Per-finger position (0.0 for full close)
        
        self.get_logger().info(
            f'Grip reinforcement loop running at {rate_hz}Hz, '
            f'commanding finger pos={close_pos:.4f}'
        )
        
        while self._grip_hold_active:
            msg = Float64MultiArray()
            msg.data = [close_pos, close_pos]
            self._gripper_pub.publish(msg)
            time.sleep(period)
        
        self.get_logger().info('Grip reinforcement loop stopped')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    # Add the moveit_interface's isolated node so action result callbacks
    # are properly delivered (without this, spin_until_future_complete
    # returns before the trajectory execution result arrives).
    executor.add_node(node.moveit.node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
