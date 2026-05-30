#!/usr/bin/env python3
# ============================================================
# arm_bringup/launch/bringup.launch.py
#
# MASTER LAUNCH FILE — Starts the COMPLETE system:
#   1. Gazebo simulation (world + robot spawn + controllers)
#   2. MoveIt2 move_group (motion planning)
#   3. MoveIt2 RViz (visualization + planning UI)
#   4. Vision pipeline (YOLO + EasyOCR + TF)
#   5. Pick-and-place control (gripper + pipeline)
#
# LAUNCH ARGUMENTS:
#   use_sim_time: Use Gazebo clock (default: true)
#   launch_gazebo: Start Gazebo (default: true)
#   launch_moveit: Start MoveIt2 (default: true)
#   launch_rviz: Start RViz2 (default: true)
#   launch_vision: Start vision pipeline (default: true)
#   launch_control: Start pick-place control (default: true)
#
# USAGE:
#   ros2 launch arm_bringup bringup.launch.py
#
#   # Without vision (for arm testing only):
#   ros2 launch arm_bringup bringup.launch.py launch_vision:=false
#
#   # Without RViz:
#   ros2 launch arm_bringup bringup.launch.py launch_rviz:=false
# ============================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # ── Package share directories ────────────────────────────────────────
    pkg_arm_gazebo = get_package_share_directory('arm_gazebo')
    pkg_arm_moveit = get_package_share_directory('arm_moveit_config')
    pkg_arm_control = get_package_share_directory('arm_control')
    pkg_arm_vision = get_package_share_directory('arm_vision')

    # ── Launch arguments ─────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'
        ),
        DeclareLaunchArgument(
            'launch_gazebo',
            default_value='true',
            description='Launch Gazebo simulation'
        ),
        DeclareLaunchArgument(
            'launch_moveit',
            default_value='true',
            description='Launch MoveIt2 move_group'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz2 with MoveIt2 plugin'
        ),
        DeclareLaunchArgument(
            'launch_vision',
            default_value='true',
            description='Launch YOLO + EasyOCR vision pipeline'
        ),
        DeclareLaunchArgument(
            'launch_control',
            default_value='true',
            description='Launch pick-and-place control pipeline'
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_gazebo = LaunchConfiguration('launch_gazebo')
    launch_moveit = LaunchConfiguration('launch_moveit')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_vision = LaunchConfiguration('launch_vision')
    launch_control = LaunchConfiguration('launch_control')

    # ── 1. GAZEBO ─────────────────────────────────────────────────────────
    # Starts Gazebo, robot_state_publisher, spawns robot, loads controllers
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_arm_gazebo, 'launch', 'arm_gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(launch_gazebo),
    )

    # ── 2. MOVEIT2 move_group ─────────────────────────────────────────────
    # Delayed 5 seconds to let Gazebo+controllers initialize first
    moveit_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_arm_moveit, 'launch', 'move_group.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items(),
                condition=IfCondition(launch_moveit),
            )
        ]
    )

    # ── 3. RVIZ2 ──────────────────────────────────────────────────────────
    # Delayed 8 seconds to let MoveIt2 fully initialize
    rviz_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_arm_moveit, 'launch', 'moveit_rviz.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items(),
                condition=IfCondition(launch_rviz),
            )
        ]
    )

    # ── 4. VISION PIPELINE ────────────────────────────────────────────────
    # Delayed 6 seconds (needs robot TF to be available)
    vision_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_arm_vision, 'launch', 'vision.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items(),
                condition=IfCondition(launch_vision),
            )
        ]
    )

    # ── 5. CONTROL PIPELINE ───────────────────────────────────────────────
    # Delayed 7 seconds (needs controllers and vision to be ready)
    control_launch = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_arm_control, 'launch', 'arm_control.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items(),
                condition=IfCondition(launch_control),
            )
        ]
    )

    # ── Status messages ───────────────────────────────────────────────────
    log_start = LogInfo(msg='[arm_bringup] Starting 6-DOF Arm complete system...')
    log_gazebo = LogInfo(msg='[arm_bringup] Step 1: Launching Gazebo...')
    log_moveit = LogInfo(msg='[arm_bringup] Step 2: Launching MoveIt2 (after 5s)...')
    log_rviz = LogInfo(msg='[arm_bringup] Step 3: Launching RViz2 (after 8s)...')
    log_vision = LogInfo(msg='[arm_bringup] Step 4: Launching Vision (after 6s)...')
    log_control = LogInfo(msg='[arm_bringup] Step 5: Launching Control (after 7s)...')

    return LaunchDescription(
        args + [
            SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
            SetEnvironmentVariable('MESA_GLSL_VERSION_OVERRIDE', '330'),
            log_start,
            log_gazebo,
            gazebo_launch,
            log_moveit,
            moveit_launch,
            log_rviz,
            rviz_launch,
            # log_vision,
            # vision_launch,
            log_control,
            control_launch,
        ]
    )
