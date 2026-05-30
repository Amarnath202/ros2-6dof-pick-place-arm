#!/usr/bin/env python3
# ============================================================
# arm_moveit_config/launch/move_group.launch.py
#
# Launches the MoveIt2 move_group action server.
# This is the core MoveIt2 node that:
#   - Loads robot description + SRDF
#   - Starts the planning pipeline (OMPL)
#   - Starts the trajectory execution manager
#   - Interfaces with ros2_control controllers
#   - Publishes planning scene
#
# Usage: ros2 launch arm_moveit_config move_group.launch.py
# ============================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name, file_path):
    """Helper: load a YAML file from a package share directory."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        raise RuntimeError(f'Failed to load {absolute_file_path}: {e}')


def generate_launch_description():

    # ── Package paths ────────────────────────────────────────────────────
    pkg_arm_description = get_package_share_directory('arm_description')
    pkg_arm_moveit = get_package_share_directory('arm_moveit_config')

    # ── Launch arguments ─────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Robot Description (URDF via Xacro) ───────────────────────────────
    # ParameterValue(value_type=str) prevents ROS2 from parsing URDF XML as YAML
    pkg_arm_gazebo = get_package_share_directory('arm_gazebo')
    controllers_yaml = os.path.join(pkg_arm_gazebo, 'config', 'ros2_controllers.yaml')
    urdf_file = os.path.join(pkg_arm_description, 'urdf', 'arm.urdf.xacro')
    xacro_clean_script = os.path.join(pkg_arm_description, 'urdf', 'xacro_clean.py')
    robot_description_content = ParameterValue(
        Command([
            xacro_clean_script, ' ', urdf_file,
            ' ros2_controllers_config:=', controllers_yaml,
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # ── Semantic Robot Description (SRDF) ─────────────────────────────────
    srdf_file = os.path.join(pkg_arm_moveit, 'config', 'arm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # ── Kinematics configuration ──────────────────────────────────────────
    kinematics_yaml = load_yaml('arm_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # ── Joint limits ──────────────────────────────────────────────────────
    joint_limits_yaml = load_yaml('arm_moveit_config', 'config/joint_limits.yaml')
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    # ── OMPL planning pipeline ────────────────────────────────────────────
    ompl_planning_yaml = load_yaml(
        'arm_moveit_config',
        'config/planning_pipelines/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': ompl_planning_yaml['planning_plugin'],
            'request_adapters': ompl_planning_yaml['request_adapters'],
            'start_state_max_bounds_error': ompl_planning_yaml['start_state_max_bounds_error'],
        }
    }

    # ── Trajectory execution ──────────────────────────────────────────────
    trajectory_execution = load_yaml('arm_moveit_config', 'config/trajectory_execution.yaml')

    # ── MoveIt controllers ────────────────────────────────────────────────
    moveit_controllers = load_yaml('arm_moveit_config', 'config/moveit_controllers.yaml')

    # ── Planning scene monitor settings ──────────────────────────────────
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'planning_scene_monitor_options': {
            'name': 'planning_scene_monitor',
            'robot_description': 'robot_description',
            'joint_state_topic': '/joint_states',
            'attached_collision_object_topic': '/move_group/planning_scene_monitor',
            'publish_planning_scene_topic': '/move_group/publish_planning_scene',
            'monitored_planning_scene_topic': '/move_group/monitored_planning_scene',
            'wait_for_initial_pose_timeout': 10.0,
        },
    }

    # ── Move Group Node ───────────────────────────────────────────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
        ],
        # Increase logging for debugging
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription([
        use_sim_time_arg,
        move_group_node,
    ])
