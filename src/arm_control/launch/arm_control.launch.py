#!/usr/bin/env python3
# ============================================================
# arm_control/launch/arm_control.launch.py
#
# Launches the arm control nodes:
#   - gripper_controller_node
#   - pick_place_node (with MoveIt2 parameters loaded)
#
# Usage: ros2 launch arm_control arm_control.launch.py
# ============================================================

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    pkg_arm_gazebo = get_package_share_directory('arm_gazebo')

    # ── Launch arguments ─────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Robot Description (URDF via Xacro) ───────────────────────────────
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

    # ── Gripper controller node ───────────────────────────────────────────
    gripper_controller_node = Node(
        package='arm_control',
        executable='gripper_controller.py',
        name='gripper_controller_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'gripper_open_position': 0.04},
            {'gripper_close_position': 0.002},
            {'command_topic': '/gripper_controller/commands'},
        ],
    )

    # ── Pick-and-place pipeline node ──────────────────────────────────────
    pick_place_node = Node(
        package='arm_control',
        executable='pick_place_node.py',
        name='pick_place_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {'use_sim_time': use_sim_time},
            {'place_x': 0.35},              # Green pad X in world (place_zone model)
            {'place_y': 0.20},              # Green pad Y in world (place_zone model)
            {'place_z': 0.355},             # tool0 Z to place cube ON table surface
            {'approach_height': 0.15},
            {'grasp_z_offset': 0.027},
            {'slow_approach_height': 0.10},
            {'planning_timeout': 30.0},
            # Auto-start: pipeline fires automatically 5s after node is ready.
            # Set to 0.0 to disable and use manual trigger only.
            {'auto_start_delay': 5.0},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        gripper_controller_node,
        pick_place_node,
    ])
