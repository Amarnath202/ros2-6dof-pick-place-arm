#!/usr/bin/env python3
# ============================================================
# arm_moveit_config/launch/moveit_rviz.launch.py
#
# Launches RViz2 with MoveIt2 MotionPlanning panel for
# interactive planning and execution visualization.
#
# Usage: ros2 launch arm_moveit_config moveit_rviz.launch.py
# ============================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():

    pkg_arm_description = get_package_share_directory('arm_description')
    pkg_arm_moveit = get_package_share_directory('arm_moveit_config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot description — pass controllers config so xacro arg resolves
    pkg_arm_gazebo = get_package_share_directory('arm_gazebo')
    controllers_yaml = os.path.join(pkg_arm_gazebo, 'config', 'ros2_controllers.yaml')
    urdf_file = os.path.join(pkg_arm_description, 'urdf', 'arm.urdf.xacro')
    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' ros2_controllers_config:=', controllers_yaml,
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # SRDF
    srdf_file = os.path.join(pkg_arm_moveit, 'config', 'arm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # Kinematics
    kinematics_yaml = load_yaml('arm_moveit_config', 'config/kinematics.yaml')

    # RViz2 configuration file with MoveIt2 MotionPlanning plugin
    rviz_config_file = os.path.join(pkg_arm_moveit, 'config', 'rviz', 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_node,
    ])
