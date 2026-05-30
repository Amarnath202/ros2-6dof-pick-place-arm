#!/usr/bin/env python3
# ============================================================
# arm_description/launch/display.launch.py
# Launches RViz2 with the arm URDF for visualization/debug.
# Usage: ros2 launch arm_description display.launch.py
# ============================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_arm_description = get_package_share_directory('arm_description')
    pkg_arm_gazebo = get_package_share_directory('arm_gazebo')

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch joint_state_publisher_gui for manual joint control'
    )
    use_gui = LaunchConfiguration('use_gui')

    # Pass controllers config as xacro argument (resolves the xacro:arg default)
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

    return LaunchDescription([
        use_gui_arg,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(use_gui),
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=UnlessCondition(use_gui),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
