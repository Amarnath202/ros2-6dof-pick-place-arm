#!/usr/bin/env python3
# ============================================================
# arm_vision/launch/vision.launch.py
#
# Launches all vision nodes:
#   1. camera_tf_broadcaster_node (for real cameras)
#   2. yolo_detector_node
#   3. easyocr_detector_node
#   4. coord_transformer_node
#
# Usage: ros2 launch arm_vision vision.launch.py
# ============================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_arm_vision = get_package_share_directory('arm_vision')
    vision_params = os.path.join(pkg_arm_vision, 'config', 'vision_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Camera TF broadcaster ─────────────────────────────────────────────
    camera_tf_node = Node(
        package='arm_vision',
        executable='camera_tf_broadcaster.py',
        name='camera_tf_broadcaster_node',
        output='screen',
        parameters=[
            vision_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── Color detector ────────────────────────────────────────────────────
    color_node = Node(
        package='arm_vision',
        executable='color_detector.py',
        name='color_detector_node',
        output='screen',
        parameters=[
            vision_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── Coordinate transformer ────────────────────────────────────────────
    coord_transformer_node = Node(
        package='arm_vision',
        executable='coord_transformer.py',
        name='coord_transformer_node',
        output='screen',
        parameters=[
            vision_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        camera_tf_node,
        color_node,
        coord_transformer_node,
    ])
