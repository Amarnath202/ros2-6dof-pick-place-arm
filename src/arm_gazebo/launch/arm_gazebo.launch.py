#!/usr/bin/env python3
# ============================================================
# arm_gazebo/launch/arm_gazebo.launch.py
#
# Launches the complete Gazebo simulation:
#   1. Gazebo server with the arm_world.world
#   2. robot_state_publisher (URDF -> TF)
#   3. Spawn robot into Gazebo via spawn_entity
#   4. Load and activate ros2_control controllers
#
# Usage: ros2 launch arm_gazebo arm_gazebo.launch.py
# ============================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package paths ────────────────────────────────────────────────────
    pkg_arm_description = get_package_share_directory('arm_description')
    pkg_arm_gazebo = get_package_share_directory('arm_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # ── Launch arguments ─────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_arm_gazebo, 'worlds', 'arm_world.world'),
        description='Full path to Gazebo world file'
    )

    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Robot X spawn position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Robot Y spawn position')
    z_arg = DeclareLaunchArgument('z', default_value='0.0', description='Robot Z spawn position')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    # ── URDF/Xacro -> robot_description ──────────────────────────────────
    # ParameterValue(value_type=str) prevents ROS2 from trying to parse
    # the URDF XML string as YAML (which causes the launch error).
    controllers_yaml = os.path.join(pkg_arm_gazebo, 'config', 'ros2_controllers.yaml')
    urdf_file = os.path.join(pkg_arm_description, 'urdf', 'arm.urdf.xacro')
    
    xacro_clean_script = os.path.join(pkg_arm_description, 'urdf', 'xacro_clean.py')
    
    # ParameterValue prevents ROS2 from trying to parse the XML as YAML or args
    robot_description_content = ParameterValue(
        Command([
            xacro_clean_script, ' ', urdf_file,
            ' ros2_controllers_config:=', controllers_yaml,
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # ── 1. Launch Gazebo server + client ─────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'false',
            'pause': 'false',
        }.items(),
    )

    # ── 2. Robot State Publisher ──────────────────────────────────────────
    # Publishes robot description and all TF transforms from URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── 3. Spawn Robot in Gazebo ──────────────────────────────────────────
    # Uses the /robot_description topic to spawn the URDF into Gazebo
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_arm',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'arm_robot',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', '0',
            '-P', '0',
            '-Y', '0',
        ],
    )

    # ── 4. Load Controllers ───────────────────────────────────────────────
    # These ExecuteProcess nodes call the ros2 control CLI to load controllers
    # They run AFTER the robot is spawned (event handler)

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    # ── Event handlers: chain controller loading ──────────────────────────
    # Load joint_state_broadcaster AFTER robot is spawned
    spawn_to_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    # Load arm_controller AFTER joint_state_broadcaster is active
    jsb_to_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller],
        )
    )

    # Load gripper_controller AFTER arm_controller is active
    arm_to_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm_controller,
            on_exit=[load_gripper_controller],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        x_arg, y_arg, z_arg,

        # Start Gazebo
        gazebo_launch,

        # Start robot state publisher
        robot_state_publisher_node,

        # Spawn robot
        spawn_robot_node,

        # Load controllers in sequence
        spawn_to_jsb,
        jsb_to_arm,
        arm_to_gripper,
    ])
