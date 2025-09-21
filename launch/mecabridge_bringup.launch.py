#!/usr/bin/env python3
"""Launch MecaBridge ros2_control hardware and controllers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("drive_arduino"), "controllers", "mecabridge_config.yaml"]
        ),
        description="YAML file with ros2_control hardware and controller configuration.",
    )

    config = LaunchConfiguration("config")

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[config],
        output="both",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    mecanum_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller"],
        output="screen",
    )

    servo_position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["servo_position_controller"],
        output="screen",
    )

    servo_velocity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["servo_velocity_controller"],
        output="screen",
    )

    esc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["esc_controller"],
        output="screen",
    )

    start_joint_state = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_spawner],
        )
    )

    start_remaining = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_spawner,
            on_start=[mecanum_spawner, servo_position_spawner, servo_velocity_spawner, esc_spawner],
        )
    )

    return LaunchDescription(
        [
            config_arg,
            control_node,
            start_joint_state,
            start_remaining,
        ]
    )
