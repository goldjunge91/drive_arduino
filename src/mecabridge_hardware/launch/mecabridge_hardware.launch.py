from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Minimal placeholder launch for integration. In real runs, provide a YAML config via parameters
    # and start controller_manager with the mecabridge_hardware plugin.
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                # Add path to your YAML with ros2_control and mecabridge_hardware plugin
            ],
        ),
    ])

