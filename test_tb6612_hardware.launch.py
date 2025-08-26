#!/usr/bin/env python3
"""
Launch file to test TB6612 Hardware Interface with various configurations
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for testing different configurations
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='',
        description='Serial device path (empty for auto-detection)'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', 
        default_value='115200',
        description='Serial baud rate'
    )
    
    drive_type_arg = DeclareLaunchArgument(
        'drive_type',
        default_value='differential', 
        choices=['differential', 'four_wheel', 'mecanum'],
        description='Drive type configuration'
    )
    
    has_encoders_arg = DeclareLaunchArgument(
        'has_encoders',
        default_value='false',
        description='Whether encoders are available'
    )
    
    # Test with invalid parameters to verify error handling
    test_invalid_arg = DeclareLaunchArgument(
        'test_invalid',
        default_value='false',
        description='Test with invalid parameters'
    )

    # Controller Manager Node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'robot_description': '''
            <robot name="test_robot">
              <ros2_control name="tb6612_hardware" type="system">
                <hardware>
                  <plugin>tb6612_hardware/TB6612HardwareInterface</plugin>
                  <param name="device">''' + LaunchConfiguration('device').perform(None) + '''</param>
                  <param name="baud_rate">''' + LaunchConfiguration('baud_rate').perform(None) + '''</param>
                  <param name="drive_type">''' + LaunchConfiguration('drive_type').perform(None) + '''</param>
                  <param name="has_encoders">''' + LaunchConfiguration('has_encoders').perform(None) + '''</param>
                  <param name="timeout">50</param>
                  <param name="loop_rate">20.0</param>
                  <param name="max_lin_vel">0.3</param>
                  <param name="max_ang_vel">1.0</param>
                  <param name="wheel_radius">0.05</param>
                  <param name="left_wheel_name">left_wheel</param>
                  <param name="right_wheel_name">right_wheel</param>
                </hardware>
                <joint name="left_wheel">
                  <command_interface name="velocity"/>
                  <state_interface name="position"/>
                  <state_interface name="velocity"/>
                </joint>
                <joint name="right_wheel">
                  <command_interface name="velocity"/>
                  <state_interface name="position"/>
                  <state_interface name="velocity"/>
                </joint>
              </ros2_control>
            </robot>
            ''',
            'update_rate': 20,
        }],
        output='screen',
    )

    # Test Error Handling Node
    error_test_node = Node(
        package='drive_arduino',
        executable='test_tb6612_error_handling.py',
        output='screen',
        condition=LaunchConfiguration('test_invalid')
    )

    return LaunchDescription([
        device_arg,
        baud_rate_arg, 
        drive_type_arg,
        has_encoders_arg,
        test_invalid_arg,
        
        LogInfo(msg=['Testing TB6612 Hardware Interface with:']),
        LogInfo(msg=['  Device: ', LaunchConfiguration('device')]),
        LogInfo(msg=['  Baud Rate: ', LaunchConfiguration('baud_rate')]),
        LogInfo(msg=['  Drive Type: ', LaunchConfiguration('drive_type')]),
        LogInfo(msg=['  Has Encoders: ', LaunchConfiguration('has_encoders')]),
        
        controller_manager,
        error_test_node,
    ])