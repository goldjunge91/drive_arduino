/*
 * Copyright (c) 2024 MecaBridge Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

TEST(TestMecabridgeHardwareInterface, load_urdf_and_check_interfaces)
{
  std::string urdf_string =
    R"(
<robot name="TestRobot">
  <ros2_control name="MecaBridgeSystem" type="system">
    <hardware>
      <plugin>mecabridge_hardware/MecaBridgeHardware</plugin>
      <param name="config_file">C:\GIT\drive_arduino\src\mecabridge_hardware\test\mecabridge\hardware\test_mecabridge_hardware_interface.yaml</param>
    </hardware>
    <joint name="front_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    <joint name="front_right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    <joint name="rear_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    <joint name="rear_right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>
)";

  rclcpp::init(0, nullptr);

  try {
    hardware_interface::ResourceManager rm(urdf_string);

    const auto & command_interfaces = rm.command_interface_keys();
    const auto & state_interfaces = rm.state_interface_keys();

    ASSERT_EQ(command_interfaces.size(), 4);
    ASSERT_EQ(state_interfaces.size(), 8);

    EXPECT_EQ(command_interfaces[0], "front_left_wheel_joint/velocity");
    EXPECT_EQ(command_interfaces[1], "front_right_wheel_joint/velocity");
    EXPECT_EQ(command_interfaces[2], "rear_left_wheel_joint/velocity");
    EXPECT_EQ(command_interfaces[3], "rear_right_wheel_joint/velocity");

    EXPECT_EQ(state_interfaces[0], "front_left_wheel_joint/velocity");
    EXPECT_EQ(state_interfaces[1], "front_left_wheel_joint/position");
    EXPECT_EQ(state_interfaces[2], "front_right_wheel_joint/velocity");
    EXPECT_EQ(state_interfaces[3], "front_right_wheel_joint/position");
    EXPECT_EQ(state_interfaces[4], "rear_left_wheel_joint/velocity");
    EXPECT_EQ(state_interfaces[5], "rear_left_wheel_joint/position");
    EXPECT_EQ(state_interfaces[6], "rear_right_wheel_joint/velocity");
    EXPECT_EQ(state_interfaces[7], "rear_right_wheel_joint/position");

  } catch (const std::exception & e) {
    FAIL() << "Exception thrown: " << e.what();
  }

  rclcpp::shutdown();
}
