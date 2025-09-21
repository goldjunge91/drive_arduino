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
    </joint>
    <joint name="front_right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="rear_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="rear_right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="pan_servo_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <joint name="rot_servo_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="esc_left_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="esc_right_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
)";

  rclcpp::init(0, nullptr);

  try {
    hardware_interface::ResourceManager rm(urdf_string);

    const auto & command_interfaces = rm.command_interface_keys();
    const auto & state_interfaces = rm.state_interface_keys();

    ASSERT_EQ(command_interfaces.size(), 8);
    ASSERT_EQ(state_interfaces.size(), 8);

    EXPECT_EQ(command_interfaces[0], "front_left_wheel_joint/velocity");
    EXPECT_EQ(command_interfaces[1], "front_right_wheel_joint/velocity");
    EXPECT_EQ(command_interfaces[2], "rear_left_wheel_joint/velocity");
    EXPECT_EQ(command_interfaces[3], "rear_right_wheel_joint/velocity");
    EXPECT_EQ(command_interfaces[4], "pan_servo_joint/position");
    EXPECT_EQ(command_interfaces[5], "rot_servo_joint/velocity");
    EXPECT_EQ(command_interfaces[6], "esc_left_joint/velocity");
    EXPECT_EQ(command_interfaces[7], "esc_right_joint/velocity");

    EXPECT_EQ(state_interfaces[0], "front_left_wheel_joint/velocity");
    EXPECT_EQ(state_interfaces[1], "front_right_wheel_joint/velocity");
    EXPECT_EQ(state_interfaces[2], "rear_left_wheel_joint/velocity");
    EXPECT_EQ(state_interfaces[3], "rear_right_wheel_joint/velocity");
    EXPECT_EQ(state_interfaces[4], "pan_servo_joint/position");
    EXPECT_EQ(state_interfaces[5], "rot_servo_joint/velocity");
    EXPECT_EQ(state_interfaces[6], "esc_left_joint/velocity");
    EXPECT_EQ(state_interfaces[7], "esc_right_joint/velocity");

  } catch (const std::exception & e) {
    FAIL() << "Exception thrown: " << e.what();
  }

  rclcpp::shutdown();
}
