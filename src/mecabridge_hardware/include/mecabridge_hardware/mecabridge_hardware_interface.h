#ifndef MECABRIDGE_HARDWARE__MECABRIDGE_HARDWARE__MECABRIDGE_HARDWARE_INTERFACE__H_
#define MECABRIDGE_HARDWARE__MECABRIDGE_HARDWARE__MECABRIDGE_HARDWARE_INTERFACE__H_



#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mecabridge_hardware/mecabridge_serial_protocol.h"
#include "mecabridge_hardware/mecabridge_drive_config.h"
#include "mecabridge_hardware/wheel.h"

namespace mecabridge_hardware
{
  class MecaBridgeHardwareInterface: public hardware_interface::SystemInterface
  {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MecaBridgeHardwareInterface);

    MecaBridgeHardwareInterface();

    hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

    std::vector < hardware_interface::StateInterface > export_state_interfaces() override;

    std::vector < hardware_interface::CommandInterface > export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    MecaBridgeDriveConfig cfg_;
    MecaBridgeSerialProtocol serial_protocol_;

    // Wheels based on drive type
    std::vector < Wheel > wheels_;

    rclcpp::Logger logger_;
    std::chrono::time_point < std::chrono::system_clock > time_;

    // Connection recovery
    bool attemptConnectionRecovery();

    // Parameter validation and error handling methods (temporarily removed for compilation fix)

    // Helper methods
    int convertVelocityToMotorCommand(double wheel_vel_rad_s);

    // Error tracking
    mutable int connection_error_count_;
    mutable int read_error_count_;
    mutable int write_error_count_;
  };

}  // namespace mecabridge_hardware

#endif  // MECABRIDGE_HARDWARE__MECABRIDGE_HARDWARE__MECABRIDGE_HARDWARE_INTERFACE__H_
