#ifndef TB6612_HARDWARE_TB6612_HARDWARE_INTERFACE_H
#define TB6612_HARDWARE_TB6612_HARDWARE_INTERFACE_H

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

#include "tb6612_hardware/tb6612_comms.h"
#include "tb6612_hardware/tb6612_config.h"
#include "tb6612_hardware/wheel.h"

namespace tb6612_hardware
{
class TB6612HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TB6612HardwareInterface);

  TB6612HardwareInterface();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  TB6612Config cfg_;
  TB6612Comms tb6612_;
  
  // Wheels based on drive type
  std::vector<Wheel> wheels_;
  
  rclcpp::Logger logger_;
  std::chrono::time_point<std::chrono::system_clock> time_;
  
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

}  // namespace tb6612_hardware

#endif  // TB6612_HARDWARE_TB6612_HARDWARE_INTERFACE_H