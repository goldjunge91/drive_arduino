#pragma once

#include <string>
#include <vector>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mecabridge_utils/config/config.hpp"

namespace mecabridge_hardware {

class MecaBridgeHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecaBridgeHardware)

  MecaBridgeHardware() = default;
  ~MecaBridgeHardware() override = default;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Deterministic joint ordering: 4 wheels (vel), 1 positional servo (pos), 1 continuous servo (vel), 2 ESC (vel normalized)
  std::vector<double> hw_states_{};   // size 8-? state interfaces
  std::vector<double> hw_commands_{}; // size depends on interfaces

  mecabridge::config::Config cfg_;

  bool configured_ = false;
  bool active_ = false;
};

} // namespace mecabridge_hardware

