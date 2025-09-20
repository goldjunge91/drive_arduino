#include "mecabridge_hardware/mecabridge_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mecabridge_hardware {

hardware_interface::CallbackReturn MecaBridgeHardware::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get config file path from <param>
  auto it = info.hardware_parameters.find("config_file");
  if (it == info.hardware_parameters.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("MecaBridgeHardware"), "config_file parameter not found");
      return hardware_interface::CallbackReturn::ERROR;
  }
  std::string config_file = it->second;

  try {
      cfg_ = mecabridge::config::parse_from_yaml_file(config_file);
  } catch (const std::runtime_error& e) {
      RCLCPP_ERROR(rclcpp::get_logger("MecaBridgeHardware"), "Failed to parse YAML config: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize vectors based on joints from config
  size_t num_joints = 4 + (cfg_.features.enable_servos ? 2 : 0) + (cfg_.features.enable_escs ? 2 : 0);
  hw_states_.assign(num_joints, 0.0);
  hw_commands_.assign(num_joints, 0.0);

  configured_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecaBridgeHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> si;
  si.reserve(hw_states_.size());

  auto wheel_names = cfg_.wheel_joint_names();
  for (size_t i = 0; i < 4; ++i) {
    si.emplace_back(hardware_interface::StateInterface(wheel_names[i], hardware_interface::HW_IF_VELOCITY, &hw_states_[i]));
  }

  size_t offset = 4;
  if (cfg_.features.enable_servos) {
    si.emplace_back(hardware_interface::StateInterface(cfg_.servos.positional.joint_name, hardware_interface::HW_IF_POSITION, &hw_states_[offset++]));
    si.emplace_back(hardware_interface::StateInterface(cfg_.servos.continuous.joint_name, hardware_interface::HW_IF_VELOCITY, &hw_states_[offset++]));
  }

  if (cfg_.features.enable_escs) {
    si.emplace_back(hardware_interface::StateInterface(cfg_.escs.left.joint_name, hardware_interface::HW_IF_VELOCITY, &hw_states_[offset++]));
    si.emplace_back(hardware_interface::StateInterface(cfg_.escs.right.joint_name, hardware_interface::HW_IF_VELOCITY, &hw_states_[offset++]));
  }

  return si;
}

std::vector<hardware_interface::CommandInterface> MecaBridgeHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> ci;
  ci.reserve(hw_commands_.size());

  auto wheel_names = cfg_.wheel_joint_names();
  for (size_t i = 0; i < 4; ++i) {
    ci.emplace_back(hardware_interface::CommandInterface(wheel_names[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  size_t offset = 4;
  if (cfg_.features.enable_servos) {
    ci.emplace_back(hardware_interface::CommandInterface(cfg_.servos.positional.joint_name, hardware_interface::HW_IF_POSITION, &hw_commands_[offset++]));
    ci.emplace_back(hardware_interface::CommandInterface(cfg_.servos.continuous.joint_name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[offset++]));
  }

  if (cfg_.features.enable_escs) {
    ci.emplace_back(hardware_interface::CommandInterface(cfg_.escs.left.joint_name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[offset++]));
    ci.emplace_back(hardware_interface::CommandInterface(cfg_.escs.right.joint_name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[offset++]));
  }
  
  return ci;
}

hardware_interface::CallbackReturn MecaBridgeHardware::on_configure(const rclcpp_lifecycle::State &) {
  if (!configured_) return hardware_interface::CallbackReturn::ERROR;
  // In future: open serial backend, perform handshake
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecaBridgeHardware::on_cleanup(const rclcpp_lifecycle::State &) {
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecaBridgeHardware::on_activate(const rclcpp_lifecycle::State &) {
  if (!configured_) return hardware_interface::CallbackReturn::ERROR;
  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecaBridgeHardware::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecaBridgeHardware::read(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!active_) return hardware_interface::return_type::ERROR;
  // In future: read STATE frame, update hw_states_
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecaBridgeHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!active_) return hardware_interface::return_type::ERROR;
  // In future: encode COMMAND frame from hw_commands_
  return hardware_interface::return_type::OK;
}

} // namespace mecabridge_hardware

PLUGINLIB_EXPORT_CLASS(mecabridge_hardware::MecaBridgeHardware, hardware_interface::SystemInterface)

