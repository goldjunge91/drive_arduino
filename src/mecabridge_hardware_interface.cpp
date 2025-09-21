#include "mecabridge/mecabridge_hardware_interface.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <chrono>
#include <optional>
#include <sstream>
#include <utility>

#include "hardware_interface/component_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mecabridge
{
namespace
{
constexpr double DEG_TO_RAD = M_PI / 180.0;

std::string trim(const std::string & input)
{
  const auto first = std::find_if_not(input.begin(), input.end(), [](unsigned char c) { return std::isspace(c) != 0; });
  if (first == input.end())
  {
    return "";
  }
  const auto last = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char c) { return std::isspace(c) != 0; }).base();
  return std::string(first, last);
}

std::vector<std::string> parse_list(const std::string & raw)
{
  std::string cleaned;
  cleaned.reserve(raw.size());
  for (char c : raw)
  {
    if (c != '[' && c != ']')
    {
      cleaned.push_back(c);
    }
  }

  std::vector<std::string> result;
  std::stringstream ss(cleaned);
  std::string item;
  while (std::getline(ss, item, ','))
  {
    item = trim(item);
    if (!item.empty())
    {
      result.push_back(item);
    }
  }
  return result;
}

bool parse_bool(const std::string & value)
{
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lowered == "1" || lowered == "true" || lowered == "yes" || lowered == "on" || lowered == "enable";
}

double parse_double(const hardware_interface::HardwareInfo & info, const std::string & key, double default_value, const rclcpp::Logger & logger)
{
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end())
  {
    return default_value;
  }

  try
  {
    return std::stod(it->second);
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(logger, "Failed to parse parameter '%s' as double: %s", key.c_str(), e.what());
    return default_value;
  }
}

int parse_int(const hardware_interface::HardwareInfo & info, const std::string & key, int default_value, const rclcpp::Logger & logger)
{
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end())
  {
    return default_value;
  }

  try
  {
    return std::stoi(it->second);
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(logger, "Failed to parse parameter '%s' as integer: %s", key.c_str(), e.what());
    return default_value;
  }
}

std::optional<std::string> get_param(const hardware_interface::HardwareInfo & info, const std::string & key)
{
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end())
  {
    return std::nullopt;
  }
  return trim(it->second);
}

const hardware_interface::ComponentInfo * find_joint(const hardware_interface::HardwareInfo & info, const std::string & name)
{
  auto it = std::find_if(info.joints.begin(), info.joints.end(), [&](const auto & joint) { return joint.name == name; });
  if (it == info.joints.end())
  {
    return nullptr;
  }
  return &(*it);
}

}  // namespace

MecaBridgeHardwareInterface::MecaBridgeHardwareInterface()
: comms_(rclcpp::get_logger("MecaBridgeComms")),
  logger_(rclcpp::get_logger("MecaBridgeHardwareInterface"))
{
}

hardware_interface::CallbackReturn MecaBridgeHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = rclcpp::get_logger("MecaBridgeHardwareInterface");
  comms_.set_logger(logger_);

  if (parse_configuration(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (configure_joints(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  comms_.configure(config_.serial);
  configured_ = true;

  zero_commands();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecaBridgeHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(4 * 2 + 1 + 1 + 2);

  for (auto & wheel : wheels_)
  {
    state_interfaces.emplace_back(wheel.name, hardware_interface::HW_IF_POSITION, &wheel.position);
    state_interfaces.emplace_back(wheel.name, hardware_interface::HW_IF_VELOCITY, &wheel.velocity);
  }

  state_interfaces.emplace_back(servo_180_.name, hardware_interface::HW_IF_POSITION, &servo_180_.position);
  state_interfaces.emplace_back(servo_360_.name, hardware_interface::HW_IF_VELOCITY, &servo_360_.velocity);

  for (auto & esc : escs_)
  {
    state_interfaces.emplace_back(esc.name, hardware_interface::HW_IF_EFFORT, &esc.state);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecaBridgeHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(4 + 1 + 1 + 2);

  for (auto & wheel : wheels_)
  {
    command_interfaces.emplace_back(wheel.name, hardware_interface::HW_IF_VELOCITY, &wheel.command);
  }

  command_interfaces.emplace_back(servo_180_.name, hardware_interface::HW_IF_POSITION, &servo_180_.command);
  command_interfaces.emplace_back(servo_360_.name, hardware_interface::HW_IF_VELOCITY, &servo_360_.command);

  for (auto & esc : escs_)
  {
    command_interfaces.emplace_back(esc.name, hardware_interface::HW_IF_EFFORT, &esc.command);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MecaBridgeHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating MecaBridge hardware interface");

  if (!ensure_connection())
  {
    RCLCPP_ERROR(logger_, "Unable to open communication with MecaBridge");
    return hardware_interface::CallbackReturn::ERROR;
  }

  zero_commands();
  safe_stop();

  last_state_time_ = std::chrono::steady_clock::now();
  last_heartbeat_time_ = last_state_time_;
  heartbeat_counter_ = 0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecaBridgeHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating MecaBridge hardware interface");

  safe_stop();
  comms_.disconnect();
  connected_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecaBridgeHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!ensure_connection())
  {
    RCLCPP_ERROR(logger_, "Read requested while not connected to MecaBridge");
    return hardware_interface::return_type::ERROR;
  }

  auto state = comms_.read_state(std::chrono::milliseconds(static_cast<int>(config_.state_timeout_ms)));
  if (!state)
  {
    auto now = std::chrono::steady_clock::now();
    if (now - last_state_time_ > std::chrono::milliseconds(static_cast<int>(config_.state_timeout_ms)))
    {
      RCLCPP_ERROR(logger_, "No state received within timeout - entering safe stop");
      safe_stop();
      connected_ = false;
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  last_state_time_ = std::chrono::steady_clock::now();

  for (size_t i = 0; i < wheels_.size(); ++i)
  {
    const double position = static_cast<double>(state->wheel_positions[i]);
    const double velocity = static_cast<double>(state->wheel_velocities[i]);
    wheels_[i].position = wheels_[i].inverted ? -position : position;
    wheels_[i].velocity = wheels_[i].inverted ? -velocity : velocity;
  }

  servo_180_.position = static_cast<double>(state->servo_position);
  servo_360_.velocity = static_cast<double>(state->servo_velocity);

  for (size_t i = 0; i < escs_.size(); ++i)
  {
    escs_[i].state = static_cast<double>(state->esc_states[i]);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecaBridgeHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  (void)period;

  if (!ensure_connection())
  {
    RCLCPP_ERROR(logger_, "Write requested while not connected to MecaBridge");
    return hardware_interface::return_type::ERROR;
  }

  enforce_limits();

  CommandPacket command;
  for (size_t i = 0; i < wheels_.size(); ++i)
  {
    const double cmd = wheels_[i].inverted ? -wheels_[i].command : wheels_[i].command;
    command.wheel_velocities[i] = static_cast<float>(cmd);
  }

  command.servo_position = static_cast<float>(servo_180_.command);
  command.servo_velocity = static_cast<float>(servo_360_.command);

  for (size_t i = 0; i < escs_.size(); ++i)
  {
    command.esc_commands[i] = static_cast<float>(escs_[i].command);
  }

  command.heartbeat = heartbeat_counter_++;
  command.flags = 0;

  if (!comms_.send_command(command))
  {
    RCLCPP_ERROR(logger_, "Failed to send actuator command frame");
    safe_stop();
    connected_ = false;
    return hardware_interface::return_type::ERROR;
  }

  const double heartbeat_period = config_.heartbeat_hz > 0.0 ? (1.0 / config_.heartbeat_hz) : 0.0;
  const auto now = std::chrono::steady_clock::now();
  if (heartbeat_period > 0.0 && (now - last_heartbeat_time_) >= std::chrono::duration<double>(heartbeat_period))
  {
    if (!comms_.send_heartbeat(heartbeat_counter_))
    {
      RCLCPP_WARN(logger_, "Failed to send heartbeat frame");
    }
    last_heartbeat_time_ = now;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MecaBridgeHardwareInterface::parse_configuration(const hardware_interface::HardwareInfo & info)
{
  auto wheel_names_opt = get_param(info, "wheel_joints");
  if (!wheel_names_opt)
  {
    RCLCPP_ERROR(logger_, "Parameter 'wheel_joints' is required");
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto wheel_names = parse_list(*wheel_names_opt);
  if (wheel_names.size() != wheels_.size())
  {
    RCLCPP_ERROR(logger_, "Expected %zu wheel joints, got %zu", wheels_.size(), wheel_names.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto wheel_inversions_opt = get_param(info, "wheel_inversions");
  std::vector<std::string> wheel_inversions = wheel_inversions_opt ? parse_list(*wheel_inversions_opt) : std::vector<std::string>{};

  const double wheel_radius = parse_double(info, "wheel_radius", 0.05, logger_);

  for (size_t i = 0; i < wheels_.size(); ++i)
  {
    config_.wheels[i].name = wheel_names[i];
    config_.wheels[i].radius = wheel_radius;
    config_.wheels[i].inverted = i < wheel_inversions.size() ? parse_bool(wheel_inversions[i]) : false;
  }

  config_.wheel_separation_x = parse_double(info, "wheel_separation_x", 0.3, logger_);
  config_.wheel_separation_y = parse_double(info, "wheel_separation_y", 0.3, logger_);
  config_.encoders.counts_per_rev = parse_int(info, "encoder_cpr", 0, logger_);

  auto servo180_opt = get_param(info, "servo_180_joint");
  auto servo360_opt = get_param(info, "servo_360_joint");
  if (!servo180_opt || !servo360_opt)
  {
    RCLCPP_ERROR(logger_, "Servo joint parameters 'servo_180_joint' and 'servo_360_joint' are required");
    return hardware_interface::CallbackReturn::ERROR;
  }

  config_.servo_180.name = *servo180_opt;
  config_.servo_360.name = *servo360_opt;

  const double servo_180_min_deg = parse_double(info, "servo_180_min_deg", -90.0, logger_);
  const double servo_180_max_deg = parse_double(info, "servo_180_max_deg", 90.0, logger_);
  config_.servo_180.min_position = servo_180_min_deg * DEG_TO_RAD;
  config_.servo_180.max_position = servo_180_max_deg * DEG_TO_RAD;

  const double servo_360_max_deg_per_sec = parse_double(info, "servo_360_max_deg_per_sec", 360.0, logger_);
  config_.servo_360.max_velocity = servo_360_max_deg_per_sec * DEG_TO_RAD;

  auto esc_names_opt = get_param(info, "esc_joints");
  if (!esc_names_opt)
  {
    RCLCPP_ERROR(logger_, "Parameter 'esc_joints' is required");
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto esc_names = parse_list(*esc_names_opt);
  if (esc_names.size() != escs_.size())
  {
    RCLCPP_ERROR(logger_, "Expected %zu ESC joints, got %zu", escs_.size(), esc_names.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const double esc_min = parse_double(info, "esc_min_command", 0.0, logger_);
  const double esc_max = parse_double(info, "esc_max_command", 1.0, logger_);

  for (size_t i = 0; i < escs_.size(); ++i)
  {
    config_.escs[i].name = esc_names[i];
    config_.escs[i].min_command = esc_min;
    config_.escs[i].max_command = esc_max;
  }

  config_.serial.device = get_param(info, "device").value_or("");
  config_.serial.baud_rate = parse_int(info, "baud_rate", 115200, logger_);
  config_.serial.timeout_ms = parse_int(info, "timeout_ms", 50, logger_);

  config_.heartbeat_hz = parse_double(info, "heartbeat_hz", 10.0, logger_);
  config_.state_timeout_ms = parse_double(info, "state_timeout_ms", 200.0, logger_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecaBridgeHardwareInterface::configure_joints(const hardware_interface::HardwareInfo & info)
{
  for (size_t i = 0; i < wheels_.size(); ++i)
  {
    wheels_[i].name = config_.wheels[i].name;
    wheels_[i].inverted = config_.wheels[i].inverted;

    const auto * joint = find_joint(info, wheels_[i].name);
    if (!joint)
    {
      RCLCPP_ERROR(logger_, "Wheel joint '%s' not found in hardware info", wheels_[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (std::find(joint->command_interfaces.begin(), joint->command_interfaces.end(), hardware_interface::HW_IF_VELOCITY) == joint->command_interfaces.end())
    {
      RCLCPP_ERROR(logger_, "Wheel joint '%s' must provide velocity command interface", wheels_[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  servo_180_.name = config_.servo_180.name;
  servo_180_.min_limit = config_.servo_180.min_position;
  servo_180_.max_limit = config_.servo_180.max_position;

  const auto * servo180_joint = find_joint(info, servo_180_.name);
  if (!servo180_joint)
  {
    RCLCPP_ERROR(logger_, "Servo joint '%s' not found", servo_180_.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (std::find(servo180_joint->command_interfaces.begin(), servo180_joint->command_interfaces.end(), hardware_interface::HW_IF_POSITION) == servo180_joint->command_interfaces.end())
  {
    RCLCPP_ERROR(logger_, "Servo joint '%s' must provide position command interface", servo_180_.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  servo_360_.name = config_.servo_360.name;
  servo_360_.max_velocity = config_.servo_360.max_velocity;

  const auto * servo360_joint = find_joint(info, servo_360_.name);
  if (!servo360_joint)
  {
    RCLCPP_ERROR(logger_, "Continuous servo joint '%s' not found", servo_360_.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (std::find(servo360_joint->command_interfaces.begin(), servo360_joint->command_interfaces.end(), hardware_interface::HW_IF_VELOCITY) == servo360_joint->command_interfaces.end())
  {
    RCLCPP_ERROR(logger_, "Continuous servo joint '%s' must provide velocity command interface", servo_360_.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < escs_.size(); ++i)
  {
    escs_[i].name = config_.escs[i].name;
    escs_[i].min_command = config_.escs[i].min_command;
    escs_[i].max_command = config_.escs[i].max_command;

    const auto * esc_joint = find_joint(info, escs_[i].name);
    if (!esc_joint)
    {
      RCLCPP_ERROR(logger_, "ESC joint '%s' not found", escs_[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (std::find(esc_joint->command_interfaces.begin(), esc_joint->command_interfaces.end(), hardware_interface::HW_IF_EFFORT) == esc_joint->command_interfaces.end())
    {
      RCLCPP_ERROR(logger_, "ESC joint '%s' must provide effort command interface", escs_[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

void MecaBridgeHardwareInterface::enforce_limits()
{
  servo_180_.command = std::max(servo_180_.min_limit, std::min(servo_180_.max_limit, servo_180_.command));

  if (servo_360_.max_velocity > 0.0)
  {
    const double limit = servo_360_.max_velocity;
    servo_360_.command = std::max(-limit, std::min(limit, servo_360_.command));
  }

  for (auto & esc : escs_)
  {
    esc.command = std::max(esc.min_command, std::min(esc.max_command, esc.command));
  }
}

void MecaBridgeHardwareInterface::zero_commands()
{
  for (auto & wheel : wheels_)
  {
    wheel.command = 0.0;
  }
  servo_180_.command = servo_180_.position;
  servo_360_.command = 0.0;
  for (auto & esc : escs_)
  {
    esc.command = 0.0;
  }
}

void MecaBridgeHardwareInterface::safe_stop()
{
  CommandPacket stop;
  for (auto & value : stop.wheel_velocities)
  {
    value = 0.0f;
  }
  stop.servo_position = static_cast<float>(servo_180_.position);
  stop.servo_velocity = 0.0f;
  stop.esc_commands = {0.0f, 0.0f};
  stop.flags = 0x01;  // Requesting safe stop mode on Pico

  if (comms_.connected())
  {
    comms_.send_command(stop);
    comms_.request_safe_stop();
  }

  zero_commands();
}

bool MecaBridgeHardwareInterface::ensure_connection()
{
  if (comms_.connected())
  {
    connected_ = true;
    return true;
  }

  if (!configured_)
  {
    comms_.configure(config_.serial);
  }

  if (!comms_.connect())
  {
    connected_ = false;
    return false;
  }

  connected_ = true;
  return true;
}

}  // namespace mecabridge

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mecabridge::MecaBridgeHardwareInterface, hardware_interface::SystemInterface)
