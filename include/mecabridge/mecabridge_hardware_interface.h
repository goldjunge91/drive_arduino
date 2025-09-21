#ifndef MECABRIDGE_MECABRIDGE_HARDWARE_INTERFACE_H
#define MECABRIDGE_MECABRIDGE_HARDWARE_INTERFACE_H

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mecabridge/mecabridge_comms.h"

namespace mecabridge
{

struct WheelConfig
{
  std::string name;
  double radius = 0.05;
  bool inverted = false;
};

struct ServoConfig
{
  std::string name;
  double min_position = 0.0;  // radians
  double max_position = 0.0;  // radians
  double max_velocity = 0.0;  // radians per second for continuous servo
};

struct EscConfig
{
  std::string name;
  double min_command = 0.0;
  double max_command = 1.0;
};

struct EncoderConfig
{
  int counts_per_rev = 0;
};

struct BridgeConfig
{
  SerialConfig serial;
  std::array<WheelConfig, 4> wheels;
  ServoConfig servo_180;
  ServoConfig servo_360;
  std::array<EscConfig, 2> escs;
  EncoderConfig encoders;
  double wheel_separation_x = 0.3;
  double wheel_separation_y = 0.3;
  double heartbeat_hz = 10.0;
  double state_timeout_ms = 200.0;
};

class MecaBridgeHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecaBridgeHardwareInterface);

  MecaBridgeHardwareInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct WheelState
  {
    std::string name;
    double position = 0.0;
    double velocity = 0.0;
    double command = 0.0;
    bool inverted = false;
  };

  struct ServoState
  {
    std::string name;
    double position = 0.0;
    double velocity = 0.0;
    double command = 0.0;
    double min_limit = 0.0;
    double max_limit = 0.0;
    double max_velocity = 0.0;
  };

  struct EscState
  {
    std::string name;
    double state = 0.0;
    double command = 0.0;
    double min_command = 0.0;
    double max_command = 1.0;
  };

  hardware_interface::CallbackReturn parse_configuration(const hardware_interface::HardwareInfo & info);

  hardware_interface::CallbackReturn configure_joints(const hardware_interface::HardwareInfo & info);

  void enforce_limits();

  void zero_commands();

  void safe_stop();

  bool ensure_connection();

  BridgeConfig config_{};
  std::array<WheelState, 4> wheels_{};
  ServoState servo_180_{};
  ServoState servo_360_{};
  std::array<EscState, 2> escs_{};

  MecaBridgeComms comms_;

  rclcpp::Logger logger_;

  bool connected_ = false;
  uint8_t heartbeat_counter_ = 0;
  std::chrono::steady_clock::time_point last_state_time_{};
  std::chrono::steady_clock::time_point last_heartbeat_time_{};
  bool configured_ = false;
};

}  // namespace mecabridge

#endif  // MECABRIDGE_MECABRIDGE_HARDWARE_INTERFACE_H
