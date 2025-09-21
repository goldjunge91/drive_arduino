#ifndef MECABRIDGE_MECABRIDGE_COMMS_H
#define MECABRIDGE_MECABRIDGE_COMMS_H

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logger.hpp"
#include "serial/serial.h"

namespace mecabridge
{

struct CommandPacket
{
  std::array<float, 4> wheel_velocities{};  // rad/s
  float servo_position = 0.0f;              // radians
  float servo_velocity = 0.0f;              // rad/s for continuous servo
  std::array<float, 2> esc_commands{};      // normalised 0..1 or configured range
  uint8_t heartbeat = 0;                    // rolling counter
  uint8_t flags = 0;                        // bitfield for safety/feature toggles
};

struct StatePacket
{
  std::array<float, 4> wheel_positions{};   // radians
  std::array<float, 4> wheel_velocities{};  // rad/s
  std::array<int32_t, 4> encoder_counts{};  // raw counts
  float servo_position = 0.0f;              // radians
  float servo_velocity = 0.0f;              // rad/s
  std::array<float, 2> esc_states{};        // reported throttle values
  uint8_t status = 0;                       // bitfield for pico status flags
  uint8_t heartbeat = 0;                    // last heartbeat counter received
};

struct SerialConfig
{
  std::string device;
  int baud_rate = 115200;
  int timeout_ms = 50;
};

class MecaBridgeComms
{
public:
  explicit MecaBridgeComms(rclcpp::Logger logger = rclcpp::get_logger("MecaBridgeComms"));

  void configure(const SerialConfig & config);

  bool connect();

  void disconnect();

  bool connected() const;

  bool send_command(const CommandPacket & command);

  bool send_heartbeat(uint8_t heartbeat_counter);

  std::optional<StatePacket> read_state(std::chrono::milliseconds timeout);

  void request_safe_stop();

  void set_logger(rclcpp::Logger logger);

private:
  static constexpr uint8_t START_BYTE = 0xAA;
  static constexpr uint8_t FRAME_COMMAND = 0x01;
  static constexpr uint8_t FRAME_STATE = 0x02;
  static constexpr uint8_t FRAME_HEARTBEAT = 0x03;
  static constexpr uint8_t FRAME_STOP = 0x7E;

  SerialConfig config_{};
  serial::Serial serial_;
  bool configured_ = false;
  rclcpp::Logger logger_;

  bool write_frame(uint8_t id, const std::vector<uint8_t> & payload);

  std::optional<std::pair<uint8_t, std::vector<uint8_t>>> read_frame(std::chrono::milliseconds timeout);

  static void append_float(std::vector<uint8_t> & buffer, float value);
  static void append_u8(std::vector<uint8_t> & buffer, uint8_t value);
  static float read_float(const uint8_t * data);
  static int32_t read_i32(const uint8_t * data);

  static uint16_t compute_crc(const std::vector<uint8_t> & data);
};

}  // namespace mecabridge

#endif  // MECABRIDGE_MECABRIDGE_COMMS_H
