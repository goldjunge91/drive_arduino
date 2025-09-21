#include "mecabridge/mecabridge_comms.h"

#include <cstring>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

namespace mecabridge
{

namespace
{
constexpr uint16_t CRC_SEED = 0xFFFF;
constexpr uint16_t CRC_POLY = 0x1021;

constexpr size_t COMMAND_PAYLOAD_SIZE = 34;  // 4 * 4 + 2 * 4 + 2 * 4 + 2
constexpr size_t STATE_PAYLOAD_SIZE = 66;    // 4 * 4 + 4 * 4 + 4 * 4 + 2 * 4 + 2
}  // namespace

MecaBridgeComms::MecaBridgeComms(rclcpp::Logger logger)
: logger_(logger)
{
}

void MecaBridgeComms::configure(const SerialConfig & config)
{
  config_ = config;
  configured_ = true;
}

bool MecaBridgeComms::connect()
{
  if (!configured_)
  {
    RCLCPP_ERROR(logger_, "Serial configuration not set before connect()");
    return false;
  }

  try
  {
    if (config_.device.empty())
    {
      RCLCPP_ERROR(logger_, "No serial device specified for MecaBridge");
      return false;
    }

    if (serial_.isOpen())
    {
      serial_.close();
    }

    serial_.setPort(config_.device);
    serial_.setBaudrate(config_.baud_rate);
    serial_.setTimeout(serial::Timeout::simpleTimeout(config_.timeout_ms));
    serial_.open();

    if (!serial_.isOpen())
    {
      RCLCPP_ERROR(logger_, "Failed to open serial device %s", config_.device.c_str());
      return false;
    }

    RCLCPP_INFO(logger_, "Connected to MecaBridge on %s @ %d", config_.device.c_str(), config_.baud_rate);
    serial_.flush();
    return true;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(logger_, "Exception while opening serial port: %s", e.what());
    return false;
  }
}

void MecaBridgeComms::disconnect()
{
  if (serial_.isOpen())
  {
    try
    {
      serial_.close();
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(logger_, "Exception while closing serial port: %s", e.what());
    }
  }
}

bool MecaBridgeComms::connected() const
{
  return serial_.isOpen();
}

bool MecaBridgeComms::send_command(const CommandPacket & command)
{
  if (!connected())
  {
    return false;
  }

  std::vector<uint8_t> payload;
  payload.reserve(COMMAND_PAYLOAD_SIZE);

  for (const auto & value : command.wheel_velocities)
  {
    append_float(payload, value);
  }

  append_float(payload, command.servo_position);
  append_float(payload, command.servo_velocity);

  for (const auto & value : command.esc_commands)
  {
    append_float(payload, value);
  }

  append_u8(payload, command.heartbeat);
  append_u8(payload, command.flags);

  return write_frame(FRAME_COMMAND, payload);
}

bool MecaBridgeComms::send_heartbeat(uint8_t heartbeat_counter)
{
  if (!connected())
  {
    return false;
  }

  std::vector<uint8_t> payload;
  payload.reserve(1);
  append_u8(payload, heartbeat_counter);
  return write_frame(FRAME_HEARTBEAT, payload);
}

std::optional<StatePacket> MecaBridgeComms::read_state(std::chrono::milliseconds timeout)
{
  if (!connected())
  {
    return std::nullopt;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline)
  {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline)
    {
      break;
    }

    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    auto frame = read_frame(remaining);
    if (!frame)
    {
      return std::nullopt;
    }

    const uint8_t id = frame->first;
    const std::vector<uint8_t> & payload = frame->second;

    if (id == FRAME_HEARTBEAT)
    {
      // Heartbeat acknowledgement; skip but report success so caller can keep alive.
      continue;
    }

    if (id != FRAME_STATE)
    {
      RCLCPP_WARN(logger_, "Unexpected frame id 0x%02x while waiting for state", id);
      continue;
    }

    if (payload.size() < STATE_PAYLOAD_SIZE)
    {
      RCLCPP_WARN(logger_, "Incomplete state payload received: %zu bytes", payload.size());
      continue;
    }

    StatePacket state;
    size_t offset = 0;

    for (auto & position : state.wheel_positions)
    {
      position = read_float(payload.data() + offset);
      offset += sizeof(float);
    }

    for (auto & velocity : state.wheel_velocities)
    {
      velocity = read_float(payload.data() + offset);
      offset += sizeof(float);
    }

    for (auto & encoder : state.encoder_counts)
    {
      encoder = read_i32(payload.data() + offset);
      offset += sizeof(int32_t);
    }

    state.servo_position = read_float(payload.data() + offset);
    offset += sizeof(float);
    state.servo_velocity = read_float(payload.data() + offset);
    offset += sizeof(float);

    for (auto & esc : state.esc_states)
    {
      esc = read_float(payload.data() + offset);
      offset += sizeof(float);
    }

    if (offset + 2 <= payload.size())
    {
      state.status = payload[offset++];
      state.heartbeat = payload[offset++];
    }
    else
    {
      RCLCPP_WARN(logger_, "State payload shorter than expected (%zu)", payload.size());
    }

    return state;
  }

  return std::nullopt;
}

void MecaBridgeComms::request_safe_stop()
{
  if (!connected())
  {
    return;
  }

  std::vector<uint8_t> payload;
  write_frame(FRAME_STOP, payload);
}

void MecaBridgeComms::set_logger(rclcpp::Logger logger)
{
  logger_ = logger;
}

bool MecaBridgeComms::write_frame(uint8_t id, const std::vector<uint8_t> & payload)
{
  if (!serial_.isOpen())
  {
    return false;
  }

  if (payload.size() > 0xFF)
  {
    RCLCPP_ERROR(logger_, "Payload too large for frame: %zu bytes", payload.size());
    return false;
  }

  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + 5);
  frame.push_back(START_BYTE);
  frame.push_back(id);
  frame.push_back(static_cast<uint8_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());

  std::vector<uint8_t> crc_data;
  crc_data.reserve(payload.size() + 2);
  crc_data.push_back(id);
  crc_data.push_back(static_cast<uint8_t>(payload.size()));
  crc_data.insert(crc_data.end(), payload.begin(), payload.end());
  const uint16_t crc = compute_crc(crc_data);

  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

  try
  {
    const size_t written = serial_.write(frame);
    if (written != frame.size())
    {
      RCLCPP_WARN(logger_, "Incomplete frame write: %zu/%zu bytes", written, frame.size());
      return false;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(logger_, "Failed to write frame: %s", e.what());
    return false;
  }

  return true;
}

std::optional<std::pair<uint8_t, std::vector<uint8_t>>> MecaBridgeComms::read_frame(std::chrono::milliseconds timeout)
{
  if (!serial_.isOpen())
  {
    return std::nullopt;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline)
  {
    uint8_t start_byte = 0;
    try
    {
      if (serial_.read(&start_byte, 1) != 1)
      {
        continue;
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(logger_, "Serial read exception: %s", e.what());
      return std::nullopt;
    }

    if (start_byte != START_BYTE)
    {
      continue;
    }

    uint8_t header[2] = {0, 0};
    if (serial_.read(header, 2) != 2)
    {
      RCLCPP_WARN(logger_, "Failed to read frame header");
      continue;
    }

    const uint8_t id = header[0];
    const uint8_t length = header[1];

    std::vector<uint8_t> payload(length);
    if (length > 0)
    {
      if (serial_.read(payload.data(), length) != length)
      {
        RCLCPP_WARN(logger_, "Failed to read frame payload (expected %u bytes)", length);
        continue;
      }
    }

    uint8_t crc_bytes[2] = {0, 0};
    if (serial_.read(crc_bytes, 2) != 2)
    {
      RCLCPP_WARN(logger_, "Failed to read frame CRC");
      continue;
    }

    const uint16_t received_crc = static_cast<uint16_t>(crc_bytes[0]) | (static_cast<uint16_t>(crc_bytes[1]) << 8);

    std::vector<uint8_t> crc_data;
    crc_data.reserve(length + 2);
    crc_data.push_back(id);
    crc_data.push_back(length);
    crc_data.insert(crc_data.end(), payload.begin(), payload.end());

    const uint16_t computed_crc = compute_crc(crc_data);
    if (received_crc != computed_crc)
    {
      RCLCPP_WARN(logger_, "CRC mismatch: received 0x%04x, computed 0x%04x", received_crc, computed_crc);
      continue;
    }

    return std::make_pair(id, payload);
  }

  return std::nullopt;
}

void MecaBridgeComms::append_float(std::vector<uint8_t> & buffer, float value)
{
  static_assert(sizeof(float) == 4, "Float size must be 4 bytes");
  uint8_t bytes[sizeof(float)];
  std::memcpy(bytes, &value, sizeof(float));
  buffer.insert(buffer.end(), bytes, bytes + sizeof(float));
}

void MecaBridgeComms::append_u8(std::vector<uint8_t> & buffer, uint8_t value)
{
  buffer.push_back(value);
}

float MecaBridgeComms::read_float(const uint8_t * data)
{
  float value;
  std::memcpy(&value, data, sizeof(float));
  return value;
}

int32_t MecaBridgeComms::read_i32(const uint8_t * data)
{
  int32_t value;
  std::memcpy(&value, data, sizeof(int32_t));
  return value;
}

uint16_t MecaBridgeComms::compute_crc(const std::vector<uint8_t> & data)
{
  uint16_t crc = CRC_SEED;
  for (const auto byte : data)
  {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int i = 0; i < 8; ++i)
    {
      if ((crc & 0x8000) != 0)
      {
        crc = static_cast<uint16_t>((crc << 1) ^ CRC_POLY);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

}  // namespace mecabridge
