#include "mecabridge_pico/frame.hpp"

#include <cstring>

namespace mecabridge_pico
{

namespace
{
constexpr uint16_t CRC_SEED = 0xFFFF;
constexpr uint16_t CRC_POLY = 0x1021;

void append_bytes(std::vector<uint8_t> & buffer, const void * data, size_t length)
{
  const auto * bytes = static_cast<const uint8_t *>(data);
  buffer.insert(buffer.end(), bytes, bytes + length);
}

}  // namespace

uint16_t compute_crc(const uint8_t * data, size_t length)
{
  uint16_t crc = CRC_SEED;
  for (size_t i = 0; i < length; ++i)
  {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit)
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

std::vector<uint8_t> pack_command(uint8_t id, const CommandFrame & command)
{
  std::vector<uint8_t> payload;
  payload.reserve(34);

  append_bytes(payload, command.wheel_velocities.data(), command.wheel_velocities.size() * sizeof(float));
  append_bytes(payload, &command.servo_position, sizeof(float));
  append_bytes(payload, &command.servo_velocity, sizeof(float));
  append_bytes(payload, command.esc_commands.data(), command.esc_commands.size() * sizeof(float));
  append_bytes(payload, &command.heartbeat, sizeof(uint8_t));
  append_bytes(payload, &command.flags, sizeof(uint8_t));

  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + 5);
  frame.push_back(START_BYTE);
  frame.push_back(id);
  frame.push_back(static_cast<uint8_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());

  std::vector<uint8_t> crc_input;
  crc_input.reserve(payload.size() + 2);
  crc_input.push_back(id);
  crc_input.push_back(static_cast<uint8_t>(payload.size()));
  crc_input.insert(crc_input.end(), payload.begin(), payload.end());

  const uint16_t crc = compute_crc(crc_input.data(), crc_input.size());
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return frame;
}

std::vector<uint8_t> pack_state(uint8_t id, const StateFrame & state)
{
  std::vector<uint8_t> payload;
  payload.reserve(66);

  append_bytes(payload, state.wheel_positions.data(), state.wheel_positions.size() * sizeof(float));
  append_bytes(payload, state.wheel_velocities.data(), state.wheel_velocities.size() * sizeof(float));
  append_bytes(payload, state.encoder_counts.data(), state.encoder_counts.size() * sizeof(int32_t));
  append_bytes(payload, &state.servo_position, sizeof(float));
  append_bytes(payload, &state.servo_velocity, sizeof(float));
  append_bytes(payload, state.esc_states.data(), state.esc_states.size() * sizeof(float));
  append_bytes(payload, &state.status, sizeof(uint8_t));
  append_bytes(payload, &state.heartbeat, sizeof(uint8_t));

  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + 5);
  frame.push_back(START_BYTE);
  frame.push_back(id);
  frame.push_back(static_cast<uint8_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());

  std::vector<uint8_t> crc_input;
  crc_input.reserve(payload.size() + 2);
  crc_input.push_back(id);
  crc_input.push_back(static_cast<uint8_t>(payload.size()));
  crc_input.insert(crc_input.end(), payload.begin(), payload.end());

  const uint16_t crc = compute_crc(crc_input.data(), crc_input.size());
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return frame;
}

std::optional<ParsedFrame> FrameParser::consume(uint8_t byte)
{
  switch (state_)
  {
    case State::WaitingForStart:
      if (byte == START_BYTE)
      {
        state_ = State::ReadingHeader;
        header_index_ = 0;
        buffer_.clear();
      }
      break;

    case State::ReadingHeader:
      if (header_index_ == 0)
      {
        id_ = byte;
        ++header_index_;
      }
      else
      {
        length_ = byte;
        buffer_.clear();
        buffer_.reserve(length_);
        header_index_ = 0;
        crc_index_ = 0;
        crc_bytes_.fill(0);
        state_ = length_ == 0 ? State::ReadingCrc : State::ReadingPayload;
      }
      break;

    case State::ReadingPayload:
      buffer_.push_back(byte);
      if (buffer_.size() >= length_)
      {
        crc_index_ = 0;
        crc_bytes_.fill(0);
        state_ = State::ReadingCrc;
      }
      break;

    case State::ReadingCrc:
      if (crc_index_ < crc_bytes_.size())
      {
        crc_bytes_[crc_index_++] = byte;
        if (crc_index_ >= crc_bytes_.size())
        {
          std::vector<uint8_t> crc_input;
          crc_input.reserve(buffer_.size() + 2);
          crc_input.push_back(id_);
          crc_input.push_back(length_);
          crc_input.insert(crc_input.end(), buffer_.begin(), buffer_.end());

          const uint16_t expected_crc = compute_crc(crc_input.data(), crc_input.size());
          const uint16_t received_crc = static_cast<uint16_t>(crc_bytes_[0]) |
                                        static_cast<uint16_t>(crc_bytes_[1]) << 8;

          state_ = State::WaitingForStart;

          if (expected_crc == received_crc)
          {
            return ParsedFrame{id_, buffer_};
          }
        }
      }
      break;
  }

  return std::nullopt;
}

void FrameParser::reset()
{
  state_ = State::WaitingForStart;
  header_index_ = 0;
  crc_index_ = 0;
  buffer_.clear();
}

}  // namespace mecabridge_pico
