#ifndef MECABRIDGE_PICO_FRAME_HPP
#define MECABRIDGE_PICO_FRAME_HPP

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace mecabridge_pico
{

constexpr uint8_t START_BYTE = 0xAA;
constexpr uint8_t FRAME_COMMAND = 0x01;
constexpr uint8_t FRAME_STATE = 0x02;
constexpr uint8_t FRAME_HEARTBEAT = 0x03;
constexpr uint8_t FRAME_STOP = 0x7E;

struct CommandFrame
{
  std::array<float, 4> wheel_velocities{};
  float servo_position = 0.0f;
  float servo_velocity = 0.0f;
  std::array<float, 2> esc_commands{};
  uint8_t heartbeat = 0;
  uint8_t flags = 0;
};

struct StateFrame
{
  std::array<float, 4> wheel_positions{};
  std::array<float, 4> wheel_velocities{};
  std::array<int32_t, 4> encoder_counts{};
  float servo_position = 0.0f;
  float servo_velocity = 0.0f;
  std::array<float, 2> esc_states{};
  uint8_t status = 0;
  uint8_t heartbeat = 0;
};

uint16_t compute_crc(const uint8_t * data, size_t length);

std::vector<uint8_t> pack_command(uint8_t id, const CommandFrame & command);
std::vector<uint8_t> pack_state(uint8_t id, const StateFrame & state);

struct ParsedFrame
{
  uint8_t id = 0;
  std::vector<uint8_t> payload;
};

class FrameParser
{
public:
  std::optional<ParsedFrame> consume(uint8_t byte);
  void reset();

private:
  enum class State
  {
    WaitingForStart,
    ReadingHeader,
    ReadingPayload,
    ReadingCrc,
  };

  State state_ = State::WaitingForStart;
  uint8_t id_ = 0;
  uint8_t length_ = 0;
  std::vector<uint8_t> buffer_;
  std::array<uint8_t, 2> crc_bytes_{};
  size_t crc_index_ = 0;
  size_t header_index_ = 0;
};

}  // namespace mecabridge_pico

#endif  // MECABRIDGE_PICO_FRAME_HPP
