#include "pico/stdlib.h"

#include "mecabridge_pico/frame.hpp"
#include "mecabridge_pico/modules/esc_module.hpp"
#include "mecabridge_pico/modules/motor_module.hpp"
#include "mecabridge_pico/modules/servo_module.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

using mecabridge_pico::CommandFrame;
using mecabridge_pico::FrameParser;
using mecabridge_pico::ParsedFrame;
using mecabridge_pico::StateFrame;
using namespace mecabridge_pico::modules;

namespace
{
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 250;
constexpr uint32_t STATE_PERIOD_MS = 20;

CommandFrame decode_command(const std::vector<uint8_t> & payload)
{
  CommandFrame command;
  size_t offset = 0;
  auto read_float = [&](float & value) {
    std::memcpy(&value, payload.data() + offset, sizeof(float));
    offset += sizeof(float);
  };

  for (auto & velocity : command.wheel_velocities)
  {
    read_float(velocity);
  }

  read_float(command.servo_position);
  read_float(command.servo_velocity);

  for (auto & esc : command.esc_commands)
  {
    read_float(esc);
  }

  if (offset < payload.size())
  {
    command.heartbeat = payload[offset++];
  }
  if (offset < payload.size())
  {
    command.flags = payload[offset++];
  }
  return command;
}

void write_frame(const std::vector<uint8_t> & frame)
{
  for (uint8_t byte : frame)
  {
    putchar_raw(byte);
  }
}

}  // namespace

int main()
{
  stdio_init_all();

  MotorModule motors;
  Servo180Module servo180;
  Servo360Module servo360;
  EscModule escs;

  motors.init();
  servo180.init();
  servo360.init();
  escs.init();

  servo180.set_limits(-1.5708f, 1.5708f);
  servo360.set_max_velocity(10.0f);
  escs.set_range(0.0f, 1.0f);

  FrameParser parser;

  absolute_time_t last_command_time = get_absolute_time();
  absolute_time_t last_state_time = get_absolute_time();
  absolute_time_t last_step_time = get_absolute_time();

  uint8_t heartbeat_counter = 0;
  bool safe_stop_active = false;

  while (true)
  {
    const int ch = getchar_timeout_us(0);
    if (ch >= 0)
    {
      const auto maybe_frame = parser.consume(static_cast<uint8_t>(ch));
      if (maybe_frame)
      {
        const ParsedFrame & frame = *maybe_frame;
        if (frame.id == mecabridge_pico::FRAME_COMMAND && frame.payload.size() >= 34)
        {
          const CommandFrame command = decode_command(frame.payload);

          for (size_t i = 0; i < command.wheel_velocities.size(); ++i)
          {
            motors.set_velocity(i, command.wheel_velocities[i]);
          }
          motors.apply();

          servo180.set_target(command.servo_position);
          servo360.set_velocity(command.servo_velocity);

          for (size_t i = 0; i < command.esc_commands.size(); ++i)
          {
            escs.set_command(i, command.esc_commands[i]);
          }
          escs.step();

          last_command_time = get_absolute_time();
          safe_stop_active = (command.flags & 0x01) != 0;
        }
        else if (frame.id == mecabridge_pico::FRAME_STOP)
        {
          motors.stop();
          servo360.set_velocity(0.0f);
          escs.stop();
          safe_stop_active = true;
        }
      }
    }

    const absolute_time_t now = get_absolute_time();
    const float dt = static_cast<float>(absolute_time_diff_us(last_step_time, now)) / 1'000'000.0f;
    last_step_time = now;

    motors.step(dt);
    servo180.step(dt);
    servo360.step(dt);

    if (to_ms_since_boot(now) - to_ms_since_boot(last_command_time) > HEARTBEAT_TIMEOUT_MS)
    {
      motors.stop();
      servo360.set_velocity(0.0f);
      escs.stop();
      safe_stop_active = true;
      last_command_time = now;
    }

    if (to_ms_since_boot(now) - to_ms_since_boot(last_state_time) >= STATE_PERIOD_MS)
    {
      StateFrame state;
      const auto & positions = motors.position();
      const auto & velocities = motors.velocity();

      for (size_t i = 0; i < positions.size(); ++i)
      {
        state.wheel_positions[i] = positions[i];
        state.wheel_velocities[i] = velocities[i];
        state.encoder_counts[i] = static_cast<int32_t>(positions[i] * 1000.0f);
      }

      state.servo_position = servo180.position();
      state.servo_velocity = servo360.velocity();

      const auto & esc_cmds = escs.command();
      for (size_t i = 0; i < esc_cmds.size(); ++i)
      {
        state.esc_states[i] = esc_cmds[i];
      }

      state.status = safe_stop_active ? 0x01 : 0x00;
      state.heartbeat = heartbeat_counter++;

      write_frame(mecabridge_pico::pack_state(mecabridge_pico::FRAME_STATE, state));
      last_state_time = now;
    }
  }

  return 0;
}
