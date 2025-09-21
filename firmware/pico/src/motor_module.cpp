#include "mecabridge_pico/modules/motor_module.hpp"

#include <algorithm>
#include <cstddef>

#ifdef MECABRIDGE_ENABLE_MOTORS
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#endif

namespace mecabridge_pico::modules
{

void MotorModule::init()
{
#ifdef MECABRIDGE_ENABLE_MOTORS
  // Configure PWM slices for each wheel motor here. Pin assignments are left to the
  // integrator; the stub simply ensures that the PWM hardware is initialised.
  for (int slice = 0; slice < 4; ++slice)
  {
    uint pwm_slice = static_cast<uint>(slice);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f);
    pwm_init(pwm_slice, &config, true);
    pwm_set_enabled(pwm_slice, true);
  }
#endif
  stop();
}

void MotorModule::set_velocity(size_t index, float radians_per_second)
{
  if (index >= command_velocity_.size())
  {
    return;
  }
  command_velocity_[index] = radians_per_second;
}

void MotorModule::stop()
{
  command_velocity_.fill(0.0f);
  actual_velocity_.fill(0.0f);
}

void MotorModule::apply()
{
#ifdef MECABRIDGE_ENABLE_MOTORS
  // Translate desired velocity into PWM duty cycle. This placeholder maps the
  // velocity to a 0-1 range and writes it to the PWM level register.
  for (size_t i = 0; i < command_velocity_.size(); ++i)
  {
    float command = std::clamp(command_velocity_[i], -1.0f, 1.0f);
    actual_velocity_[i] = command;
    uint16_t level = static_cast<uint16_t>((command * 0.5f + 0.5f) * 0xFFFF);
    pwm_set_gpio_level(static_cast<uint>(i), level);
  }
#else
  actual_velocity_ = command_velocity_;
#endif
}

void MotorModule::step(float dt_seconds)
{
  for (size_t i = 0; i < position_radians_.size(); ++i)
  {
    position_radians_[i] += actual_velocity_[i] * dt_seconds;
  }
}

}  // namespace mecabridge_pico::modules
