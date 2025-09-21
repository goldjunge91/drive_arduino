#include "mecabridge_pico/modules/servo_module.hpp"

#include <algorithm>

#ifdef MECABRIDGE_ENABLE_SERVOS
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#endif

namespace mecabridge_pico::modules
{

void Servo180Module::init()
{
#ifdef MECABRIDGE_ENABLE_SERVOS
  // Configure PWM for positional servo (typical 50 Hz signal)
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, 64.0f);
  pwm_init(0, &config, true);
  pwm_set_enabled(0, true);
#endif
  position_ = 0.0f;
  command_ = 0.0f;
}

void Servo180Module::set_limits(float min_rad, float max_rad)
{
  min_ = min_rad;
  max_ = max_rad;
}

void Servo180Module::set_target(float rad)
{
  command_ = std::clamp(rad, min_, max_);
}

void Servo180Module::step(float dt_seconds)
{
  (void)dt_seconds;
  position_ = command_;
#ifdef MECABRIDGE_ENABLE_SERVOS
  // Map radians to PWM microseconds (assume +-90° equals 1000-2000us)
  const float normalised = (command_ - min_) / (max_ - min_);
  const uint16_t pulse = static_cast<uint16_t>(1000 + normalised * 1000);
  pwm_set_gpio_level(0, pulse);
#endif
}

void Servo360Module::init()
{
#ifdef MECABRIDGE_ENABLE_SERVOS
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, 64.0f);
  pwm_init(1, &config, true);
  pwm_set_enabled(1, true);
#endif
  velocity_ = 0.0f;
  position_ = 0.0f;
  command_ = 0.0f;
}

void Servo360Module::set_max_velocity(float rad_per_sec)
{
  max_velocity_ = std::max(rad_per_sec, 0.0f);
}

void Servo360Module::set_velocity(float rad_per_sec)
{
  command_ = std::clamp(rad_per_sec, -max_velocity_, max_velocity_);
}

void Servo360Module::step(float dt_seconds)
{
  velocity_ = command_;
  position_ += velocity_ * dt_seconds;
#ifdef MECABRIDGE_ENABLE_SERVOS
  const float normalised = (command_ / max_velocity_ + 1.0f) * 0.5f;
  const uint16_t pulse = static_cast<uint16_t>(1500 + (normalised - 0.5f) * 1000);
  pwm_set_gpio_level(1, pulse);
#endif
}

}  // namespace mecabridge_pico::modules
