#include "mecabridge_pico/modules/esc_module.hpp"

#include <algorithm>

#ifdef MECABRIDGE_ENABLE_ESCS
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#endif

namespace mecabridge_pico::modules
{

void EscModule::init()
{
#ifdef MECABRIDGE_ENABLE_ESCS
  for (int slice = 2; slice < 4; ++slice)
  {
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.0f);
    pwm_init(static_cast<uint>(slice), &config, true);
    pwm_set_enabled(static_cast<uint>(slice), true);
  }
#endif
  stop();
}

void EscModule::set_command(size_t index, float value)
{
  if (index >= command_.size())
  {
    return;
  }
  command_[index] = std::clamp(value, min_, max_);
}

void EscModule::set_range(float min_value, float max_value)
{
  min_ = min_value;
  max_ = max_value;
}

void EscModule::step()
{
#ifdef MECABRIDGE_ENABLE_ESCS
  for (size_t i = 0; i < command_.size(); ++i)
  {
    const float range = std::max(max_ - min_, 0.0001f);
    const float normalised = (command_[i] - min_) / range;
    const uint16_t pulse = static_cast<uint16_t>(1000 + normalised * 1000);
    pwm_set_gpio_level(static_cast<uint>(i + 2), pulse);
  }
#endif
}

void EscModule::stop()
{
  command_.fill(min_);
}

}  // namespace mecabridge_pico::modules
