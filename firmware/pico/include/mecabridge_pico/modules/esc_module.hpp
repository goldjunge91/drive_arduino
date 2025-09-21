#ifndef MECABRIDGE_PICO_ESC_MODULE_HPP
#define MECABRIDGE_PICO_ESC_MODULE_HPP

#include <array>

namespace mecabridge_pico::modules
{

class EscModule
{
public:
  void init();
  void set_command(size_t index, float value);
  void set_range(float min_value, float max_value);
  void step();
  void stop();

  const std::array<float, 2> & command() const { return command_; }

private:
  std::array<float, 2> command_{};
  float min_ = 0.0f;
  float max_ = 1.0f;
};

}  // namespace mecabridge_pico::modules

#endif  // MECABRIDGE_PICO_ESC_MODULE_HPP
