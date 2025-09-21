#ifndef MECABRIDGE_PICO_MOTOR_MODULE_HPP
#define MECABRIDGE_PICO_MOTOR_MODULE_HPP

#include <array>

namespace mecabridge_pico::modules
{

class MotorModule
{
public:
  void init();
  void set_velocity(size_t index, float radians_per_second);
  void stop();
  void apply();
  void step(float dt_seconds);

  const std::array<float, 4> & command() const { return command_velocity_; }
  const std::array<float, 4> & position() const { return position_radians_; }
  const std::array<float, 4> & velocity() const { return actual_velocity_; }

private:
  std::array<float, 4> command_velocity_{};
  std::array<float, 4> actual_velocity_{};
  std::array<float, 4> position_radians_{};
};

}  // namespace mecabridge_pico::modules

#endif  // MECABRIDGE_PICO_MOTOR_MODULE_HPP
