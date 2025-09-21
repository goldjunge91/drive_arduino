#ifndef MECABRIDGE_PICO_SERVO_MODULE_HPP
#define MECABRIDGE_PICO_SERVO_MODULE_HPP

namespace mecabridge_pico::modules
{

class Servo180Module
{
public:
  void init();
  void set_limits(float min_rad, float max_rad);
  void set_target(float rad);
  void step(float dt_seconds);
  float position() const { return position_; }
  float command() const { return command_; }

private:
  float min_ = -1.5708f;  // -90 degrees
  float max_ = 1.5708f;   // 90 degrees
  float position_ = 0.0f;
  float command_ = 0.0f;
};

class Servo360Module
{
public:
  void init();
  void set_max_velocity(float rad_per_sec);
  void set_velocity(float rad_per_sec);
  void step(float dt_seconds);
  float velocity() const { return velocity_; }
  float position() const { return position_; }
  float command() const { return command_; }

private:
  float max_velocity_ = 10.0f;
  float velocity_ = 0.0f;
  float position_ = 0.0f;
  float command_ = 0.0f;
};

}  // namespace mecabridge_pico::modules

#endif  // MECABRIDGE_PICO_SERVO_MODULE_HPP
