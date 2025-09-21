#ifndef MECABRIDGE_HARDWARE__MECABRIDGE_HARDWARE__WHEEL__H_
#define MECABRIDGE_HARDWARE__MECABRIDGE_HARDWARE__WHEEL__H_



#include <string>

namespace mecabridge_hardware
{

  class Wheel
  {
public:
    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double eff = 0;
    double velSetPt = 0;
    double rads_per_count = 0;

    Wheel() = default;

    Wheel(const std::string & wheel_name, int counts_per_rev);

    void setup(const std::string & wheel_name, int counts_per_rev);

    double calcEncAngle();
  };

}  // namespace mecabridge_hardware

#endif  // MECABRIDGE_HARDWARE__MECABRIDGE_HARDWARE__WHEEL__H_
