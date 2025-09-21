#ifndef MECABRIDGE_HARDWARE__MECABRIDGE_UTILS__SAFETY__WATCHDOG__HPP_
#define MECABRIDGE_HARDWARE__MECABRIDGE_UTILS__SAFETY__WATCHDOG__HPP_


#pragma once

#include <chrono>
#include <cstdint>

namespace mecabridge
{
namespace safety
{

class Watchdog
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  explicit Watchdog(std::chrono::milliseconds timeout = std::chrono::milliseconds(150));

  void updateOnValidFrame(TimePoint now);
  bool tripped(TimePoint now);
  void reset();
  void reset(TimePoint now);
  uint16_t flags();

private:
  std::chrono::milliseconds timeout_;
  TimePoint last_valid_frame_time_;
  bool tripped_ = false;
};

} // namespace safety
} // namespace mecabridge
#endif  // MECABRIDGE_HARDWARE__MECABRIDGE_UTILS__SAFETY__WATCHDOG__HPP_
