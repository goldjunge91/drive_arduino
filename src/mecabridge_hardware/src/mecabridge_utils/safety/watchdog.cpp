#include "watchdog.hpp"
#include "mecabridge_utils/protocol/frame.hpp"

namespace mecabridge
{
namespace safety
{

Watchdog::Watchdog(std::chrono::milliseconds timeout)
: timeout_(timeout)
{
  reset();
}

void Watchdog::updateOnValidFrame(TimePoint now)
{
  last_valid_frame_time_ = now;
  tripped_ = false;
}

bool Watchdog::tripped(TimePoint now)
{
  if (tripped_) {
    return true;
  }

  if ((now - last_valid_frame_time_) > timeout_) {
    tripped_ = true;
  }

  return tripped_;
}

void Watchdog::reset()
{
  last_valid_frame_time_ = std::chrono::steady_clock::now();
  tripped_ = false;
}

void Watchdog::reset(TimePoint now)
{
  last_valid_frame_time_ = now;
  tripped_ = false;
}

uint16_t Watchdog::flags()
{
  if (tripped_) {
    return static_cast<uint16_t>(protocol::StatusFlags::WATCHDOG_TRIGGERED);
  }
  return 0;
}

} // namespace safety
} // namespace mecabridge
