#ifndef MECABRIDGE_HARDWARE__MECABRIDGE_UTILS__SERIAL__LOOPBACK_SERIAL_BACKEND__HPP_
#define MECABRIDGE_HARDWARE__MECABRIDGE_UTILS__SERIAL__LOOPBACK_SERIAL_BACKEND__HPP_


#pragma once

#include "serial_backend.hpp"
#include <queue>
#include <vector>
#include <mutex>

namespace mecabridge
{
namespace serial
{

/**
 * @brief Serial backend that provides loopback functionality for testing
 *
 * This backend simulates a connected device by echoing command frames
 * as state frames with appropriate transformations. Useful for integration
 * testing without physical hardware.
 */
class LoopbackSerialBackend : public SerialBackend
{
public:
  LoopbackSerialBackend();
  ~LoopbackSerialBackend() override = default;

  bool open(const SerialOptions & opts) override;
  void close() override;
  bool is_open() const override;

  int read(uint8_t * buf, size_t len) override;
  int write(const uint8_t * buf, size_t len) override;

  /**
   * @brief Add a pre-generated state frame to the input queue
   *
   * @param frame_data Raw frame bytes
   * @param frame_size Size of frame in bytes
   */
  void injectStateFrame(const uint8_t * frame_data, size_t frame_size);

  /**
   * @brief Clear all pending input data
   */
  void clearInput();

  /**
   * @brief Get the last command frame that was written
   *
   * @return Vector containing the last written frame bytes
   */
  std::vector<uint8_t> getLastCommandFrame() const;

private:
  mutable std::mutex mutex_;
  bool is_open_;
  std::queue<uint8_t> input_queue_;
  std::vector<uint8_t> last_command_frame_;

  // Convert command frame to state frame (for loopback simulation)
  void processCommandFrame(const uint8_t * data, size_t size);
};

} // namespace serial
} // namespace mecabridge
#endif  // MECABRIDGE_HARDWARE__MECABRIDGE_UTILS__SERIAL__LOOPBACK_SERIAL_BACKEND__HPP_
