#ifndef MECABRIDGE_HARDWARE__MECABRIDGE_UTILS__SERIAL__MOCK_SERIAL_BACKEND__HPP_
#define MECABRIDGE_HARDWARE__MECABRIDGE_UTILS__SERIAL__MOCK_SERIAL_BACKEND__HPP_


#pragma once

#include "serial_backend.hpp"

namespace mecabridge
{
namespace serial
{

class MockSerialBackend : public SerialBackend
{
public:
  MockSerialBackend()
  : is_open_(false) {}
  ~MockSerialBackend() override = default;

  bool open(const SerialOptions & opts) override
  {
    (void)opts; // Suppress unused parameter warning
    is_open_ = true;
    return true;
  }

  void close() override
  {
    is_open_ = false;
  }

  bool is_open() const override
  {
    return is_open_;
  }

  int read(uint8_t * buf, size_t len) override
  {
    (void)buf; // Suppress unused parameter warning
    (void)len;
    return 0; // No data available
  }

  int write(const uint8_t * buf, size_t len) override
  {
    (void)buf; // Suppress unused parameter warning
    return static_cast<int>(len); // Pretend all data was written
  }

private:
  bool is_open_;
};

} // namespace serial
} // namespace mecabridge
#endif  // MECABRIDGE_HARDWARE__MECABRIDGE_UTILS__SERIAL__MOCK_SERIAL_BACKEND__HPP_
