#pragma once

// Wrapper header to expose package-prefixed include path
// "mecabridge_hardware/mecabridge_utils/protocol/crc16.hpp"
// and provide a legacy unqualified symbol for tests that expect
// crc16_ccitt_false to live in the global namespace.

#include "mecabridge_utils/protocol/crc16.hpp"

// Inline forwarding function in the global namespace so older test code that
// calls crc16_ccitt_false(...) without qualification still links and compiles.
inline uint16_t crc16_ccitt_false(const uint8_t * data, size_t len)
{
  return ::mecabridge::protocol::crc16_ccitt_false(data, len);
}
