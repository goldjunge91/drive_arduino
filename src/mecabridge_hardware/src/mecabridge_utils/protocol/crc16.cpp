#include "crc16.hpp"

namespace mecabridge
{
namespace protocol
{

CRC16::CRC16()
: crc_(INIT_VALUE)
{
}

void CRC16::reset()
{
  crc_ = INIT_VALUE;
}

void CRC16::update(uint8_t byte)
{
  // CRC-16/CCITT-FALSE algorithm implementation
  crc_ ^= static_cast<uint16_t>(byte) << 8;
  for (int b = 0; b < 8; ++b) {
    if (crc_ & 0x8000) {
      crc_ = (crc_ << 1) ^ POLYNOMIAL;
    } else {
      crc_ <<= 1;
    }
  }
}

void CRC16::update(const uint8_t * data, size_t len)
{
  for (size_t i = 0; i < len; ++i) {
    update(data[i]);
  }
}

uint16_t CRC16::finalize() const
{
  return crc_;
}

uint16_t crc16_ccitt_false(const uint8_t * data, size_t len)
{
  if (data == nullptr || len == 0) {
    return CRC16::INIT_VALUE;
  }

  CRC16 crc;
  crc.update(data, len);
  return crc.finalize();
}

} // namespace protocol
} // namespace mecabridge
