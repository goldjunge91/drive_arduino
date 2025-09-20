#pragma once

#include <cstdint>
#include <cstddef>

namespace mecabridge {
namespace protocol {

/**
 * CRC-16/CCITT-FALSE implementation
 * 
 * Specifications:
 * - Polynomial: 0x1021
 * - Init: 0xFFFF
 * - Reflect In: false
 * - Reflect Out: false
 * - XorOut: 0x0000
 * 
 * Test vector: "123456789" -> 0x29B1
 */

class CRC16 {
public:
    static constexpr uint16_t POLYNOMIAL = 0x1021;
    static constexpr uint16_t INIT_VALUE = 0xFFFF;
    
    CRC16();
    
    void reset();
    void update(uint8_t byte);
    void update(const uint8_t* data, size_t len);
    uint16_t finalize() const;
    
private:
    uint16_t crc_;
};

// Convenience function for one-shot calculation
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);

} // namespace protocol
} // namespace mecabridge