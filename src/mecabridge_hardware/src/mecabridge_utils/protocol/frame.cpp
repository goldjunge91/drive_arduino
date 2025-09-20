#include "frame.hpp"
#include "crc16.hpp"
#include <cstring>
#include <algorithm>

namespace mecabridge {
namespace protocol {

ErrorCode encodeCommand(const CommandFramePayload& command, uint8_t* buffer, size_t buffer_size, size_t& bytes_written) {
    // TODO: Implement command frame encoding
    // Frame structure: START_BYTE + FRAME_ID + LEN + PAYLOAD + CRC16
    // Total size: 1 + 1 + 1 + 35 + 2 = 40 bytes
    
    bytes_written = 0;
    return ErrorCode::NOT_IMPLEMENTED;
}

ErrorCode encodeState(const StateFramePayload& state, uint8_t* buffer, size_t buffer_size, size_t& bytes_written) {
    // TODO: Implement state frame encoding
    // Frame structure: START_BYTE + FRAME_ID + LEN + PAYLOAD + CRC16
    // Total size: 1 + 1 + 1 + 39 + 2 = 44 bytes
    
    bytes_written = 0;
    return ErrorCode::NOT_IMPLEMENTED;
}

ParseResult tryParseFrame(ByteSpan input, ParsedFrame& result) {
    // TODO: Implement frame parsing state machine
    // 1. Check for START_BYTE (0xAA)
    // 2. Extract frame_id and len
    // 3. Read payload
    // 4. Verify CRC16
    
    result.result = ParseResult::UNKNOWN_ERROR;
    return ParseResult::UNKNOWN_ERROR;
}

bool validateCommandPayload(CommandFramePayload& command) {
    // TODO: Implement validation rules
    // - All normalized values within [-1.0, 1.0]
    // - servo_pos_rad within configured [min,max]
    
    return false;
}

void clampCommandValues(CommandFramePayload& command, uint16_t& flags_set) {
    // TODO: Implement clamping with flag setting
    // - Clamp normalized values to [-1.0, 1.0]
    // - Set appropriate flags when clamping occurs
    
    flags_set = 0;
}

} // namespace protocol
} // namespace mecabridge