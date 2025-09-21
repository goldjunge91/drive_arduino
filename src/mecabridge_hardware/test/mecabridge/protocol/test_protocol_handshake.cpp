#include <gtest/gtest.h>
#include "mecabridge_utils/protocol/frame.hpp"

using namespace mecabridge::protocol;

TEST(ProtocolHandshakeTest, VersionMismatch) {
  // 1. Create a state frame with a mismatched protocol version
  StateFramePayload state_payload = {};   // Zero-initialize
  state_payload.protocol_version = PROTOCOL_VERSION + 1;   // Mismatch

  // 2. Encode this frame into a buffer
  uint8_t buffer[128];
  size_t bytes_written = 0;
  ErrorCode err = encodeState(state_payload, buffer, sizeof(buffer), bytes_written);

  ASSERT_EQ(err, ErrorCode::OK);
  ASSERT_GT(bytes_written, 0);

  // 3. Attempt to parse the frame
  ParsedFrame parsed_frame;
  ByteSpan input_span(buffer, bytes_written);
  ParseResult parse_res = tryParseFrame(input_span, parsed_frame);

  // 4. Assert that the parse fails with a version mismatch error
  ASSERT_EQ(parse_res, ParseResult::VERSION_MISMATCH);
}

TEST(ProtocolHandshakeTest, VersionMatch) {
  // 1. Create a state frame with a matching protocol version
  StateFramePayload state_payload = {};   // Zero-initialize
  // The protocol version is set automatically by encodeState

  // 2. Encode this frame into a buffer
  uint8_t buffer[128];
  size_t bytes_written = 0;
  ErrorCode err = encodeState(state_payload, buffer, sizeof(buffer), bytes_written);

  ASSERT_EQ(err, ErrorCode::OK);
  ASSERT_GT(bytes_written, 0);

  // 3. Attempt to parse the frame
  ParsedFrame parsed_frame;
  ByteSpan input_span(buffer, bytes_written);
  ParseResult parse_res = tryParseFrame(input_span, parsed_frame);

  // 4. Assert that the parse succeeds
  ASSERT_EQ(parse_res, ParseResult::SUCCESS);
}
