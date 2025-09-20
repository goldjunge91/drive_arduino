#include <cstddef>
#include <cstdint>
#include <iostream>

struct StateFramePayload
{
  uint32_t encoder_counts[4];       // Raw absolute counters
  uint16_t dt_ms;                   // Elapsed ms since previous state frame
  float servo_pos_rad;              // Current servo position
  float servo_cont_vel_norm;        // Echo last commanded
  float esc_norm[2];                // Echo last commanded normalized
  uint16_t seq_echo;                // Last accepted COMMAND seq
  uint16_t flags;                   // Status flags bitfield
  uint8_t error_code;               // Recent error cause
  uint8_t protocol_version;         // Protocol version of the firmware
} __attribute__((packed));

int main()
{
  std::cout << "StateFramePayload size: " << sizeof(StateFramePayload) << std::endl;
  return 0;
}
