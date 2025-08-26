#ifndef TESTABLE_TB6612_COMMS_H
#define TESTABLE_TB6612_COMMS_H

#include <string>
#include <vector>
#include <memory>
#include "mock_serial.h"

namespace tb6612_hardware
{

// Testable version of TB6612Comms that uses dependency injection
class TestableTB6612Comms
{
public:
  TestableTB6612Comms();
  TestableTB6612Comms(std::shared_ptr<MockSerial> mock_serial);

  void setMockSerial(std::shared_ptr<MockSerial> mock_serial);
  void setup(const std::string & serial_device, int32_t baud_rate, int32_t timeout_ms);
  void sendPing();

  // Motor command methods for different drive types
  void setDifferentialMotors(int left_val, int right_val);
  void setFourMotors(int fl, int fr, int rl, int rr);

  // Encoder reading methods (optional)
  void readDifferentialEncoders(int & left_enc, int & right_enc);
  void readFourEncoders(int & fl, int & fr, int & rl, int & rr);

  bool connected() const;

  // Connection recovery
  bool attemptReconnection();

  // Error statistics and diagnostics
  void getConnectionStats(int & write_errors, int & read_errors, int & reconnection_attempts) const;
  void resetErrorCounters();
  
private:
  std::shared_ptr<MockSerial> mock_serial_;
  
  std::string findSerialPort(const std::string &preferred_port);
  void sendMsg(const std::string &msg);
  std::string sendMsgWithResponse(const std::string &msg);
  
  // Connection parameters for reconnection
  std::string last_device_;
  int32_t last_baud_rate_;
  int32_t last_timeout_ms_;
  
  // Serial port candidates for auto-detection
  const std::vector<std::string> SERIAL_PORT_CANDIDATES = {
    "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"
  };
  
  // Error tracking
  mutable int write_error_count_;
  mutable int read_error_count_;
  mutable int reconnection_attempts_;
  
  // Connection state
  bool is_connected_;
};

}  // namespace tb6612_hardware

#endif  // TESTABLE_TB6612_COMMS_H