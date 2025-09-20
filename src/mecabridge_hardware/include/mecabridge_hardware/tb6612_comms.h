#ifndef TB6612_HARDWARE_TB6612_COMMS_H
#define TB6612_HARDWARE_TB6612_COMMS_H

#include <string>
#include <vector>
#include <serial/serial.h>

namespace tb6612_hardware
{

class TB6612Comms
{
public:
  TB6612Comms();
  TB6612Comms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  
  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  void sendPing();  // Sends "PING\n" for Arduino reset sync
  
  // Motor command methods for different drive types
  void setDifferentialMotors(int left_val, int right_val);  // "V {left} {right}\n"
  void setFourMotors(int fl, int fr, int rl, int rr);       // "M {fl} {fr} {rl} {rr}\n"
  
  // Encoder reading methods (optional)
  void readDifferentialEncoders(int &left_enc, int &right_enc);     // "E\n" -> "left right"
  void readFourEncoders(int &fl, int &fr, int &rl, int &rr);        // "E\n" -> "fl fr rl rr"
  
  bool connected() const;
  
  // Connection recovery
  bool attemptReconnection();
  
  // Testing support - allows injection of mock serial for unit tests
  #ifdef TESTING_MODE
  void setMockSerial(std::shared_ptr<class MockSerial> mock_serial);
  #endif
  
  // Error statistics and diagnostics
  void getConnectionStats(int& write_errors, int& read_errors, int& reconnection_attempts) const;
  void resetErrorCounters();
  
private:
  serial::Serial serial_conn_;
  
  #ifdef TESTING_MODE
  std::shared_ptr<class MockSerial> mock_serial_;
  bool use_mock_serial_;
  #endif
  
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
};

}  // namespace tb6612_hardware

#endif  // TB6612_HARDWARE_TB6612_COMMS_H