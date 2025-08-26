#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <string>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

// Mock serial library for testing
#include "mock_serial.h"
#include "testable_tb6612_comms.h"

using namespace tb6612_hardware;
using ::testing::_;
using ::testing::Return;
using ::testing::Throw;
using ::testing::StrictMock;
using ::testing::InSequence;

class TB6612CommsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS 2 for logging
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    
    // Create mock serial connection
    mock_serial_ = std::make_shared<StrictMock<MockSerial>>();
    
    // Create TestableTB6612Comms instance for testing
    comms_ = std::make_unique<TestableTB6612Comms>();
  }

  void TearDown() override
  {
    comms_.reset();
    mock_serial_.reset();
  }

  std::shared_ptr<StrictMock<MockSerial>> mock_serial_;
  std::unique_ptr<TestableTB6612Comms> comms_;
};

// Test serial port detection and connection logic
class TB6612CommsConnectionTest : public TB6612CommsTest
{
};

TEST_F(TB6612CommsConnectionTest, TestAutoDetectionWithValidPort)
{
  // Test that auto-detection finds the first available port
  EXPECT_CALL(*mock_serial_, setPort("/dev/ttyUSB0"));
  EXPECT_CALL(*mock_serial_, setBaudrate(115200));
  EXPECT_CALL(*mock_serial_, setTimeout(_));
  EXPECT_CALL(*mock_serial_, open());
  EXPECT_CALL(*mock_serial_, isOpen())
    .WillOnce(Return(true))
    .WillOnce(Return(true));
  EXPECT_CALL(*mock_serial_, write("PING\n"))
    .WillOnce(Return(5));

  // Inject mock into TestableTB6612Comms for testing
  comms_->setMockSerial(mock_serial_);
  
  EXPECT_NO_THROW(comms_->setup("", 115200, 50));
  EXPECT_TRUE(comms_->connected());
}

TEST_F(TB6612CommsConnectionTest, TestAutoDetectionWithMultiplePorts)
{
  // Test that auto-detection tries multiple ports until one works
  {
    InSequence seq;
    
    // First port fails
    EXPECT_CALL(*mock_serial_, setPort("/dev/ttyUSB0"));
    EXPECT_CALL(*mock_serial_, setBaudrate(115200));
    EXPECT_CALL(*mock_serial_, setTimeout(_));
    EXPECT_CALL(*mock_serial_, open())
      .WillOnce(Throw(std::runtime_error("Port not available")));
    
    // Second port succeeds
    EXPECT_CALL(*mock_serial_, setPort("/dev/ttyUSB1"));
    EXPECT_CALL(*mock_serial_, setBaudrate(115200));
    EXPECT_CALL(*mock_serial_, setTimeout(_));
    EXPECT_CALL(*mock_serial_, open());
    EXPECT_CALL(*mock_serial_, isOpen())
      .WillOnce(Return(true))
      .WillOnce(Return(true));
    EXPECT_CALL(*mock_serial_, write("PING\n"))
      .WillOnce(Return(5));
  }

  comms_->setMockSerial(mock_serial_);
  
  EXPECT_NO_THROW(comms_->setup("", 115200, 50));
  EXPECT_TRUE(comms_->connected());
}

TEST_F(TB6612CommsConnectionTest, TestAutoDetectionFailsAllPorts)
{
  // Test that auto-detection throws when no ports are available
  EXPECT_CALL(*mock_serial_, setPort(_))
    .Times(4);  // Should try all 4 candidate ports
  EXPECT_CALL(*mock_serial_, setBaudrate(115200))
    .Times(4);
  EXPECT_CALL(*mock_serial_, setTimeout(_))
    .Times(4);
  EXPECT_CALL(*mock_serial_, open())
    .Times(4)
    .WillRepeatedly(Throw(std::runtime_error("Port not available")));

  comms_->setMockSerial(mock_serial_);
  
  EXPECT_THROW(comms_->setup("", 115200, 50), std::runtime_error);
  EXPECT_FALSE(comms_->connected());
}

TEST_F(TB6612CommsConnectionTest, TestSpecificPortConnection)
{
  // Test connection to a specific port
  EXPECT_CALL(*mock_serial_, setPort("/dev/ttyACM0"));
  EXPECT_CALL(*mock_serial_, setBaudrate(115200));
  EXPECT_CALL(*mock_serial_, setTimeout(_));
  EXPECT_CALL(*mock_serial_, open());
  EXPECT_CALL(*mock_serial_, isOpen())
    .WillOnce(Return(true))
    .WillOnce(Return(true));
  EXPECT_CALL(*mock_serial_, write("PING\n"))
    .WillOnce(Return(5));

  comms_->setMockSerial(mock_serial_);
  
  EXPECT_NO_THROW(comms_->setup("/dev/ttyACM0", 115200, 50));
  EXPECT_TRUE(comms_->connected());
}

TEST_F(TB6612CommsConnectionTest, TestInvalidParameters)
{
  // Test that invalid parameters are corrected to defaults
  EXPECT_CALL(*mock_serial_, setPort("/dev/ttyUSB0"));
  EXPECT_CALL(*mock_serial_, setBaudrate(115200));  // Should use default, not -1
  EXPECT_CALL(*mock_serial_, setTimeout(_));
  EXPECT_CALL(*mock_serial_, open());
  EXPECT_CALL(*mock_serial_, isOpen())
    .WillOnce(Return(true))
    .WillOnce(Return(true));
  EXPECT_CALL(*mock_serial_, write("PING\n"))
    .WillOnce(Return(5));

  comms_->setMockSerial(mock_serial_);
  
  // Pass invalid parameters - should be corrected internally
  EXPECT_NO_THROW(comms_->setup("/dev/ttyUSB0", -1, -1));
  EXPECT_TRUE(comms_->connected());
}

// Test motor command formatting for different drive configurations
class TB6612CommsMotorCommandTest : public TB6612CommsTest
{
protected:
  void SetUp() override
  {
    TB6612CommsTest::SetUp();
    
    // Set up a connected state for motor command tests
    EXPECT_CALL(*mock_serial_, setPort(_));
    EXPECT_CALL(*mock_serial_, setBaudrate(_));
    EXPECT_CALL(*mock_serial_, setTimeout(_));
    EXPECT_CALL(*mock_serial_, open());
    EXPECT_CALL(*mock_serial_, isOpen())
      .WillRepeatedly(Return(true));
    EXPECT_CALL(*mock_serial_, write("PING\n"))
      .WillOnce(Return(5));

    comms_->setMockSerial(mock_serial_);
    comms_->setup("/dev/ttyUSB0", 115200, 50);
  }
};

TEST_F(TB6612CommsMotorCommandTest, TestDifferentialMotorCommandFormatting)
{
  // Test differential motor command formatting: "V {left} {right}\n"
  EXPECT_CALL(*mock_serial_, write("V 50 -30\n"))
    .WillOnce(Return(9));

  EXPECT_NO_THROW(comms_->setDifferentialMotors(50, -30));
}

TEST_F(TB6612CommsMotorCommandTest, TestDifferentialMotorValueClamping)
{
  // Test that values outside [-100, 100] are clamped
  EXPECT_CALL(*mock_serial_, write("V 100 -100\n"))
    .WillOnce(Return(11));

  EXPECT_NO_THROW(comms_->setDifferentialMotors(150, -150));
}

TEST_F(TB6612CommsMotorCommandTest, TestFourMotorCommandFormatting)
{
  // Test four motor command formatting: "M {fl} {fr} {rl} {rr}\n"
  EXPECT_CALL(*mock_serial_, write("M 25 -50 75 -25\n"))
    .WillOnce(Return(16));

  EXPECT_NO_THROW(comms_->setFourMotors(25, -50, 75, -25));
}

TEST_F(TB6612CommsMotorCommandTest, TestFourMotorValueClamping)
{
  // Test that all four motor values are clamped properly
  EXPECT_CALL(*mock_serial_, write("M 100 -100 100 -100\n"))
    .WillOnce(Return(21));

  EXPECT_NO_THROW(comms_->setFourMotors(200, -200, 150, -150));
}

TEST_F(TB6612CommsMotorCommandTest, TestMotorCommandBoundaryValues)
{
  // Test boundary values (exactly -100 and 100)
  EXPECT_CALL(*mock_serial_, write("V -100 100\n"))
    .WillOnce(Return(11));

  EXPECT_NO_THROW(comms_->setDifferentialMotors(-100, 100));
}

TEST_F(TB6612CommsMotorCommandTest, TestMotorCommandZeroValues)
{
  // Test zero values (stop command)
  EXPECT_CALL(*mock_serial_, write("V 0 0\n"))
    .WillOnce(Return(5));

  EXPECT_NO_THROW(comms_->setDifferentialMotors(0, 0));
}

// Test encoder reading functionality with mock serial responses
class TB6612CommsEncoderTest : public TB6612CommsTest
{
protected:
  void SetUp() override
  {
    TB6612CommsTest::SetUp();
    
    // Set up a connected state for encoder tests
    EXPECT_CALL(*mock_serial_, setPort(_));
    EXPECT_CALL(*mock_serial_, setBaudrate(_));
    EXPECT_CALL(*mock_serial_, setTimeout(_));
    EXPECT_CALL(*mock_serial_, open());
    EXPECT_CALL(*mock_serial_, isOpen())
      .WillRepeatedly(Return(true));
    EXPECT_CALL(*mock_serial_, write("PING\n"))
      .WillOnce(Return(5));

    comms_->setMockSerial(mock_serial_);
    comms_->setup("/dev/ttyUSB0", 115200, 50);
  }
};

TEST_F(TB6612CommsEncoderTest, TestDifferentialEncoderReading)
{
  // Test successful differential encoder reading
  EXPECT_CALL(*mock_serial_, write("E\n"))
    .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
    .WillOnce(Return("1234 -5678\n"));

  int left_enc, right_enc;
  EXPECT_NO_THROW(comms_->readDifferentialEncoders(left_enc, right_enc));
  
  EXPECT_EQ(left_enc, 1234);
  EXPECT_EQ(right_enc, -5678);
}

TEST_F(TB6612CommsEncoderTest, TestFourEncoderReading)
{
  // Test successful four encoder reading
  EXPECT_CALL(*mock_serial_, write("E\n"))
    .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
    .WillOnce(Return("100 -200 300 -400\n"));

  int fl, fr, rl, rr;
  EXPECT_NO_THROW(comms_->readFourEncoders(fl, fr, rl, rr));
  
  EXPECT_EQ(fl, 100);
  EXPECT_EQ(fr, -200);
  EXPECT_EQ(rl, 300);
  EXPECT_EQ(rr, -400);
}

TEST_F(TB6612CommsEncoderTest, TestEncoderReadingInvalidFormat)
{
  // Test handling of invalid encoder response format
  EXPECT_CALL(*mock_serial_, write("E\n"))
    .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
    .WillOnce(Return("invalid response\n"));

  int left_enc, right_enc;
  EXPECT_NO_THROW(comms_->readDifferentialEncoders(left_enc, right_enc));
  
  // Should default to 0 on parse error
  EXPECT_EQ(left_enc, 0);
  EXPECT_EQ(right_enc, 0);
}

TEST_F(TB6612CommsEncoderTest, TestEncoderReadingPartialResponse)
{
  // Test handling of partial encoder response
  EXPECT_CALL(*mock_serial_, write("E\n"))
    .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
    .WillOnce(Return("1234\n"));  // Missing second value

  int left_enc, right_enc;
  EXPECT_NO_THROW(comms_->readDifferentialEncoders(left_enc, right_enc));
  
  // Should default to 0 on incomplete response
  EXPECT_EQ(left_enc, 0);
  EXPECT_EQ(right_enc, 0);
}

TEST_F(TB6612CommsEncoderTest, TestEncoderReadingEmptyResponse)
{
  // Test handling of empty encoder response
  EXPECT_CALL(*mock_serial_, write("E\n"))
    .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
    .WillOnce(Return(""));

  int left_enc, right_enc;
  EXPECT_NO_THROW(comms_->readDifferentialEncoders(left_enc, right_enc));
  
  // Should default to 0 on empty response
  EXPECT_EQ(left_enc, 0);
  EXPECT_EQ(right_enc, 0);
}

// Test error handling scenarios and connection failures
class TB6612CommsErrorHandlingTest : public TB6612CommsTest
{
protected:
  void SetUp() override
  {
    TB6612CommsTest::SetUp();
    
    // Set up a connected state for error handling tests
    EXPECT_CALL(*mock_serial_, setPort(_));
    EXPECT_CALL(*mock_serial_, setBaudrate(_));
    EXPECT_CALL(*mock_serial_, setTimeout(_));
    EXPECT_CALL(*mock_serial_, open());
    EXPECT_CALL(*mock_serial_, isOpen())
      .WillRepeatedly(Return(true));
    EXPECT_CALL(*mock_serial_, write("PING\n"))
      .WillOnce(Return(5));

    comms_->setMockSerial(mock_serial_);
    comms_->setup("/dev/ttyUSB0", 115200, 50);
  }
};

TEST_F(TB6612CommsErrorHandlingTest, TestMotorCommandWriteFailure)
{
  // Test handling of serial write failure during motor command
  EXPECT_CALL(*mock_serial_, write("V 50 -30\n"))
    .WillOnce(Throw(std::runtime_error("Write failed")));

  EXPECT_THROW(comms_->setDifferentialMotors(50, -30), std::runtime_error);
  
  // Verify error counter is incremented
  int write_errors, read_errors, reconnection_attempts;
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_GT(write_errors, 0);
}

TEST_F(TB6612CommsErrorHandlingTest, TestEncoderReadFailure)
{
  // Test handling of serial read failure during encoder reading
  EXPECT_CALL(*mock_serial_, write("E\n"))
    .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
    .WillOnce(Throw(std::runtime_error("Read failed")));

  int left_enc, right_enc;
  EXPECT_NO_THROW(comms_->readDifferentialEncoders(left_enc, right_enc));
  
  // Should default to 0 on read error
  EXPECT_EQ(left_enc, 0);
  EXPECT_EQ(right_enc, 0);
  
  // Verify error counter is incremented
  int write_errors, read_errors, reconnection_attempts;
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_GT(read_errors, 0);
}

TEST_F(TB6612CommsErrorHandlingTest, TestConnectionLoss)
{
  // Test handling of connection loss
  EXPECT_CALL(*mock_serial_, isOpen())
    .WillOnce(Return(false));  // Simulate connection loss

  EXPECT_FALSE(comms_->connected());
}

TEST_F(TB6612CommsErrorHandlingTest, TestReconnectionAttempt)
{
  // Test reconnection functionality
  {
    InSequence seq;
    
    // First check shows disconnected
    EXPECT_CALL(*mock_serial_, isOpen())
      .WillOnce(Return(false));
    
    // Reconnection attempt
    EXPECT_CALL(*mock_serial_, close());
    EXPECT_CALL(*mock_serial_, setPort(_));
    EXPECT_CALL(*mock_serial_, setBaudrate(_));
    EXPECT_CALL(*mock_serial_, setTimeout(_));
    EXPECT_CALL(*mock_serial_, open());
    EXPECT_CALL(*mock_serial_, isOpen())
      .WillOnce(Return(true))
      .WillOnce(Return(true));
    EXPECT_CALL(*mock_serial_, write("PING\n"))
      .WillOnce(Return(5));
  }

  EXPECT_FALSE(comms_->connected());
  EXPECT_TRUE(comms_->attemptReconnection());
  
  // Verify reconnection counter is incremented
  int write_errors, read_errors, reconnection_attempts;
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_GT(reconnection_attempts, 0);
}

TEST_F(TB6612CommsErrorHandlingTest, TestReconnectionFailure)
{
  // Test failed reconnection attempt
  {
    InSequence seq;
    
    // First check shows disconnected
    EXPECT_CALL(*mock_serial_, isOpen())
      .WillOnce(Return(false));
    
    // Reconnection attempt fails
    EXPECT_CALL(*mock_serial_, close());
    EXPECT_CALL(*mock_serial_, setPort(_));
    EXPECT_CALL(*mock_serial_, setBaudrate(_));
    EXPECT_CALL(*mock_serial_, setTimeout(_));
    EXPECT_CALL(*mock_serial_, open())
      .WillOnce(Throw(std::runtime_error("Reconnection failed")));
  }

  EXPECT_FALSE(comms_->connected());
  EXPECT_FALSE(comms_->attemptReconnection());
}

TEST_F(TB6612CommsErrorHandlingTest, TestErrorCounterReset)
{
  // Generate some errors first
  EXPECT_CALL(*mock_serial_, write("V 50 -30\n"))
    .WillOnce(Throw(std::runtime_error("Write failed")));

  EXPECT_THROW(comms_->setDifferentialMotors(50, -30), std::runtime_error);
  
  // Verify error counter is set
  int write_errors, read_errors, reconnection_attempts;
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_GT(write_errors, 0);
  
  // Reset counters
  comms_->resetErrorCounters();
  
  // Verify counters are reset
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_EQ(write_errors, 0);
  EXPECT_EQ(read_errors, 0);
  EXPECT_EQ(reconnection_attempts, 0);
}

TEST_F(TB6612CommsErrorHandlingTest, TestCommandsOnDisconnectedState)
{
  // Test that commands fail gracefully when not connected
  EXPECT_CALL(*mock_serial_, isOpen())
    .WillRepeatedly(Return(false));

  EXPECT_THROW(comms_->setDifferentialMotors(50, -30), std::runtime_error);
  EXPECT_THROW(comms_->setFourMotors(10, 20, 30, 40), std::runtime_error);
  
  int left_enc, right_enc;
  EXPECT_THROW(comms_->readDifferentialEncoders(left_enc, right_enc), std::runtime_error);
  
  int fl, fr, rl, rr;
  EXPECT_THROW(comms_->readFourEncoders(fl, fr, rl, rr), std::runtime_error);
}

// Test ping functionality
class TB6612CommsPingTest : public TB6612CommsTest
{
protected:
  void SetUp() override
  {
    TB6612CommsTest::SetUp();
    
    // Set up a connected state for ping tests
    EXPECT_CALL(*mock_serial_, setPort(_));
    EXPECT_CALL(*mock_serial_, setBaudrate(_));
    EXPECT_CALL(*mock_serial_, setTimeout(_));
    EXPECT_CALL(*mock_serial_, open());
    EXPECT_CALL(*mock_serial_, isOpen())
      .WillRepeatedly(Return(true));
    EXPECT_CALL(*mock_serial_, write("PING\n"))
      .WillOnce(Return(5));

    comms_->setMockSerial(mock_serial_);
    comms_->setup("/dev/ttyUSB0", 115200, 50);
  }
};

TEST_F(TB6612CommsPingTest, TestPingCommand)
{
  // Test that ping sends correct command
  EXPECT_CALL(*mock_serial_, write("PING\n"))
    .WillOnce(Return(5));

  EXPECT_NO_THROW(comms_->sendPing());
}

TEST_F(TB6612CommsPingTest, TestPingFailure)
{
  // Test that ping failure is handled gracefully
  EXPECT_CALL(*mock_serial_, write("PING\n"))
    .WillOnce(Throw(std::runtime_error("Ping failed")));

  // Ping failure should not throw, just log warning
  EXPECT_NO_THROW(comms_->sendPing());
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  
  int result = RUN_ALL_TESTS();
  
  rclcpp::shutdown();
  return result;
}