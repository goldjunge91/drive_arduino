#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

// Include the actual TB6612Comms header for testing
#include "tb6612_hardware/tb6612_comms.h"

using namespace tb6612_hardware;

class TB6612CommsSimpleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS 2 for logging
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    // Clean up
  }
};

// Test constructor and basic initialization
TEST_F(TB6612CommsSimpleTest, TestDefaultConstructor)
{
  EXPECT_NO_THROW({
    TB6612Comms comms;
    // Default constructor should not throw
    EXPECT_FALSE(comms.connected());
  });
}

TEST_F(TB6612CommsSimpleTest, TestParameterizedConstructorWithInvalidPort)
{
  // Test that constructor with invalid port handles errors gracefully
  EXPECT_THROW({
    TB6612Comms comms("/dev/nonexistent_port", 115200, 50);
  }, std::exception);
}

TEST_F(TB6612CommsSimpleTest, TestSetupWithInvalidPort)
{
  TB6612Comms comms;
  
  // Test that setup with invalid port throws appropriate error
  EXPECT_THROW({
    comms.setup("/dev/nonexistent_port", 115200, 50);
  }, std::exception);
  
  EXPECT_FALSE(comms.connected());
}

TEST_F(TB6612CommsSimpleTest, TestSetupWithEmptyPortAutoDetection)
{
  TB6612Comms comms;
  
  // Test auto-detection when no port is specified
  // This should fail in test environment but not crash
  EXPECT_THROW({
    comms.setup("", 115200, 50);
  }, std::runtime_error);
  
  EXPECT_FALSE(comms.connected());
}

TEST_F(TB6612CommsSimpleTest, TestSetupWithInvalidParameters)
{
  TB6612Comms comms;
  
  // Test that invalid parameters are handled (should not crash)
  EXPECT_THROW({
    comms.setup("/dev/nonexistent_port", -1, -1);
  }, std::exception);
  
  EXPECT_FALSE(comms.connected());
}

// Test motor command methods when not connected
TEST_F(TB6612CommsSimpleTest, TestMotorCommandsWhenDisconnected)
{
  TB6612Comms comms;
  
  // Motor commands should throw when not connected
  EXPECT_THROW({
    comms.setDifferentialMotors(50, -30);
  }, std::runtime_error);
  
  EXPECT_THROW({
    comms.setFourMotors(25, -50, 75, -25);
  }, std::runtime_error);
}

// Test encoder reading methods when not connected
TEST_F(TB6612CommsSimpleTest, TestEncoderReadingWhenDisconnected)
{
  TB6612Comms comms;
  
  int left_enc, right_enc;
  // Encoder reading should throw when not connected
  EXPECT_THROW({
    comms.readDifferentialEncoders(left_enc, right_enc);
  }, std::runtime_error);
  
  int fl, fr, rl, rr;
  EXPECT_THROW({
    comms.readFourEncoders(fl, fr, rl, rr);
  }, std::runtime_error);
}

// Test ping functionality when not connected
TEST_F(TB6612CommsSimpleTest, TestPingWhenDisconnected)
{
  TB6612Comms comms;
  
  // Ping should not throw even when disconnected (it logs warning)
  EXPECT_NO_THROW({
    comms.sendPing();
  });
}

// Test connection status
TEST_F(TB6612CommsSimpleTest, TestConnectionStatus)
{
  TB6612Comms comms;
  
  // Should be disconnected initially
  EXPECT_FALSE(comms.connected());
}

// Test reconnection attempt when not connected
TEST_F(TB6612CommsSimpleTest, TestReconnectionWhenDisconnected)
{
  TB6612Comms comms;
  
  // Reconnection should fail when never connected
  EXPECT_FALSE(comms.attemptReconnection());
  EXPECT_FALSE(comms.connected());
}

// Test error statistics
TEST_F(TB6612CommsSimpleTest, TestErrorStatistics)
{
  TB6612Comms comms;
  
  int write_errors, read_errors, reconnection_attempts;
  comms.getConnectionStats(write_errors, read_errors, reconnection_attempts);
  
  // Initially should be zero
  EXPECT_EQ(write_errors, 0);
  EXPECT_EQ(read_errors, 0);
  EXPECT_EQ(reconnection_attempts, 0);
  
  // Test reset functionality
  EXPECT_NO_THROW({
    comms.resetErrorCounters();
  });
  
  // Should still be zero after reset
  comms.getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_EQ(write_errors, 0);
  EXPECT_EQ(read_errors, 0);
  EXPECT_EQ(reconnection_attempts, 0);
}

// Test motor command formatting (we can test the logic without actual serial connection)
class TB6612CommsFormatTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

// Test value clamping logic by examining expected behavior
TEST_F(TB6612CommsFormatTest, TestMotorValueClamping)
{
  TB6612Comms comms;
  
  // We can't test the actual serial output, but we can test that
  // the methods handle extreme values without crashing
  EXPECT_THROW({
    comms.setDifferentialMotors(200, -200);  // Should clamp to [-100, 100]
  }, std::runtime_error);  // Throws because not connected
  
  EXPECT_THROW({
    comms.setFourMotors(150, -150, 200, -200);  // Should clamp to [-100, 100]
  }, std::runtime_error);  // Throws because not connected
}

TEST_F(TB6612CommsFormatTest, TestBoundaryValues)
{
  TB6612Comms comms;
  
  // Test boundary values
  EXPECT_THROW({
    comms.setDifferentialMotors(-100, 100);  // Exact boundaries
  }, std::runtime_error);  // Throws because not connected
  
  EXPECT_THROW({
    comms.setDifferentialMotors(0, 0);  // Zero values
  }, std::runtime_error);  // Throws because not connected
}

// Integration test class for testing with mock behavior
class TB6612CommsIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(TB6612CommsIntegrationTest, TestCompleteWorkflow)
{
  TB6612Comms comms;
  
  // Test complete workflow from construction to usage
  EXPECT_FALSE(comms.connected());
  
  // Attempt setup (will fail in test environment)
  EXPECT_THROW({
    comms.setup("", 115200, 50);
  }, std::runtime_error);
  
  // Should still be disconnected
  EXPECT_FALSE(comms.connected());
  
  // Commands should fail
  EXPECT_THROW({
    comms.setDifferentialMotors(50, -30);
  }, std::runtime_error);
  
  // Reconnection should fail
  EXPECT_FALSE(comms.attemptReconnection());
  
  // Error stats should show reconnection attempt
  int write_errors, read_errors, reconnection_attempts;
  comms.getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_GT(reconnection_attempts, 0);
}

// Test serial port detection logic
TEST_F(TB6612CommsIntegrationTest, TestSerialPortDetection)
{
  TB6612Comms comms;
  
  // Test that auto-detection handles the case where no ports are available
  EXPECT_THROW({
    comms.setup("", 115200, 50);
  }, std::runtime_error);
  
  // Test with specific invalid port
  EXPECT_THROW({
    comms.setup("/dev/invalid_port", 115200, 50);
  }, std::exception);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  
  int result = RUN_ALL_TESTS();
  
  rclcpp::shutdown();
  return result;
}