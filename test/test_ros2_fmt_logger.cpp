// Copyright (C) 2025 Nobleo Autonomous Solutions B.V.

#include <gtest/gtest.h>
#include <rcutils/logging.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <thread>

#include "ros2_fmt_logger/ros2_fmt_logger.hpp"

using std::chrono_literals::operator""ms;

// Custom logging handler to capture log output
static std::vector<std::pair<int, std::string>> captured_logs;

static void test_log_handler(
  const rcutils_log_location_t * /*location*/, int severity, const char * /*name*/,
  rcutils_time_point_value_t /*timestamp*/, const char * format, va_list * args)
{
  char buffer[1024];
  vsnprintf(buffer, sizeof(buffer), format, *args);
  captured_logs.emplace_back(severity, std::string(buffer));
}

class Ros2FmtLoggerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_node");
    rcl_logger_ = node_->get_logger();
    fmt_logger_ = std::make_unique<ros2_fmt_logger::Logger>(rcl_logger_);

    // Set up custom log handler
    original_handler_ = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(test_log_handler);
    captured_logs.clear();
  }

  void TearDown() override
  {
    // Restore original handler
    rcutils_logging_set_output_handler(original_handler_);
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger rcl_logger_{rclcpp::get_logger("default")};
  std::unique_ptr<ros2_fmt_logger::Logger> fmt_logger_;
  rcutils_logging_output_handler_t original_handler_;
};

TEST_F(Ros2FmtLoggerTest, TestFatalLoggingEquivalence)
{
  // Test with fmt logger
  fmt_logger_->fatal("Value: {}", 5);
  ASSERT_EQ(captured_logs.size(), 1u);
  auto fmt_severity = captured_logs[0].first;
  auto fmt_message = captured_logs[0].second;

  // Clear for next test
  captured_logs.clear();

  // Test with RCLCPP_FATAL
  RCLCPP_FATAL(rcl_logger_, "Value: %d", 5);
  ASSERT_EQ(captured_logs.size(), 1u);
  auto rclcpp_severity = captured_logs[0].first;
  auto rclcpp_message = captured_logs[0].second;

  // Both should use FATAL severity
  EXPECT_EQ(fmt_severity, RCUTILS_LOG_SEVERITY_FATAL);
  EXPECT_EQ(rclcpp_severity, RCUTILS_LOG_SEVERITY_FATAL);
  EXPECT_EQ(fmt_severity, rclcpp_severity);

  // Both should produce the same message content
  EXPECT_EQ(fmt_message, rclcpp_message);
  EXPECT_EQ(fmt_message, "Value: 5");
}

TEST_F(Ros2FmtLoggerTest, TestFatalOnceLogging)
{
  // Test the fatal_once functionality
  fmt_logger_->fatal_once("Test message");
  fmt_logger_->fatal_once("Test message");
  fmt_logger_->fatal_once("Test message");

  // Should only log once, even when called multiple times
  EXPECT_EQ(captured_logs.size(), 1u);
  EXPECT_EQ(captured_logs[0].second, "Test message");
  EXPECT_EQ(captured_logs[0].first, RCUTILS_LOG_SEVERITY_FATAL);
}

TEST_F(Ros2FmtLoggerTest, TestFatalThrottleLogging)
{
  // Clear any previous logs
  captured_logs.clear();

  // Create a logger with a clock for throttling tests
  auto clock = rclcpp::Clock{RCL_STEADY_TIME};
  auto fmt_logger = ros2_fmt_logger::Logger{rcl_logger_, clock};

  auto throttle_duration = 10ms;

  // First call should log immediately
  fmt_logger.fatal_throttle(throttle_duration, "Throttled message: {}", 1);
  EXPECT_EQ(captured_logs.size(), 1u);
  if (captured_logs.size() >= 1) {
    EXPECT_EQ(captured_logs[0].second, "Throttled message: 1");
    EXPECT_EQ(captured_logs[0].first, RCUTILS_LOG_SEVERITY_FATAL);
  }

  // Immediate subsequent calls should be throttled (not logged)
  fmt_logger.fatal_throttle(throttle_duration, "Throttled message: {}", 2);
  fmt_logger.fatal_throttle(throttle_duration, "Throttled message: {}", 3);
  EXPECT_EQ(captured_logs.size(), 1u);  // Still only 1 log entry

  // Wait for throttle duration to pass
  std::this_thread::sleep_for(20ms);

  // Now it should log again
  fmt_logger.fatal_throttle(throttle_duration, "Throttled message: {}", 4);
  EXPECT_EQ(captured_logs.size(), 2u);
  if (captured_logs.size() >= 2) {
    EXPECT_EQ(captured_logs[1].second, "Throttled message: 4");
    EXPECT_EQ(captured_logs[1].first, RCUTILS_LOG_SEVERITY_FATAL);
  }

  // Immediate call should be throttled again
  fmt_logger.fatal_throttle(throttle_duration, "Throttled message: {}", 5);
  EXPECT_EQ(captured_logs.size(), 2u);  // Still only 2 log entries
}

TEST_F(Ros2FmtLoggerTest, TestFatalOnChangeLogging)
{
  // Clear any previous logs
  captured_logs.clear();

  // Test with integer values
  int sensor_value = 100;

  // First call should log (initial value)
  fmt_logger_->fatal_on_change(sensor_value, "Sensor value changed to: {}", sensor_value);
  EXPECT_EQ(captured_logs.size(), 1u);
  if (captured_logs.size() >= 1) {
    EXPECT_EQ(captured_logs[0].second, "Sensor value changed to: 100");
    EXPECT_EQ(captured_logs[0].first, RCUTILS_LOG_SEVERITY_FATAL);
  }

  // Same value should not log again
  fmt_logger_->fatal_on_change(sensor_value, "Sensor value changed to: {}", sensor_value);
  fmt_logger_->fatal_on_change(sensor_value, "Sensor value changed to: {}", sensor_value);
  EXPECT_EQ(captured_logs.size(), 1u);  // Still only 1 log entry

  // Different value should log
  sensor_value = 200;
  fmt_logger_->fatal_on_change(sensor_value, "Sensor value changed to: {}", sensor_value);
  EXPECT_EQ(captured_logs.size(), 2u);
  if (captured_logs.size() >= 2) {
    EXPECT_EQ(captured_logs[1].second, "Sensor value changed to: 200");
    EXPECT_EQ(captured_logs[1].first, RCUTILS_LOG_SEVERITY_FATAL);
  }

  // Same value again should not log
  fmt_logger_->fatal_on_change(sensor_value, "Sensor value changed to: {}", sensor_value);
  EXPECT_EQ(captured_logs.size(), 2u);
}
