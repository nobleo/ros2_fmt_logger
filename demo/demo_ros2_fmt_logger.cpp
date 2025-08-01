// Copyright (C) 2025 Nobleo Autonomous Solutions B.V.

#include <chrono>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "ros2_fmt_logger/ros2_fmt_logger.hpp"

using std::chrono_literals::operator""ms;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("demo_node");
  auto rcl_logger = node->get_logger();
  auto fmt_logger = ros2_fmt_logger::Logger(rcl_logger);

  std::cout << "\n=== Demonstrating equivalent logging outputs ===\n" << std::endl;

  std::cout << "Integer formatting:" << std::endl;
  fmt_logger.fatal("Value: {}", 5);
  RCLCPP_FATAL(rcl_logger, "Value: %d", 5);
  // std::cout << "\nUsing RCLCPP macros with the fmt_logger:" << std::endl;
  // RCLCPP_FATAL(fmt_logger, "Value: %d", 5);

  std::cout << "\nComplex formatting:" << std::endl;
  fmt_logger.fatal("Item {} at ({}, {}) = {:.2f}", 42, 10, 20, 1.2345);
  RCLCPP_FATAL(rcl_logger, "Item %d at (%d, %d) = %.2f", 42, 10, 20, 1.2345);

  std::cout << "\nFatal once functionality (called 3 times, should only log once):" << std::endl;
  for (int i = 0; i < 3; ++i) {
    fmt_logger.fatal_once("This message appears only once: {}", i);
    fmt_logger.fatal_once("This one only once as well: {}", i);
  }

  std::cout << "\nThrottle functionality (called 10 times with 500ms throttle):" << std::endl;
  for (size_t i = 0; i < 10; ++i) {
    std::cout << "Loop iteration " << i << std::endl;
    fmt_logger.fatal_throttle(500ms, "Throttled message #{} - only some will appear", i);
    fmt_logger.fatal_throttle(500ms, "Logging twice: {}", i);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Sleep 200ms between calls
  }

  std::cout << "\nFatal on change functionality (logs only when value changes):" << std::endl;

  // Simulate sensor readings that change over time
  std::vector<int> sensor_readings = {100, 100, 100, 200, 200, 150, 150, 300};
  for (const auto reading : sensor_readings) {
    std::cout << "Sensor reading = " << reading << std::endl;
    fmt_logger.fatal_on_change(reading, "Sensor reading changed to: {}", reading);
    // fmt_logger.fatal_on_change(reading, 80, "Sensor reading changed significantly to: {}", reading);
  }

  std::cout << "\nFatal on change with different types:" << std::endl;

  // Test with floating point values
  std::cout << "\nFatal on change with floating point values:" << std::endl;
  std::vector<double> temperatures = {20.5, 20.5, 25.1, 25.1, 30.7, 20.5};
  for (const auto temperature : temperatures) {
    std::cout << "Temperature = " << temperature << "°C" << std::endl;
    fmt_logger.fatal_on_change(temperature, "Temperature changed to: {:.1f}°C", temperature);
    fmt_logger.fatal_on_change(temperature, "Also temp changed to: {:.1f}°C", temperature);
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
