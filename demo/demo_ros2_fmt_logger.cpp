// Copyright (C) 2025 Nobleo Autonomous Solutions B.V.

#include <chrono>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "ros2_fmt_logger/ros2_fmt_logger.hpp"

#define RCLCPP_CUSTOM(logger, ...)                                                     \
  do {                                                                                 \
    static_assert(                                                                     \
      ::std::is_convertible<                                                           \
        typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
        typename ::rclcpp::Logger>::value,                                             \
      "First argument to logging macros must be an rclcpp::Logger");                   \
                                                                                       \
    RCUTILS_LOG_FATAL_NAMED((logger).get_name(), __VA_ARGS__);                         \
  } while (0)

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
  std::cout << "\nUsing RCLCPP macros with the fmt_logger:" << std::endl;
  RCLCPP_CUSTOM(fmt_logger, "Value: %d", 5);

  std::cout << "\nComplex formatting:" << std::endl;
  fmt_logger.fatal("Item {} at ({}, {}) = {:.2f}", 42, 10, 20, 1.2345);
  RCLCPP_FATAL(rcl_logger, "Item %d at (%d, %d) = %.2f", 42, 10, 20, 1.2345);

  std::cout << "\nFatal once functionality (called 3 times, should only log once):" << std::endl;
  fmt_logger.fatal_once("This message appears only once");
  fmt_logger.fatal_once("This message appears only once");
  fmt_logger.fatal_once("This message appears only once");

  std::cout << "\nThrottle functionality (called 10 times with 500ms throttle):" << std::endl;
  auto throttle_duration = rclcpp::Duration::from_nanoseconds(500000000);  // 500ms
  for (int i = 0; i < 10; ++i) {
    fmt_logger.fatal_throttle(
      throttle_duration, "Throttled message #{} - only some will appear", i);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Sleep 200ms between calls
    std::cout << "Loop iteration " << i << " completed" << std::endl;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
