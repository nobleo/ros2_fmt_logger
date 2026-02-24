// Copyright (C) 2026 Nobleo Autonomous Solutions B.V.

#pragma once

#include <fmt/chrono.h>

#include <chrono>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

/**
 * @brief fmt formatter for rclcpp::Duration.
 *
 * Delegates to the std::chrono::duration<double> formatter, so all fmt chrono
 * format specs are supported. Formatting will be in seconds.
 *
 * @example
 * @code
 * rclcpp::Duration d{800ms};
 * fmt::format("{}", d);         // "0.8s"
 * @endcode
 */
template <>
struct fmt::formatter<rclcpp::Duration> : fmt::formatter<std::chrono::duration<double>>
{
  auto format(const rclcpp::Duration & duration, fmt::format_context & ctx) const
  {
    return fmt::formatter<std::chrono::duration<double>>::format(
      duration.to_chrono<std::chrono::duration<double>>(), ctx);
  }
};

/**
 * @brief fmt formatter for rclcpp::Time.
 *
 * Converts the ROS time (nanoseconds since epoch) to a std::chrono::sys_time,
 * so all fmt chrono format specs are supported.
 *
 * @example
 * @code
 * rclcpp::Time t = node->get_clock()->now();
 * fmt::format("{}", t);                    // "2026-02-24 08:59:17"
 * fmt::format("{:%H:%M:%S}", t);           // "08:59:17"
 * @endcode
 */
template <>
struct fmt::formatter<rclcpp::Time>
: fmt::formatter<std::chrono::sys_time<std::chrono::nanoseconds>>
{
  auto format(const rclcpp::Time & time, fmt::format_context & ctx) const
  {
    const auto tp =
      std::chrono::sys_time<std::chrono::nanoseconds>{std::chrono::nanoseconds{time.nanoseconds()}};
    return fmt::formatter<std::chrono::sys_time<std::chrono::nanoseconds>>::format(tp, ctx);
  }
};
