// Copyright (C) 2026 Nobleo Autonomous Solutions B.V.

#pragma once

#include <fmt/chrono.h>

#include <chrono>
#include <rclcpp/duration.hpp>

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
