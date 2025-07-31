// Copyright (C) 2025 Nobleo Autonomous Solutions B.V.

#pragma once

#include <fmt/format.h>
#include <rcutils/logging.h>

#include <rclcpp/logger.hpp>
#include <source_location>
#include <string>
#include <string_view>

namespace ros2_fmt_logger
{

struct format_string
{
  std::string_view str;
  std::source_location loc;

  format_string(
    const char * str, const std::source_location & loc = std::source_location::current())
  : str{str}, loc{loc}
  {
  }
};

class Logger : public rclcpp::Logger
{
public:
  explicit Logger(const rclcpp::Logger & logger) : rclcpp::Logger(logger) {}

  Logger(const rclcpp::Logger & logger, const rclcpp::Clock & clock)
  : rclcpp::Logger(logger), clock_(clock)
  {
  }

  template <typename... Args>
  void fatal(const format_string & format, Args &&... args) const
  {
    log(RCUTILS_LOG_SEVERITY_FATAL, format, fmt::make_format_args(args...));
  }

  template <typename... Args>
  void fatal_once(const format_string & format, Args &&... args) const
  {
    log_once(RCUTILS_LOG_SEVERITY_FATAL, format, fmt::make_format_args(args...));
  }

  template <typename... Args>
  void fatal_throttle(
    const rclcpp::Duration & duration, const format_string & format, Args &&... args) const
  {
    log_throttle(RCUTILS_LOG_SEVERITY_FATAL, duration, format, fmt::make_format_args(args...));
  }

  template <typename T, typename... Args>
  void fatal_on_change(const T value, const format_string & format, Args &&... args) const
  {
    static T last_value;
    if (value != last_value) {
      last_value = value;
      log(RCUTILS_LOG_SEVERITY_FATAL, format, fmt::make_format_args(args...));
    }
  }

private:
  rclcpp::Clock clock_{
    RCL_STEADY_TIME};  // Default to steady time, can be overridden in constructor

  void log(
    const RCUTILS_LOG_SEVERITY severity, const format_string & format,
    const fmt::format_args & args) const
  {
    RCUTILS_LOGGING_AUTOINIT;
    if (rcutils_logging_logger_is_enabled_for(get_name(), severity)) {
      rcutils_log_location_t rcutils_location{
        .function_name = format.loc.function_name(),
        .file_name = format.loc.file_name(),
        .line_number = static_cast<size_t>(format.loc.line())};
      rcutils_log(
        &rcutils_location, severity, get_name(), "%s", fmt::vformat(format.str, args).c_str());
    }
  }

  void log_once(
    const RCUTILS_LOG_SEVERITY severity, const format_string & format,
    const fmt::format_args & args) const
  {
    static bool logged = false;
    if (!logged) {
      logged = true;
      log(severity, format, args);
    }
  }

  void log_throttle(
    const RCUTILS_LOG_SEVERITY severity, const rclcpp::Duration & duration,
    const format_string & format, const fmt::format_args & args) const
  {
    static rclcpp::Time last_logged(static_cast<int64_t>(0), clock_.get_clock_type());

    try {
      auto now = clock_.now();
      if ((now - last_logged) >= duration) {
        last_logged = now;
        log(severity, format, args);
      }
    } catch (...) {
      // now() can throw, just ignore
    }
  }
};

}  // namespace ros2_fmt_logger
