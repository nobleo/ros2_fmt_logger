// Copyright (C) 2025 Nobleo Autonomous Solutions B.V.

#pragma once

#include <fmt/format.h>
#include <rcutils/logging.h>

#include <rclcpp/clock.hpp>
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
  explicit Logger(const rclcpp::Logger & logger) : rclcpp::Logger{logger} {}

  Logger(const rclcpp::Logger & logger, const rclcpp::Clock & clock)
  : rclcpp::Logger(logger), clock_(clock)
  {
  }

  template <typename... Args>
  void debug(const format_string & format, Args &&... args) const
  {
    log(RCUTILS_LOG_SEVERITY_DEBUG, format, fmt::make_format_args(args...));
  }

  template <typename... Args, typename Unique = decltype([] {})>
  void debug_once(const format_string & format, Args &&... args) const
  {
    log_once<Unique>(RCUTILS_LOG_SEVERITY_DEBUG, format, fmt::make_format_args(args...));
  }

  template <typename... Args, typename Unique = decltype([] {})>
  void debug_throttle(
    const rclcpp::Duration & duration, const format_string & format, Args &&... args) const
  {
    log_throttle<Unique>(
      RCUTILS_LOG_SEVERITY_DEBUG, duration, format, fmt::make_format_args(args...));
  }

  template <typename T, typename... Args, typename Unique = decltype([] {})>
  void debug_on_change(const T value, const format_string & format, Args &&... args) const
  {
    log_on_change<T, Unique>(
      RCUTILS_LOG_SEVERITY_DEBUG, value, format, fmt::make_format_args(args...));
  }

  template <typename TV, typename TT, typename... Args, typename Unique = decltype([] {})>
  void debug_on_change(
    const TV & value, const TT & threshold, const format_string & format, Args &&... args) const
  {
    log_on_change<TV, TT, Unique>(
      RCUTILS_LOG_SEVERITY_DEBUG, value, threshold, format, fmt::make_format_args(args...));
  }

  template <typename... Args>
  void info(const format_string & format, Args &&... args) const
  {
    log(RCUTILS_LOG_SEVERITY_INFO, format, fmt::make_format_args(args...));
  }

  template <typename... Args, typename Unique = decltype([] {})>
  void info_once(const format_string & format, Args &&... args) const
  {
    log_once<Unique>(RCUTILS_LOG_SEVERITY_INFO, format, fmt::make_format_args(args...));
  }

  template <typename... Args, typename Unique = decltype([] {})>
  void info_throttle(
    const rclcpp::Duration & duration, const format_string & format, Args &&... args) const
  {
    log_throttle<Unique>(
      RCUTILS_LOG_SEVERITY_INFO, duration, format, fmt::make_format_args(args...));
  }

  template <typename T, typename... Args, typename Unique = decltype([] {})>
  void info_on_change(const T value, const format_string & format, Args &&... args) const
  {
    log_on_change<T, Unique>(
      RCUTILS_LOG_SEVERITY_INFO, value, format, fmt::make_format_args(args...));
  }

  template <typename TV, typename TT, typename... Args, typename Unique = decltype([] {})>
  void info_on_change(
    const TV & value, const TT & threshold, const format_string & format, Args &&... args) const
  {
    log_on_change<TV, TT, Unique>(
      RCUTILS_LOG_SEVERITY_INFO, value, threshold, format, fmt::make_format_args(args...));
  }

  template <typename... Args>
  void warn(const format_string & format, Args &&... args) const
  {
    log(RCUTILS_LOG_SEVERITY_WARN, format, fmt::make_format_args(args...));
  }

  template <typename... Args, typename Unique = decltype([] {})>
  void warn_once(const format_string & format, Args &&... args) const
  {
    log_once<Unique>(RCUTILS_LOG_SEVERITY_WARN, format, fmt::make_format_args(args...));
  }

  template <typename... Args, typename Unique = decltype([] {})>
  void warn_throttle(
    const rclcpp::Duration & duration, const format_string & format, Args &&... args) const
  {
    log_throttle<Unique>(
      RCUTILS_LOG_SEVERITY_WARN, duration, format, fmt::make_format_args(args...));
  }

  template <typename T, typename... Args, typename Unique = decltype([] {})>
  void warn_on_change(const T value, const format_string & format, Args &&... args) const
  {
    log_on_change<T, Unique>(
      RCUTILS_LOG_SEVERITY_WARN, value, format, fmt::make_format_args(args...));
  }

  template <typename TV, typename TT, typename... Args, typename Unique = decltype([] {})>
  void warn_on_change(
    const TV & value, const TT & threshold, const format_string & format, Args &&... args) const
  {
    log_on_change<TV, TT, Unique>(
      RCUTILS_LOG_SEVERITY_WARN, value, threshold, format, fmt::make_format_args(args...));
  }

  template <typename... Args>
  void error(const format_string & format, Args &&... args) const
  {
    log(RCUTILS_LOG_SEVERITY_ERROR, format, fmt::make_format_args(args...));
  }

  template <typename... Args, typename Unique = decltype([] {})>
  void error_once(const format_string & format, Args &&... args) const
  {
    log_once<Unique>(RCUTILS_LOG_SEVERITY_ERROR, format, fmt::make_format_args(args...));
  }

  template <typename... Args, typename Unique = decltype([] {})>
  void error_throttle(
    const rclcpp::Duration & duration, const format_string & format, Args &&... args) const
  {
    log_throttle<Unique>(
      RCUTILS_LOG_SEVERITY_ERROR, duration, format, fmt::make_format_args(args...));
  }

  template <typename T, typename... Args, typename Unique = decltype([] {})>
  void error_on_change(const T value, const format_string & format, Args &&... args) const
  {
    log_on_change<T, Unique>(
      RCUTILS_LOG_SEVERITY_ERROR, value, format, fmt::make_format_args(args...));
  }

  template <typename TV, typename TT, typename... Args, typename Unique = decltype([] {})>
  void error_on_change(
    const TV & value, const TT & threshold, const format_string & format, Args &&... args) const
  {
    log_on_change<TV, TT, Unique>(
      RCUTILS_LOG_SEVERITY_ERROR, value, threshold, format, fmt::make_format_args(args...));
  }

  template <typename... Args>
  void fatal(const format_string & format, Args &&... args) const
  {
    log(RCUTILS_LOG_SEVERITY_FATAL, format, fmt::make_format_args(args...));
  }

  template <typename... Args, typename Unique = decltype([] {})>
  void fatal_once(const format_string & format, Args &&... args) const
  {
    log_once<Unique>(RCUTILS_LOG_SEVERITY_FATAL, format, fmt::make_format_args(args...));
  }

  template <typename... Args, typename Unique = decltype([] {})>
  void fatal_throttle(
    const rclcpp::Duration & duration, const format_string & format, Args &&... args) const
  {
    log_throttle<Unique>(
      RCUTILS_LOG_SEVERITY_FATAL, duration, format, fmt::make_format_args(args...));
  }

  template <typename T, typename... Args, typename Unique = decltype([] {})>
  void fatal_on_change(const T value, const format_string & format, Args &&... args) const
  {
    log_on_change<T, Unique>(
      RCUTILS_LOG_SEVERITY_FATAL, value, format, fmt::make_format_args(args...));
  }

  template <typename TV, typename TT, typename... Args, typename Unique = decltype([] {})>
  void fatal_on_change(
    const TV & value, const TT & threshold, const format_string & format, Args &&... args) const
  {
    log_on_change<TV, TT, Unique>(
      RCUTILS_LOG_SEVERITY_FATAL, value, threshold, format, fmt::make_format_args(args...));
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

  template <typename Unique>
  void log_once(
    const RCUTILS_LOG_SEVERITY severity, const format_string & format,
    const fmt::format_args & args) const
  {
    static bool already_logged = false;
    if (!already_logged) {
      already_logged = true;
      log(severity, format, args);
    }
  }

  template <typename Unique>
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

  template <typename T, typename Unique>
  void log_on_change(
    const RCUTILS_LOG_SEVERITY severity, const T & value, const format_string & format,
    const fmt::format_args & args) const
  {
    static T last_value;
    if (value != last_value) {
      last_value = value;
      log(severity, format, args);
    }
  }

  template <typename TV, typename TT, typename Unique>
  void log_on_change(
    const RCUTILS_LOG_SEVERITY severity, const TV & value, const TT & threshold,
    const format_string & format, const fmt::format_args & args) const
  {
    static TV last_value;
    if ((value - last_value) >= threshold) {
      last_value = value;
      log(severity, format, args);
    }
  }
};

}  // namespace ros2_fmt_logger
