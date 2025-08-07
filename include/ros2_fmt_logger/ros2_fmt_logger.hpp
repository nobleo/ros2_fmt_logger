// Copyright (C) 2025 Nobleo Autonomous Solutions B.V.

#pragma once

#include <fmt/format.h>
#include <rcutils/logging.h>

#include <cmath>
#include <optional>
#include <rclcpp/clock.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logger.hpp>
#include <source_location>
#include <string>
#include <string_view>

namespace ros2_fmt_logger
{

/**
 * @struct format_and_loc
 * @brief A wrapper for format strings that automatically captures source location
 *
 * This struct combines a format string with source location information for debugging.
 * It automatically captures the file, line, and function where the logging call was made.
 * This is required to have a single first argument as the number of arguments for all functions
 * is dynamic.
 */
struct format_and_loc
{
  std::string_view str;
  std::source_location loc;

  explicit(false) format_and_loc(
    const char * str, const std::source_location & loc = std::source_location::current())
  : str{str}, loc{loc}
  {
  }
};

/**
 * @class Logger
 * @brief Modern fmt-style logger that extends rclcpp::Logger
 *
 * @example
 * @code
 * auto logger = ros2_fmt_logger::Logger(node->get_logger());
 * logger.info("Processing item {} with value {:.2f}", id, value);
 * logger.warn_throttle(1s, "High CPU usage: {:.1%}", cpu_percent);
 * logger.error_once("Configuration error detected");
 * @endcode
 */
class Logger : public rclcpp::Logger
{
public:
  /**
   * @brief Construct a Logger from an existing rclcpp::Logger
   * @param logger The rclcpp::Logger to extend
   */
  explicit Logger(const rclcpp::Logger & logger) : rclcpp::Logger{logger} {}

  /**
   * @brief Construct a Logger with a specific clock for throttling features
   * @param logger The rclcpp::Logger to extend
   * @param clock Clock used for throttling functionality
   */
  Logger(const rclcpp::Logger & logger, const rclcpp::Clock & clock)
  : rclcpp::Logger(logger), clock_(clock)
  {
  }

  /**
   * @brief Log a debug message with fmt-style formatting
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.debug("Processing item {} of {}", current, total);
   * logger.debug("Sensor reading: {:.3f}", value);
   * @endcode
   */
  template <typename... Args>
  void debug(const format_and_loc & format, Args &&... args) const
  {
    log(RCUTILS_LOG_SEVERITY_DEBUG, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log a debug message only once, regardless of how many times called
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * Useful for logging in loops or frequently called functions where you only
   * want to see the message once.
   *
   * @example
   * @code
   * for (const auto& item : items) {
   *   logger.debug_once("Processing items in loop");
   *   process_item(item);
   * }
   * @endcode
   */
  template <typename... Args, typename Unique = decltype([]() { /*single instance per call*/ })>
  void debug_once(const format_and_loc & format, Args &&... args) const
  {
    log_once<Unique>(RCUTILS_LOG_SEVERITY_DEBUG, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log a debug message with throttling (rate limiting)
   * @param duration Minimum time between log messages
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * Prevents log spam by ensuring the message is only logged at most once
   * per specified duration.
   *
   * @example
   * @code
   * logger.debug_throttle(100ms, "High frequency data: {}", sensor_value);
   * logger.debug_throttle(1s, "Loop iteration {}", i);
   * @endcode
   */
  template <typename... Args, typename Unique = decltype([]() { /*single instance per call*/ })>
  void debug_throttle(
    const rclcpp::Duration & duration, const format_and_loc & format, Args &&... args) const
  {
    log_throttle<Unique>(
      RCUTILS_LOG_SEVERITY_DEBUG, duration, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log a debug message only when the monitored value changes
   * @param value The value to monitor for changes
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * Logs the message only when the provided value is different from the
   * last time this function was called.
   *
   * @example
   * @code
   * logger.debug_on_change(state, "State changed to: {}", state);
   * logger.debug_on_change(sensor_reading, "Sensor: {:.2f}", sensor_reading);
   * @endcode
   */
  template <typename T, typename... Args, typename Unique = decltype([]() { /*single instance*/ })>
  void debug_on_change(const T value, const format_and_loc & format, Args &&... args) const
  {
    log_on_change<T, Unique>(
      RCUTILS_LOG_SEVERITY_DEBUG, value, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log a debug message only when the monitored value changes by a threshold
   * @param value The value to monitor for changes
   * @param threshold Minimum change required to trigger logging
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * Logs the message only when the provided value differs from the last
   * logged value by at least the specified threshold.
   *
   * @example
   * @code
   * logger.debug_on_change(temperature, 1.0, "Temperature: {:.1f}°C", temperature);
   * logger.debug_on_change(position, 0.1, "Position: {:.2f}m", position);
   * @endcode
   */
  template <
    typename TV, typename TT, typename... Args, typename Unique = decltype([]() { /*single*/ })>
  void debug_on_change(
    const TV & value, const TT & threshold, const format_and_loc & format, Args &&... args) const
  {
    log_on_change<TV, TT, Unique>(
      RCUTILS_LOG_SEVERITY_DEBUG, value, threshold, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log an informational message with fmt-style formatting
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * Use for general informational messages about normal operation.
   *
   * @example
   * @code
   * logger.info("Node started successfully");
   * logger.info("Connected to {} on port {}", host, port);
   * logger.info("Processing {} items", items.size());
   * @endcode
   */
  template <typename... Args>
  void info(const format_and_loc & format, Args &&... args) const
  {
    log(RCUTILS_LOG_SEVERITY_INFO, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log an informational message only once
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.info_once("System initialization complete");
   * @endcode
   */
  template <typename... Args, typename Unique = decltype([]() { /*single instance per call*/ })>
  void info_once(const format_and_loc & format, Args &&... args) const
  {
    log_once<Unique>(RCUTILS_LOG_SEVERITY_INFO, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log an informational message with throttling
   * @param duration Minimum time between log messages
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.info_throttle(5s, "System status: {} items processed", count);
   * @endcode
   */
  template <typename... Args, typename Unique = decltype([]() { /*single instance per call*/ })>
  void info_throttle(
    const rclcpp::Duration & duration, const format_and_loc & format, Args &&... args) const
  {
    log_throttle<Unique>(
      RCUTILS_LOG_SEVERITY_INFO, duration, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log an informational message when a value changes
   * @param value The value to monitor for changes
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.info_on_change(current_state, "State transition to: {}", current_state);
   * @endcode
   */
  template <typename T, typename... Args, typename Unique = decltype([]() { /*single instance*/ })>
  void info_on_change(const T value, const format_and_loc & format, Args &&... args) const
  {
    log_on_change<T, Unique>(
      RCUTILS_LOG_SEVERITY_INFO, value, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log an informational message when a value changes by a threshold
   * @param value The value to monitor for changes
   * @param threshold Minimum change required to trigger logging
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.info_on_change(progress, 0.1, "Progress: {:.1%}", progress);
   * @endcode
   */
  template <
    typename TV, typename TT, typename... Args, typename Unique = decltype([]() { /*single*/ })>
  void info_on_change(
    const TV & value, const TT & threshold, const format_and_loc & format, Args &&... args) const
  {
    log_on_change<TV, TT, Unique>(
      RCUTILS_LOG_SEVERITY_INFO, value, threshold, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log a warning message with fmt-style formatting
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * Use for conditions that are potentially problematic but not necessarily errors.
   *
   * @example
   * @code
   * logger.warn("High CPU usage detected: {:.1%}", cpu_usage);
   * logger.warn("Deprecated parameter '{}' used", param_name);
   * logger.warn("Connection unstable, {} retries remaining", retries);
   * @endcode
   */
  template <typename... Args>
  void warn(const format_and_loc & format, Args &&... args) const
  {
    log(RCUTILS_LOG_SEVERITY_WARN, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log a warning message only once
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.warn_once("Deprecated API usage detected");
   * @endcode
   */
  template <typename... Args, typename Unique = decltype([]() { /*single instance per call*/ })>
  void warn_once(const format_and_loc & format, Args &&... args) const
  {
    log_once<Unique>(RCUTILS_LOG_SEVERITY_WARN, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log a warning message with throttling
   * @param duration Minimum time between log messages
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.warn_throttle(1s, "High memory usage: {:.1f} MB", memory_mb);
   * @endcode
   */
  template <typename... Args, typename Unique = decltype([]() { /*single instance per call*/ })>
  void warn_throttle(
    const rclcpp::Duration & duration, const format_and_loc & format, Args &&... args) const
  {
    log_throttle<Unique>(
      RCUTILS_LOG_SEVERITY_WARN, duration, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log a warning message when a value changes
   * @param value The value to monitor for changes
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.warn_on_change(error_count, "Error count changed: {}", error_count);
   * @endcode
   */
  template <typename T, typename... Args, typename Unique = decltype([]() { /*single instance*/ })>
  void warn_on_change(const T value, const format_and_loc & format, Args &&... args) const
  {
    log_on_change<T, Unique>(
      RCUTILS_LOG_SEVERITY_WARN, value, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log a warning message when a value changes by a threshold
   * @param value The value to monitor for changes
   * @param threshold Minimum change required to trigger logging
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.warn_on_change(latency_ms, 50.0, "High latency: {:.1f}ms", latency_ms);
   * @endcode
   */
  template <
    typename TV, typename TT, typename... Args, typename Unique = decltype([]() { /*single*/ })>
  void warn_on_change(
    const TV & value, const TT & threshold, const format_and_loc & format, Args &&... args) const
  {
    log_on_change<TV, TT, Unique>(
      RCUTILS_LOG_SEVERITY_WARN, value, threshold, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log an error message with fmt-style formatting
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * Use for error conditions that indicate something has gone wrong
   * but the system can potentially recover.
   *
   * @example
   * @code
   * logger.error("Failed to connect to server: {}", error_msg);
   * logger.error("Invalid parameter value: {} (expected > 0)", value);
   * logger.error("File not found: {}", filename);
   * @endcode
   */
  template <typename... Args>
  void error(const format_and_loc & format, Args &&... args) const
  {
    log(RCUTILS_LOG_SEVERITY_ERROR, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log an error message only once
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.error_once("Configuration file missing");
   * @endcode
   */
  template <typename... Args, typename Unique = decltype([]() { /*single instance per call*/ })>
  void error_once(const format_and_loc & format, Args &&... args) const
  {
    log_once<Unique>(RCUTILS_LOG_SEVERITY_ERROR, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log an error message with throttling
   * @param duration Minimum time between log messages
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.error_throttle(1s, "Connection lost to {}", device_name);
   * @endcode
   */
  template <typename... Args, typename Unique = decltype([]() { /*single instance per call*/ })>
  void error_throttle(
    const rclcpp::Duration & duration, const format_and_loc & format, Args &&... args) const
  {
    log_throttle<Unique>(
      RCUTILS_LOG_SEVERITY_ERROR, duration, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log an error message when a value changes
   * @param value The value to monitor for changes
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.error_on_change(error_state, "System error state: {}", error_state);
   * @endcode
   */
  template <typename T, typename... Args, typename Unique = decltype([]() { /*single instance*/ })>
  void error_on_change(const T value, const format_and_loc & format, Args &&... args) const
  {
    log_on_change<T, Unique>(
      RCUTILS_LOG_SEVERITY_ERROR, value, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log an error message when a value changes by a threshold
   * @param value The value to monitor for changes
   * @param threshold Minimum change required to trigger logging
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.error_on_change(error_rate, 0.05, "Error rate spike: {:.2%}", error_rate);
   * @endcode
   */
  template <
    typename TV, typename TT, typename... Args, typename Unique = decltype([]() { /*single*/ })>
  void error_on_change(
    const TV & value, const TT & threshold, const format_and_loc & format, Args &&... args) const
  {
    log_on_change<TV, TT, Unique>(
      RCUTILS_LOG_SEVERITY_ERROR, value, threshold, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log a fatal error message with fmt-style formatting
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * Use for critical errors that indicate the system cannot continue
   * normal operation. These typically precede system shutdown or restart.
   *
   * @example
   * @code
   * logger.fatal("Critical system failure: {}", error_details);
   * logger.fatal("Unable to initialize hardware: {}", device_name);
   * logger.fatal("Memory allocation failed for {} bytes", size);
   * @endcode
   */
  template <typename... Args>
  void fatal(const format_and_loc & format, Args &&... args) const
  {
    log(RCUTILS_LOG_SEVERITY_FATAL, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log a fatal error message only once
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.fatal_once("Critical configuration error detected");
   * @endcode
   */
  template <typename... Args, typename Unique = decltype([]() { /*single instance per call*/ })>
  void fatal_once(const format_and_loc & format, Args &&... args) const
  {
    log_once<Unique>(RCUTILS_LOG_SEVERITY_FATAL, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log a fatal error message with throttling
   * @param duration Minimum time between log messages
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.fatal_throttle(1s, "Critical system overload detected");
   * @endcode
   */
  template <typename... Args, typename Unique = decltype([]() { /*single instance per call*/ })>
  void fatal_throttle(
    const rclcpp::Duration & duration, const format_and_loc & format, Args &&... args) const
  {
    log_throttle<Unique>(
      RCUTILS_LOG_SEVERITY_FATAL, duration, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log a fatal error message when a value changes
   * @param value The value to monitor for changes
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.fatal_on_change(critical_system_state, "Critical state change: {}", critical_system_state);
   * @endcode
   */
  template <typename T, typename... Args, typename Unique = decltype([]() { /*single instance*/ })>
  void fatal_on_change(const T value, const format_and_loc & format, Args &&... args) const
  {
    log_on_change<T, Unique>(
      RCUTILS_LOG_SEVERITY_FATAL, value, format, fmt::make_format_args(args...));
  }

  /**
   * @brief Log a fatal error message when a value changes by a threshold
   * @param value The value to monitor for changes
   * @param threshold Minimum change required to trigger logging
   * @param format Format string with placeholders
   * @param args Arguments to format into the string
   *
   * @example
   * @code
   * logger.fatal_on_change(system_load, 0.95, "System overload: {:.1%}", system_load);
   * @endcode
   */
  template <
    typename TV, typename TT, typename... Args, typename Unique = decltype([]() { /*single*/ })>
  void fatal_on_change(
    const TV & value, const TT & threshold, const format_and_loc & format, Args &&... args) const
  {
    log_on_change<TV, TT, Unique>(
      RCUTILS_LOG_SEVERITY_FATAL, value, threshold, format, fmt::make_format_args(args...));
  }

private:
  /// Clock used for throttling functionality, defaults to steady time
  rclcpp::Clock clock_{RCL_STEADY_TIME};

  /**
   * @brief Core logging function that formats and outputs messages
   * @param severity The log severity level
   * @param format Format string with source location information
   * @param args Pre-formatted arguments for the message
   *
   * This is the fundamental logging function that all other logging methods use.
   * It handles the interaction with the underlying RCL logging system.
   */
  void log(
    const RCUTILS_LOG_SEVERITY severity, const format_and_loc & format,
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

  /**
   * @brief Logging function that ensures a message is only logged once
   * @param severity The log severity level
   * @param format Format string with source location information
   * @param args Pre-formatted arguments for the message
   *
   * Uses a static boolean per template instantiation to track whether
   * the message has already been logged.
   */
  template <typename Unique>
  void log_once(
    const RCUTILS_LOG_SEVERITY severity, const format_and_loc & format,
    const fmt::format_args & args) const
  {
    static bool already_logged = false;
    if (!already_logged) {
      already_logged = true;
      log(severity, format, args);
    }
  }

  /**
   * @brief Logging function that throttles messages based on time duration
   * @param severity The log severity level
   * @param duration Minimum time between log messages
   * @param format Format string with source location information
   * @param args Pre-formatted arguments for the message
   *
   * Uses a static timestamp per template instantiation to track when the
   * message was last logged and prevents logging more frequently than the
   * specified duration.
   */
  template <typename Unique>
  void log_throttle(
    const RCUTILS_LOG_SEVERITY severity, const rclcpp::Duration & duration,
    const format_and_loc & format, const fmt::format_args & args) const
  {
    static rclcpp::Time last_logged(static_cast<int64_t>(0), clock_.get_clock_type());

    try {
      auto now = clock_.now();
      if ((now - last_logged) >= duration) {
        last_logged = now;
        log(severity, format, args);
      }
    } catch (const rclcpp::exceptions::RCLError & ex) {  // now() can throw
      log(RCUTILS_LOG_SEVERITY_ERROR, "{}", fmt::make_format_args(ex.what()));
      log(severity, format, args);
    }
  }

  /**
   * @brief Logging function that only logs when a monitored value changes
   * @param severity The log severity level
   * @param value The value to monitor for changes
   * @param format Format string with source location information
   * @param args Pre-formatted arguments for the message
   *
   * Uses a static optional variable per template instantiation to store the last
   * seen value and compares it with the current value using operator!=.
   * Does not log the initial value.
   */
  template <typename T, typename Unique>
  void log_on_change(
    const RCUTILS_LOG_SEVERITY severity, const T & value, const format_and_loc & format,
    const fmt::format_args & args) const
  {
    static std::optional<T> last_value;
    if (!last_value || last_value.value() != value) {
      if (last_value.has_value()) {
        log(severity, format, args);
      }
      last_value = value;
    }
  }

  /**
   * @brief Logging function that logs when a value changes by a threshold amount
   * @param severity The log severity level
   * @param value The value to monitor for changes
   * @param threshold Minimum change required to trigger logging
   * @param format Format string with source location information
   * @param args Pre-formatted arguments for the message
   *
   * Uses a static optional variable per template instantiation to store the last
   * logged value and compares the difference with the threshold using operator>=.
   * Does not log the initial value.
   */
  template <typename TV, typename TT, typename Unique>
  void log_on_change(
    const RCUTILS_LOG_SEVERITY severity, const TV & value, const TT & threshold,
    const format_and_loc & format, const fmt::format_args & args) const
  {
    static std::optional<TV> last_value;
    if (!last_value || std::abs(value - last_value.value()) >= threshold) {
      if (last_value.has_value()) {
        log(severity, format, args);
      }
      last_value = value;
    }
  }
};

}  // namespace ros2_fmt_logger
