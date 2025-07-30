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

class Logger
{
public:
  explicit Logger(const rclcpp::Logger & logger) : rcl_logger_(logger) {}

  template <typename... Args>
  void fatal(const format_string & format, Args &&... args) const
  {
    _fatal(format, fmt::make_format_args(args...));
  }

private:
  rclcpp::Logger rcl_logger_;

  void _fatal(const format_string & format, fmt::format_args args) const
  {
    RCUTILS_LOGGING_AUTOINIT;
    if (rcutils_logging_logger_is_enabled_for(rcl_logger_.get_name(), RCUTILS_LOG_SEVERITY_FATAL)) {
      rcutils_log_location_t rcutils_location{
        .function_name = format.loc.function_name(),
        .file_name = format.loc.file_name(),
        .line_number = static_cast<size_t>(format.loc.line())};
      rcutils_log(
        &rcutils_location, RCUTILS_LOG_SEVERITY_FATAL, rcl_logger_.get_name(), "%s",
        fmt::vformat(format.str, args).c_str());
    }
  }
};

}  // namespace ros2_fmt_logger
