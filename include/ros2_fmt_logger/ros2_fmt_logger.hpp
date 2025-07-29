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
  explicit Logger(const rclcpp::Logger & logger) : rcllog_(logger) {}

  template <typename... Args>
  void fatal(const format_string & format, Args &&... args) const
  {
    _fatal(format, fmt::make_format_args(args...));
  }

private:
  rclcpp::Logger rcllog_;

  void _fatal(const format_string & format, fmt::format_args args) const
  {
    if (rcutils_logging_logger_is_enabled_for(rcllog_.get_name(), RCUTILS_LOG_SEVERITY_FATAL)) {
      std::string formatted_msg = fmt::vformat(format.str, args);
      rcutils_log_location_t rcutils_location = {
        format.loc.function_name(), format.loc.file_name(), static_cast<size_t>(format.loc.line())};
      rcutils_log(
        &rcutils_location, RCUTILS_LOG_SEVERITY_FATAL, rcllog_.get_name(), "%s",
        formatted_msg.c_str());
    }
  }
};

}  // namespace ros2_fmt_logger
