// Copyright (C) 2025 Nobleo Autonomous Solutions B.V.

#pragma once

#include <rclcpp/logger.hpp>

namespace ros2_fmt_logger
{
class Logger
{
public:
  explicit Logger(const rclcpp::Logger & logger) : rcllog_(logger) {}

  //   RCLCPP_FATAL(
  //   old_logger_, "No messages or signals retrieved from dbc-file: %s", HRI_dbcFile_.c_str());
  // logger_.fatal("No messages or signals retrieved from dbc-file: {}", HRI_dbcFile_);

private:
  rclcpp::Logger rcllog_;
};
}  // namespace ros2_fmt_logger
