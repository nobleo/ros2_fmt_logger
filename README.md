<!-- Copyright (C) 2025 Nobleo Autonomous Solutions B.V. -->

# ros2_fmt_logger

A modern, ROS 2 logging library that provides fmt-style formatting as a replacement for RCLCPP logging macros.

## Features

- Function calls instead of macros: `logger.info("Hello, {}!", name)` instead of `RCLCPP_INFO(logger, "Hello, %s", name.c_str())`
- Additional `.on_change()` method for logging changes in values
- chrono syntax for throttling: `logger.warn_throttle(1s, "Warning: {}", value)`

## Examples

### Once-only logging

```cpp
logger.info_once("This message will only appear once, no matter how many times called");
```

### Throttled logging

```cpp
using std::chrono_literals::operator""s;
logger.warn_throttle(1s, "This warning appears at most once per second: {}", value);
```

### Change-based logging

```cpp
// Log only when the value changes
logger.info_on_change(sensor_value, "Sensor reading changed to: {}", sensor_value);

// Log only when change exceeds threshold
logger.error_on_change(temperature, 5.0, "Temperature changed significantly: {:.1f}°C", temperature);
```

## Quick Start

### Include in the package.xml

```xml
<depend>ros2_fmt_logger</depend>
```

### Configure c++20 and find_package

```cmake
find_package(ros2_fmt_logger REQUIRED)
target_link_libraries(your_target ros2_fmt_logger::ros2_fmt_logger)
```

### Include the header

```cpp
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>
```

### Create a logger instance

```cpp
// From an rclcpp::Logger
auto logger = ros2_fmt_logger::Logger(node->get_logger());

// With custom clock for throttling features
auto logger = ros2_fmt_logger::Logger(node->get_logger(), node->get_clock());
```

### 3. Use modern logging syntax

```cpp
// Instead of: RCLCPP_INFO(logger, "Processing item %d with value %.2f", id, value);
logger.info("Processing item {} with value {:.2f}", id, value);

// Instead of: RCLCPP_ERROR(logger, "Failed to connect to %s:%d", host.c_str(), port);
logger.error("Failed to connect to {}:{}", host, port);
```

## Format String Syntax

Uses the powerful [fmt library](https://fmt.dev/latest/syntax.html) format syntax:

```cpp
// Basic formatting
logger.info("Hello, {}!", name);

// Positional arguments
logger.info("Processing {1} of {0} items", total, current);

// Format specifiers
logger.info("Progress: {:.1%}", progress);  // Percentage with 1 decimal
logger.info("Value: {:08.2f}", value);     // Zero-padded floating point
logger.info("Hex: {:#x}", number);         // Hexadecimal with 0x prefix

// Container formatting (requires fmt/ranges.h)
logger.info("Values: {}", std::vector{1, 2, 3, 4});
```

See [demo_ros2_fmt_logger.cpp](demo/demo_ros2_fmt_logger.cpp) for more examples.

## Credits

![Nobleo Logo](https://nobleo-technology.nl/wp-content/uploads/2024/08/Nobleo_BG-paars_logo-wit-small.jpg)

This is a package developed by Nobleo Autonomous Solutions B.V.

Looking for paid support options? Contact us at (<https://nobleo-technology.nl>).
