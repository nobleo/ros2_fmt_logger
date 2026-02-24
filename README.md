<!-- Copyright (C) 2025 Nobleo Autonomous Solutions B.V. -->

# ros2_fmt_logger

A modern, ROS 2 logging library that provides fmt-style formatting as a replacement for RCLCPP logging macros.

## Features

- Function calls instead of macros: `logger.info("Hello, {}!", name)` instead of `RCLCPP_INFO(logger, "Hello, %s", name.c_str())`
- Additional `.on_change()` method for logging changes in values
- Chrono syntax for throttling: `logger.warn_throttle(1s, "Warning: {}", value)`
- Backwards compatible with the macros, so easy to start using in existing projects without a full rewrite of the current log statements

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
#include <ros2_fmt_logger/logger.hpp>
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

Uses the powerful [fmt library](https://fmt.dev/latest/syntax) format syntax:

```cpp
// Basic formatting
logger.info("Hello, {}!", name);

// Positional arguments
logger.info("Processing {1} of {0} items", total, current);

// Format specifiers
logger.info("Progress: {:.1}%", progress);  // Percentage with 1 decimal
logger.info("Value: {:08.2f}", value);      // Zero-padded floating point
logger.info("Hex: {:#x}", number);          // Hexadecimal with 0x prefix

// Container formatting (requires fmt/ranges.h)
logger.info("Values: {}", std::vector{1, 2, 3, 4});
```

See [demo_ros2_fmt_logger.cpp](demo/demo_ros2_fmt_logger.cpp) for more examples.

## ROS Type Formatting

The optional `rclcpp_formatters.hpp` header provides `fmt` formatters for common ROS types:

```cpp
#include <ros2_fmt_logger/logger.hpp>
#include <ros2_fmt_logger/rclcpp_formatters.hpp>  // IWYU pragma: keep
```

### Supported types

| Type | Example output |
|---|---|
| `rclcpp::Duration` | `0.8s`, `5s` |
| `rclcpp::Time` | `2024-05-11 10:40:00` |
| `rclcpp::(Wall)Rate` | `10Hz`, `0.5Hz` |

```cpp
rclcpp::Duration duration{800ms};
logger.info("Duration: {}", duration);  // "Duration: 0.8s"

rclcpp::Time time = node->get_clock()->now();
// Default human-readable format or custom fmt chrono format specifiers:
logger.info("Default: {}", time);        // "Default: 2026-02-24 08:59:17"
logger.info("Date: {:%Y-%m-%d}", time);  // "Date: 2026-02-24"
```
See [chrono-format-specifications](https://fmt.dev/latest/syntax/#chrono-format-specifications) for the supported format specifiers for `rclcpp::Time` and `rclcpp::Duration`.

## Alternatives

- *[ros2_logging_fmt](https://github.com/facontidavide/ros2_logging_fmt)* similar idea, different implementation
- *[rclcpp_logging](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/include/rclcpp/logging.hpp)* official ROS 2 logging library
- *[rosfmt](https://github.com/xqms/rosfmt)* ROS 1 package for fmt-style logging

## Credits

<!-- markdownlint-disable MD033 -->
<img src="https://nobleo-technology.nl/wp-content/uploads/2024/08/Nobleo_BG-paars_logo-wit-small.jpg" alt="Nobleo Logo" width="300"/>
<!-- markdownlint-enable MD033 -->

This is a package developed by Nobleo Autonomous Solutions B.V.

Looking for paid support options? Contact us at [nobleo-technology.nl](https://nobleo-technology.nl).
