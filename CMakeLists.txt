# Copyright (C) 2025 Nobleo Autonomous Solutions B.V.

cmake_minimum_required(VERSION 3.28)
project(ros2_fmt_logger)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#add_compile_options(-Wall -Wextra -Wfloat-conversion)
#add_link_options(-Wl,--no-undefined)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_library(${PROJECT_NAME} INTERFACE)

# Set up include directories for the target
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Ensure C++20 features are available
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)
# add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Install the target
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Export the target so it can be used by other packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rcutils fmt)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Unit tests
  ament_add_gtest(test_ros2_fmt_logger test/test_ros2_fmt_logger.cpp)
  target_link_libraries(test_ros2_fmt_logger
    ${PROJECT_NAME}
    fmt::fmt
    rclcpp::rclcpp
    rcutils::rcutils
  )

  # Demo executable
  add_executable(demo_ros2_fmt_logger demo/demo_ros2_fmt_logger.cpp)
  target_link_libraries(demo_ros2_fmt_logger PUBLIC
    ${PROJECT_NAME}
    fmt::fmt
    rclcpp::rclcpp
    rcutils::rcutils
  )
  install(TARGETS demo_ros2_fmt_logger
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if($ENV{ROS_DISTRO} STREQUAL "jazzy")  # Can be removed after Jazzy EOL or https://github.com/ament/ament_cmake/pull/596
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
else()
  ament_auto_package()
endif()
