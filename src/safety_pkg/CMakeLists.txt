cmake_minimum_required(VERSION 3.8)
project(safety_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# Include directories
include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${ackermann_msgs_INCLUDE_DIRS}
)

# Create the ttc calculator library
add_library(ttc_calculator
  src/ttc_calculator.cpp
)
ament_target_dependencies(ttc_calculator
  sensor_msgs
)

# Create the safety node executable
add_executable(safety_node
  src/main.cpp
  src/safety_node.cpp
)
target_link_libraries(safety_node
  ttc_calculator
)
ament_target_dependencies(safety_node
  rclcpp
  sensor_msgs
  nav_msgs
  ackermann_msgs
)

# Install targets
install(TARGETS
  safety_node
  ttc_calculator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
   
  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
