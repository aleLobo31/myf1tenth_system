cmake_minimum_required(VERSION 3.5)
project(f1tenth_stack)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
)

# Create the node executables
add_executable(tf_publisher_node 
  src/main.cpp
  src/tf_publisher_node.cpp
)

# Link dependencies for tf_publisher_node
ament_target_dependencies(tf_publisher_node 
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# Install targets
install(TARGETS 
  tf_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY 
  include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()