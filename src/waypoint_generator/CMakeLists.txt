cmake_minimum_required(VERSION 3.5)
project(waypoint_generator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)


# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
)

# Create the node executables
add_executable(waypoint_generator
  src/waypoint_generator.cpp
)

# Link dependencies for waypoint_generator
ament_target_dependencies(waypoint_generator 
  rclcpp 
  nav_msgs
)

# Añadimos los ejecutables a la carpeta install
install(TARGETS
  waypoint_generator
  DESTINATION lib/${PROJECT_NAME} 
)

#Install include directories
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