cmake_minimum_required(VERSION 3.5)
project(pure_pursuit)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Inclumos Headers
include_directories(include)

# Creamos los ejecutables
add_executable(pure_pursuit src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit rclcpp tf2_ros nav_msgs ackermann_msgs geometry_msgs visualization_msgs)

# Añadimos los ejecutables a la carpeta install
install(TARGETS
  pure_pursuit
  DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()