cmake_minimum_required(VERSION 3.5)
project(pure_pursuit_pkg)

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

# Incluir Headers
include_directories(include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
)

# Creamos los ejecutables
add_executable(pure_pursuit_node 
  src/pure_pursuit_node.cpp
  ) 

ament_target_dependencies(pure_pursuit_node  
  rclcpp 
  tf2_ros 
  nav_msgs 
  ackermann_msgs 
  geometry_msgs 
  visualization_msgs
)

# Añadimos los ejecutables a la carpeta install
install(TARGETS
  pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY 
  config 
  launch 
  racelines 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()