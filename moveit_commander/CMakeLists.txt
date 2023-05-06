cmake_minimum_required(VERSION 3.5)
project(moveit_commander)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Create the executable
add_executable(moveit_commander_node src/moveit_commander_node.cpp)

ament_target_dependencies(moveit_commander_node
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS moveit_commander_node
  DESTINATION lib/${PROJECT_NAME}
)

# Generate package information
ament_package()
