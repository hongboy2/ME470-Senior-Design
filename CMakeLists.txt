cmake_minimum_required(VERSION 3.8)
project(robot_gantry_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)
#find_package(joint_state_publisher REQUIRED)
#find_package(robot_state_publisher REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    urdf
    launch
    meshes
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
