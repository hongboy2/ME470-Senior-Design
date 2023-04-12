cmake_minimum_required(VERSION 3.5)
project(gantry_hardware_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(gantry_hardware_interface SHARED
  src/gantry_hardware_interface.cpp
  src/Stepper_Driver_TMC2208.cpp # Add this line
)

target_include_directories(gantry_hardware_interface PRIVATE
  include
)

ament_target_dependencies(gantry_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface my_hardware_interface_package.xml)

install(TARGETS gantry_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add the test executable
  ament_add_gmock(test_load_gantry_hardware_interface test/test_load_gantry_hardware_interface.cpp)

  # Link the test executable to the necessary libraries
  target_link_libraries(test_load_gantry_hardware_interface gantry_hardware_interface)
endif()

ament_export_libraries(gantry_hardware_interface)
ament_package()

