#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <pluginlib/class_loader.hpp>
#include <memory>
#include "ros2_control_test_assets/descriptions.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "test_testable_resource_manager.hpp"

class TestLoadGantryHardwareInterface : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    class_loader = std::make_shared<pluginlib::ClassLoader<hardware_interface::SystemInterface>>("hardware_interface", "hardware_interface::SystemInterface");
  }

  void SetUp() override
  {
    hardware_info.type = "gantry_hardware_interface/GantryHardwareInterface";
  }

  static std::shared_ptr<pluginlib::ClassLoader<hardware_interface::SystemInterface>> class_loader;
  hardware_interface::HardwareInfo hardware_info;
};

std::shared_ptr<pluginlib::ClassLoader<hardware_interface::SystemInterface>> TestLoadGantryHardwareInterface::class_loader{nullptr};

TEST_F(TestLoadGantryHardwareInterface, loading)
{
  auto urdf = ros2_control_test_assets::urdf_head + ros2_control_test_assets::generic_system_2dof + ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(TestableResourceManager rm(urdf));
}

