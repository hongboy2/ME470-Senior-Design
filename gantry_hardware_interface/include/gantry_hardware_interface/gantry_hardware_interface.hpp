#ifndef GANTRY_HARDWARE_INTERFACE_HPP_
#define GANTRY_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "gantry_hardware_interface/Stepper_Driver_TMC2208.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>
#include <string>


using hardware_interface::return_type;
using rclcpp_lifecycle::LifecycleNode;

namespace gantry_hardware_interface
{

class GantryHardwareInterface : public hardware_interface::SystemInterface, public LifecycleNode
{
public:
    GantryHardwareInterface();

    hardware_interface::CallbackReturn  on_init(const hardware_interface::HardwareInfo &info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

    return_type read() override;
    return_type write() override;

    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
    StepperMotor stepper_x_;
    StepperMotor stepper_y_;
    StepperMotor stepper_z_;
    std::vector<uint8_t> motor_ids_;
    std::vector<double> position_states_;
    std::vector<double> position_commands_;
    std::vector<double> position_commands_saved_;
    std::vector<double> rev_counts_to_position(const std::vector<double>& rev_counts);
    std::vector<double> position_to_rev_counts(const std::vector<double>& pos);
};

} // namespace gantry_hardware_interface

#endif // GANTRY_HARDWARE_INTERFACE_HPP_

