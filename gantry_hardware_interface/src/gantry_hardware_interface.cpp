#include "gantry_hardware_interface/gantry_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <hardware_interface/system_interface.hpp>
#include "Stepper_Driver_TMC2208.hpp"
#include "rclcpp/rclcpp.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>
#include <string>

namespace gantry_hardware_interface
{

GantryHardwareInterface::GantryHardwareInterface()
: LifecycleNode("gantry_hardware_interface")
{
}

//helper function that translates rev_counts to position in meters
std::vector<double> rev_counts_to_position(const std::vector<double>& rev_counts) {
    double lead_screw_pitch = 2.0; // mm
    double motor_steps_per_revolution = 200;

    double steps_per_mm = motor_steps_per_revolution / lead_screw_pitch;

    std::vector<double> pos(rev_counts.size());
    for (size_t i = 0; i < rev_counts.size(); ++i) {
        pos[i] = rev_counts[i] * motor_steps_per_revolution / steps_per_mm;
    }

    return pos;
}

//helper function that translates position in meters to rev_counts
std::vector<double> position_to_rev_counts(const std::vector<double>& pos) {
    double lead_screw_pitch = 2.0; // mm
    double motor_steps_per_revolution = 200;

    double steps_per_mm = motor_steps_per_revolution / lead_screw_pitch;

    std::vector<double> rev_counts(pos.size());
    for (size_t i = 0; i < pos.size(); ++i) {
        rev_counts[i] = pos[i] * steps_per_mm / motor_steps_per_revolution;
    }

    return rev_counts;
}

hardware_interface::CallbackReturn GantryHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{   
    // Call the parent's on_init method to process standard values like name
    hardware_interface::SystemInterface::on_init(info);

    // Initialize your hardware interface here
    stepper_x_ = StepperMotor(/* parameters */);
    stepper_y_ = StepperMotor(/* parameters */);
    stepper_z_ = StepperMotor(/* parameters */);


    //initialize the position states and commands
    position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    position_commands_saved_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    motor_ids_.resize(info_.joints.size(), std::numeric_limits<uint8_t>::quiet_NaN());


    return hardware::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GantryHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    // Configure your hardware interface here
    //Write the on_configure method where you usually setup the communication to the hardware and set everything up so that the hardware can be activated.
    // Configure the stepper motor direction, movement mode, power, and enable the motor.
    // Assuming you have methods setDirection, setPowered, setEnabled, and setMovementMode in your StepperMotor class.
    // Replace with appropriate method names and arguments if necessary.

    // Configure stepper_x_
    stepper_x_.setDirection(/* your direction value for x */);
    stepper_x_.setMovementMode(/* your movement mode value for x */);
    stepper_x_.setPowered(/* your power value for x */);
    stepper_x_.setEnabled(/* true to enable the motor */);

    // Configure stepper_y_
    stepper_y_.setDirection(/* your direction value for y */);
    stepper_y_.setMovementMode(/* your movement mode value for y */);
    stepper_y_.setPowered(/* your power value for y */);
    stepper_y_.setEnabled(/* true to enable the motor */);

    // Configure stepper_z_
    stepper_z_.setDirection(/* your direction value for z */);
    stepper_z_.setMovementMode(/* your movement mode value for z */);
    stepper_z_.setPowered(/* your power value for z */);
    stepper_z_.setEnabled(/* true to enable the motor */);

    // Optionally, log a message indicating successful configuration.
    RCLCPP_INFO(this->get_logger(), "GantryHardwareInterface configured");

    // Return success if the configuration process completed successfully.
 
    return hardware::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> export_state_interfaces() {

    RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"), "Adding position state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]
            )
        );
    }

}


std::vector<hardware_interface::CommandInterface> export_command_interfaces() {
    
        RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"), "export_command_interfaces");
    
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++) {
            RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"), "Adding position command interface: %s", info_.joints[i].name.c_str());
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]
                )
            );
        }
}

hardware_interface::CallbackReturn GantryHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    // Activate your hardware interface here
    //Implement the on_activate method where hardware “power” is enabled. This is where you would start the motors
    //perform homing 
        // Perform the homing procedure for each axis
    if (!stepper_x_.home())
    {
        RCLCPP_ERROR(get_logger(), "Failed to home the X axis");
        return hardware::CallbackReturn::ERROR;
    }
    if (!stepper_y_.home())
    {
        RCLCPP_ERROR(get_logger(), "Failed to home the Y axis");
        return hardware::CallbackReturn::ERROR;
    }
    if (!stepper_z_.home())
    {
        RCLCPP_ERROR(get_logger(), "Failed to home the Z axis");
        return hardware::CallbackReturn::ERROR;
    }

    // Update the position state values after homing
    position_state_[0] = 0.0;
    position_state_[1] = 0.0;
    position_state_[2] = 0.0;

    RCLCPP_INFO(get_logger(), "Gantry system successfully homed");
    return hardware::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GantryHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    // Deactivate your hardware interface here
    // Add any necessary code to safely deactivate your hardware.
    // For example, you can stop the stepper motors:
    stepper_x_.stop();
    stepper_y_.stop();
    stepper_z_.stop();

    // Optionally, you can log a message:
    RCLCPP_INFO(this->get_logger(), "GantryHardwareInterface deactivated");

    // Return success if the deactivation process completed successfully.
    return hardware::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GantryHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    // Clean up your hardware interface here
    // we might not need it for this project
    return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn GantryHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    // Shutdown your hardware interface here
    // we might not need it for this project
    return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn GantryHardwareInterface::on_error(const rclcpp_lifecycle::State &previous_state)
{
    // Handle errors in your hardware interface here
    // we might not need it for this project
    return hardware_interface::return_type::OK;
}

return_type GantryHardwareInterface::read()
{
    // Read from your hardware interface here
    // Return a value indicating success or failure
    // Read absolute revolution counts for each axis
    double rev_counts_x = stepper_x_.getRevCountsAbs();
    double rev_counts_y = stepper_y_.getRevCountsAbs();
    double rev_counts_z = stepper_z_.getRevCountsAbs();
    //convert to meters
    std::vector<double> pos = rev_counts_to_position(rev_counts_x, rev_counts_y, rev_counts_z);
    // Update the position state values
    position_state_[0] = pos[0];
    position_state_[1] = pos[1];
    position_state_[2] = pos[2];
    
    return hardware_interface::return_type::OK;
}

return_type GantryHardwareInterface::write()
{
    // Write to your hardware interface here
    // Return a value indicating success or failure
    // Convert the position command values to revolutions
    std::vector<double> rev_counts = position_to_rev_counts(position_command_[0], position_command_[1], position_command_[2]);
    // apply the command to the stepper motors
    stepper_x_.runToRevCount(rev_counts[0]);
    stepper_y_.runToRevCount(rev_counts[1]);
    stepper_z_.runToRevCount(rev_counts[2]);
    return hardware_interface::return_type::OK;
}

} // namespace gantry_hardware_interface

PLUGINLIB_EXPORT_CLASS(gantry_hardware_interface::GantryHardwareInterface, hardware_interface::SystemInterface) 
