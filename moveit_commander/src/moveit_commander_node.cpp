#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.use_global_arguments(false);
  auto node = rclcpp::Node::make_shared("moveit_commander_cpp", "", node_options);

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "end_effector_group");

  // Define multiple joint state targets
  std::vector<std::vector<double>> joint_state_targets = {
    {-0.135,0.122, 0},
    {-0.135, 0.122, -0.048},
    {-0.135, 0.122 , 0 },
    {-0.135, -0.120, 0 },
    {-0.135, -0.120, -0.048},
    {-0.135, -0.120, 0},
    {0.001, -0.120, 0 },
    {0.001, -0.120, -0.048},
    {0.001, -0.120, 0},
    {0.001, 0.140, 0},
    {0.001, 0.140, -0.048},
    {0.001,0.140,0}
  };

  for (const auto& joint_state_target : joint_state_targets)
  {
    // Set a joint state target
    move_group_interface.setJointValueTarget(joint_state_target);

    // Create a plan to that target joint state
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_interface.plan(plan));

    // Execute the plan
    if (success)
    {
      move_group_interface.execute(plan);
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    }
  }

  rclcpp::shutdown();
  return 0;
}

