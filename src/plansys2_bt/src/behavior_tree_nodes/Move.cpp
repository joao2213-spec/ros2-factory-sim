#include "behavior_tree_nodes/Move.hpp"

namespace plansys2_bt_tests
{

Move::Move(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("move_action_node"))
{
  action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    node_, "/carter1/navigate_to_pose");
}

BT::NodeStatus Move::tick()
{
  std::string robot;
  getInput<std::string>("robot", robot);

  std::string goal_str;
  getInput<std::string>("goal", goal_str);

  std::vector<std::string> coords;
  std::istringstream iss(goal_str);
  std::string s;
  while (getline(iss, s, ',')) {
    coords.push_back(s);
  }

  if (coords.size() != 2) {
    return BT::NodeStatus::FAILURE;
  }

  double goal_x = std::stod(coords[0]);
  double goal_y = std::stod(coords[1]);

  nav2_msgs::action::NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = goal_x;
  goal_msg.pose.pose.position.y = goal_y;
  goal_msg.pose.pose.orientation.w = 1.0;

  auto future_goal_handle = action_client_->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node_, future_goal_handle) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return BT::NodeStatus::FAILURE;
  }

  auto goal_handle = future_goal_handle.get();
  if (!goal_handle) {
    return BT::NodeStatus::FAILURE;
  }

  auto future_result = action_client_->async_get_result(goal_handle);
  rclcpp::spin_until_future_complete(node_, future_result);

  auto result = future_result.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace plansys2_bt_tests

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plansys2_bt_tests::Move>("Move");
}
