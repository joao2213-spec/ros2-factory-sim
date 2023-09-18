#ifndef PLANSYS2_BT__BEHAVIOR_TREE_NODES__MOVE_HPP_
#define PLANSYS2_BT__BEHAVIOR_TREE_NODES__MOVE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace plansys2_bt_tests
{

class Move : public BT::SyncActionNode
{
public:
  Move(
    const std::string & name,
    const BT::NodeConfiguration & config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("robot"),
      BT::InputPort<std::string>("goal")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::NodeOptions options_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>> action_client_;
};

}  // namespace plansys2_bt_tests

#endif  // PLANSYS2_BT__BEHAVIOR_TREE_NODES__MOVE_HPP_
