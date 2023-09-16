#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "behavior_tree_nodes/Move.hpp"


#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2_bt_tests
{

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: plansys2::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  node->declare_parameter("waypoints");
  node->declare_parameter("waypoint_coords");

  if (node->has_parameter("waypoints")) {
    std::vector<std::string> wp_names;

    node->get_parameter_or("waypoints", wp_names, {});

    for (auto & wp : wp_names) {
      node->declare_parameter("waypoint_coords." + wp);

      std::vector<double> coords;
      if (node->get_parameter_or("waypoint_coords." + wp, coords, {})) {
        geometry_msgs::msg::Pose2D pose;
        pose.x = coords[0];
        pose.y = coords[1];
        pose.theta = coords[2];

        waypoints_[wp] = pose;
      } else {
        std::cerr << "No coordinate configured for waypoint [" << wp << "]" << std::endl;
      }
    }
  }
}

void
Move::on_tick()
{
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  std::cout << "Executing shell command...\n";
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  std::string goal;
  getInput<std::string>("goal", goal);

  geometry_msgs::msg::Pose2D pose2nav;
  if (waypoints_.find(goal) != waypoints_.end()) {
    pose2nav = waypoints_[goal];
  } else {
    std::cerr << "No coordinate for waypoint [" << goal << "]" << std::endl;
  }

  geometry_msgs::msg::PoseStamped goal_pos;

  goal_pos.header.frame_id = "map";
  goal_pos.header.stamp = node->now();
  goal_pos.pose.position.x = pose2nav.x;
  goal_pos.pose.position.y = pose2nav.y;
  goal_pos.pose.position.z = 0;
  goal_pos.pose.orientation = tf2::toMsg(tf2::Quaternion({0.0, 0.0, 1.0}, pose2nav.theta));

  goal_.pose = goal_pos;
}

BT::NodeStatus
Move::on_success()
{
  return BT::NodeStatus::SUCCESS;
}


}  // namespace plansys2_bt_tests

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<plansys2_bt_tests::Move>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<plansys2_bt_tests::Move>(
    "Move", builder);
}