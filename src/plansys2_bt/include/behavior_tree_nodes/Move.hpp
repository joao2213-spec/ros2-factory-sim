#ifndef PLANSYS2_BT__BEHAVIOR_TREE_NODES__MOVE_HPP_
#define PLANSYS2_BT__BEHAVIOR_TREE_NODES__MOVE_HPP_

#include <string>
#include <map>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "plansys2_bt_actions/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace plansys2_bt_tests
{

class Move : public plansys2::BtActionNode<
    nav2_msgs::action::NavigateToPose>
{
public:
  explicit Move(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("goal")
    };
  }

private:
  int goal_reached_;
  std::map<std::string, geometry_msgs::msg::Pose2D> waypoints_;
};

}

#endif