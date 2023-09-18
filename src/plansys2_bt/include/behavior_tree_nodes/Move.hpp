#ifndef PLANSYS2_BT__BEHAVIOR_TREE_NODES__MOVE_HPP_
#define PLANSYS2_BT__BEHAVIOR_TREE_NODES__MOVE_HPP_

#include <string>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace plansys2_bt_tests
{

class Move : public BT::SyncActionNode
{
public:
  Move(
    const std::string & name,
    const BT::NodeConfiguration & config)
    : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override;
};

}  // namespace plansys2_bt_tests

#endif  // PLANSYS2_BT__BEHAVIOR_TREE_NODES__MOVE_HPP_
