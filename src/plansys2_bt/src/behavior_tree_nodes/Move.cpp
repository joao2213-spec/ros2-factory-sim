#include <iostream>
#include "behavior_tree_nodes/Move.hpp"

namespace plansys2_bt_tests
{

BT::NodeStatus
Move::tick()
{
  std::cout << "Hello, World!" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace plansys2_bt_tests

#include "behaviortree_cpp_v3/bt_factory.h"

BT::NodeBuilder getNodeBuilderForMove()
{
    return [](const std::string& name, const BT::NodeConfiguration& config) -> std::unique_ptr<BT::TreeNode> {
        return std::make_unique<plansys2_bt_tests::Move>(name, config);
    };
}

BT_REGISTER_NODES(factory)
{
    factory.registerBuilder<plansys2_bt_tests::Move>("Move", getNodeBuilderForMove());
}

