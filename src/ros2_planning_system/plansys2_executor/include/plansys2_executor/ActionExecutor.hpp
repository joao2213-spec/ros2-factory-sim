// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLANSYS2_EXECUTOR__ACTIONEXECUTOR_HPP_
#define PLANSYS2_EXECUTOR__ACTIONEXECUTOR_HPP_

#include <string>
#include <memory>
#include <vector>

#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2
{

class ActionExecutor
{
public:
  enum Status
  {
    IDLE,
    DEALING,
    RUNNING,
    SUCCESS,
    FAILURE,
    CANCELLED
  };

  using Ptr = std::shared_ptr<ActionExecutor>;
  static Ptr make_shared(
    const std::string & action,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  {
    return std::make_shared<ActionExecutor>(action, node);
  }

  explicit ActionExecutor(
    const std::string & action, rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  BT::NodeStatus tick(const rclcpp::Time & now);
  void cancel();
  BT::NodeStatus get_status();
  bool is_finished();

  // Methods for debug
  Status get_internal_status() const {return state_;}
  void set_internal_status(Status state) {state_ = state;}
  std::string get_action_name() const {return action_name_;}
  std::vector<std::string> get_action_params() const {return action_params_;}
  plansys2_msgs::msg::ActionExecution last_msg;

  rclcpp::Time get_start_time() const {return start_execution_;}
  rclcpp::Time get_status_time() const {return state_time_;}

  std::string get_feedback() const {return feedback_;}
  float get_completion() const {return completion_;}

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  Status state_;
  rclcpp::Time state_time_;
  rclcpp::Time start_execution_;

  std::string action_;
  std::string action_name_;
  std::string current_performer_id_;
  std::vector<std::string> action_params_;

  std::string feedback_;
  float completion_;

  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::ActionExecution>::SharedPtr
    action_hub_pub_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr action_hub_sub_;

  void action_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg);
  void request_for_performers();
  void confirm_performer(const std::string & node_id);
  void reject_performer(const std::string & node_id);

  std::string get_name(const std::string & action_expr);
  std::vector<std::string> get_params(const std::string & action_expr);

  void wait_timeout();
  rclcpp::TimerBase::SharedPtr waiting_timer_;
};

struct ActionExecutionInfo
{
  std::shared_ptr<ActionExecutor> action_executor = {nullptr};
  bool at_start_effects_applied = {false};
  bool at_end_effects_applied = {false};
  std::shared_ptr<plansys2_msgs::msg::DurativeAction> durative_action_info = {nullptr};
  std::string execution_error_info;
  double duration;
  double duration_overrun_percentage = -1.0;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__ACTIONEXECUTOR_HPP_
