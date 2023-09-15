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

#include "plansys2_domain_expert/DomainExpertNode.hpp"

#include <string>
#include <memory>
#include <vector>

#include "plansys2_core/Utils.hpp"
#include "plansys2_popf_plan_solver/popf_plan_solver.hpp"

#include "lifecycle_msgs/msg/state.hpp"

namespace plansys2
{

DomainExpertNode::DomainExpertNode()
: rclcpp_lifecycle::LifecycleNode("domain_expert")
{
  declare_parameter("model_file", "");

  get_types_service_ = create_service<plansys2_msgs::srv::GetDomainTypes>(
    "domain_expert/get_domain_types",
    std::bind(
      &DomainExpertNode::get_domain_types_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
  get_domain_actions_service_ = create_service<plansys2_msgs::srv::GetDomainActions>(
    "domain_expert/get_domain_actions",
    std::bind(
      &DomainExpertNode::get_domain_actions_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
  get_domain_action_details_service_ =
    create_service<plansys2_msgs::srv::GetDomainActionDetails>(
    "domain_expert/get_domain_action_details", std::bind(
      &DomainExpertNode::get_domain_action_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
  get_domain_durative_actions_service_ = create_service<plansys2_msgs::srv::GetDomainActions>(
    "domain_expert/get_domain_durative_actions",
    std::bind(
      &DomainExpertNode::get_domain_durative_actions_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
  get_domain_durative_action_details_service_ =
    create_service<plansys2_msgs::srv::GetDomainDurativeActionDetails>(
    "domain_expert/get_domain_durative_action_details", std::bind(
      &DomainExpertNode::get_domain_durative_action_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
  get_domain_predicates_service_ = create_service<plansys2_msgs::srv::GetStates>(
    "domain_expert/get_domain_predicates", std::bind(
      &DomainExpertNode::get_domain_predicates_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
  get_domain_predicate_details_service_ =
    create_service<plansys2_msgs::srv::GetNodeDetails>(
    "domain_expert/get_domain_predicate_details", std::bind(
      &DomainExpertNode::get_domain_predicate_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
  get_domain_functions_service_ = create_service<plansys2_msgs::srv::GetStates>(
    "domain_expert/get_domain_functions", std::bind(
      &DomainExpertNode::get_domain_functions_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
  get_domain_function_details_service_ =
    create_service<plansys2_msgs::srv::GetNodeDetails>(
    "domain_expert/get_domain_function_details", std::bind(
      &DomainExpertNode::get_domain_function_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
  get_domain_service_ = create_service<plansys2_msgs::srv::GetDomain>(
    "domain_expert/get_domain", std::bind(
      &DomainExpertNode::get_domain_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
}


using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
DomainExpertNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Configuring...", get_name());
  std::string model_file = get_parameter("model_file").get_value<std::string>();

  auto model_files = tokenize(model_file, ":");

  std::ifstream domain_ifs(model_files[0]);
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  domain_expert_ = std::make_shared<DomainExpert>(domain_str);

  std::string check = planner->check_domain(domain_expert_->getDomain(), get_namespace());
  if (!check.empty()) {
    RCLCPP_ERROR_STREAM(get_logger(), "PDDL syntax error: \n" << check);
    return CallbackReturnT::FAILURE;
  }

  for (size_t i = 1; i < model_files.size(); i++) {
    std::ifstream domain_ifs(model_files[i]);
    std::string domain_str((
        std::istreambuf_iterator<char>(domain_ifs)),
      std::istreambuf_iterator<char>());
    domain_expert_->extendDomain(domain_str);

    std::string check = planner->check_domain(domain_expert_->getDomain(), get_namespace());

    if (!check.empty()) {
      RCLCPP_ERROR_STREAM(get_logger(), "PDDL syntax error: \n" << check);
      return CallbackReturnT::FAILURE;
    }
  }

  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DomainExpertNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DomainExpertNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DomainExpertNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DomainExpertNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DomainExpertNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_ERROR(get_logger(), "[%s] Error transition", get_name());

  return CallbackReturnT::SUCCESS;
}

void
DomainExpertNode::get_domain_types_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainTypes::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainTypes::Response> response)
{
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->types = domain_expert_->getTypes();
  }
}

void
DomainExpertNode::get_domain_actions_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainActions::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainActions::Response> response)
{
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    for (const auto & action : domain_expert_->getActions()) {
      response->actions.push_back(action);
    }
  }
}

void
DomainExpertNode::get_domain_action_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainActionDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainActionDetails::Response> response)
{
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";

    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto action = domain_expert_->getAction(request->action, request->parameters);

    if (action) {
      response->action = *action;
      response->success = true;
    } else {
      RCLCPP_WARN(get_logger(), "Requesting a non-existing action [%s]", request->action.c_str());
      response->success = false;
      response->error_info = "Action not found";
    }
  }
}

void
DomainExpertNode::get_domain_durative_actions_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainActions::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainActions::Response> response)
{
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    for (const auto & action : domain_expert_->getDurativeActions()) {
      response->actions.push_back(action);
    }
  }
}

void
DomainExpertNode::get_domain_durative_action_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainDurativeActionDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainDurativeActionDetails::Response> response)
{
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";

    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto action = domain_expert_->getDurativeAction(request->durative_action, request->parameters);

    if (action) {
      response->durative_action = *action;
      response->success = true;
    } else {
      RCLCPP_WARN(
        get_logger(), "Requesting a non-existing durative action [%s]",
        request->durative_action.c_str());
      response->success = false;
      response->error_info = "Durative action not found";
    }
  }
}

void
DomainExpertNode::get_domain_predicates_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response)
{
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->states = plansys2::convertVector<plansys2_msgs::msg::Node, plansys2::Predicate>(
      domain_expert_->getPredicates());
  }
}

void
DomainExpertNode::get_domain_predicate_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response)
{
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto predicate = domain_expert_->getPredicate(request->expression);
    if (predicate) {
      response->node = predicate.value();
      response->success = true;
    } else {
      RCLCPP_WARN(
        get_logger(), "Requesting a non-existing predicate [%s]",
        request->expression);
      response->success = false;
      response->error_info = "Predicate not found";
    }
  }
}

void
DomainExpertNode::get_domain_functions_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response)
{
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->states = plansys2::convertVector<plansys2_msgs::msg::Node, plansys2::Function>(
      domain_expert_->getFunctions());
  }
}

void
DomainExpertNode::get_domain_function_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response)
{
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto function = domain_expert_->getFunction(request->expression);
    if (function) {
      response->node = function.value();
      response->success = true;
    } else {
      RCLCPP_WARN(
        get_logger(), "Requesting a non-existing function [%s]",
        request->expression);
      response->success = false;
      response->error_info = "Function not found";
    }
  }
}

void
DomainExpertNode::get_domain_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomain::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomain::Response> response)
{
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;

    std::ostringstream stream;
    stream << domain_expert_->getDomain();
    response->domain = stream.str();
  }
}


}  // namespace plansys2
