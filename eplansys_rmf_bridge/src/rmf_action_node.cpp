// Copyright 2026 Haniel Ulises
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

// The bridge: PlanSys2 performers that do their work by asking Open-RMF.
//
// One performer per action in the task map. A performer that moves a robot
// submits an RMF task pinned to the robot its agent is bound to, waits, and
// finishes with whatever the robot observed. A performer for a speech act
// submits nothing, because RMF has no representation of saying things and no
// robot moves to say them.
//
// What makes this more than a remote control is the last argument to finish().
// A sensing action's whole point is to find something out, and the policy
// branches on what it found. RMF will not carry that by itself, so the token
// travels in one of the two free-form fields a task state does carry and is
// lifted back out here.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "eplansys_rmf_bridge/RmfTaskClient.hpp"
#include "eplansys_rmf_bridge/TaskMapping.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

namespace eplansys_rmf_bridge
{

class RmfAction : public plansys2::ActionExecutorClient
{
public:
  RmfAction(
    const std::string & node_name,
    const std::string & action,
    ActionSpec spec,
    std::shared_ptr<TaskMapping> mapping,
    std::shared_ptr<RmfTaskClient> client,
    double timeout)
  : ActionExecutorClient(node_name),
    action_(action),
    spec_(std::move(spec)),
    mapping_(std::move(mapping)),
    client_(std::move(client)),
    timeout_(timeout)
  {
    set_parameter(rclcpp::Parameter("action_name", action));
    set_parameter(rclcpp::Parameter("rate", 4.0));
  }

private:
  void do_work() override
  {
    if (!started_) {
      started_ = true;
      begun_ = now();
      request_id_.clear();

      if (spec_.local) {
        RCLCPP_INFO(
          get_logger(), "%s: speech act, nothing for RMF to do (%.1fs)",
          action_.c_str(), spec_.duration);
        return;
      }

      if (!begin_task()) {
        conclude(false, "no RMF task could be submitted", "");
      }
      return;
    }

    if (spec_.local) {
      run_local();
      return;
    }

    run_task();
  }

  /// The acting agent is the action's first argument. Every action in the
  /// survey domain is written that way, and the epistemic model records what
  /// is learnt against that name, so it is also the name that has to decide
  /// which robot moves.
  std::string acting_agent() const
  {
    const auto & args = get_arguments();
    return args.empty() ? std::string{} : args.front();
  }

  bool begin_task()
  {
    const auto agent = acting_agent();
    if (agent.empty()) {
      RCLCPP_ERROR(get_logger(), "%s: no agent argument", action_.c_str());
      return false;
    }

    const auto binding = mapping_->agent(agent);
    if (!binding.has_value()) {
      RCLCPP_ERROR(
        get_logger(),
        "%s: agent \"%s\" is bound to no robot. Submitting anyway would let "
        "RMF pick, and the model would credit %s with sensing something it "
        "never saw.", action_.c_str(), agent.c_str(), agent.c_str());
      return false;
    }

    const auto waypoint = mapping_->waypoint_for(spec_, agent);
    if (waypoint.empty()) {
      RCLCPP_ERROR(
        get_logger(), "%s: no waypoint for agent \"%s\"",
        action_.c_str(), agent.c_str());
      return false;
    }

    const int64_t start_millis = now().nanoseconds() / 1000000;
    const std::vector<std::string> labels{
      "eplansys.action=" + action_,
      "eplansys.agent=" + agent,
    };

    const auto request = RmfTaskClient::go_to_place(
      waypoint, spec_.orientation, start_millis, labels);

    request_id_ = client_->submit_to_robot(
      binding->fleet, binding->robot, request);

    RCLCPP_INFO(
      get_logger(), "%s: %s -> %s/%s heading for %s",
      action_.c_str(), agent.c_str(), binding->fleet.c_str(),
      binding->robot.c_str(), waypoint.c_str());
    return true;
  }

  void run_local()
  {
    const auto elapsed = (now() - begun_).seconds();
    if (elapsed < spec_.duration) {
      send_feedback(
        static_cast<float>(elapsed / spec_.duration), "speaking");
      return;
    }
    conclude(true, "done", resolve_outcome(""));
  }

  void run_task()
  {
    const auto status = client_->status(request_id_);
    const auto elapsed = (now() - begun_).seconds();

    switch (status.phase) {
      case TaskPhase::Rejected:
        conclude(false, "RMF refused the task", "");
        return;

      case TaskPhase::Succeeded:
        conclude(true, "done", resolve_outcome(status.outcome));
        return;

      case TaskPhase::Failed:
        conclude(false, status.error, "");
        return;

      case TaskPhase::Pending:
      case TaskPhase::Running:
      default:
        break;
    }

    if (timeout_ > 0.0 && elapsed > timeout_) {
      RCLCPP_ERROR(
        get_logger(), "%s: RMF task %s still %s after %.0fs",
        action_.c_str(),
        status.task_id.empty() ? "(unacknowledged)" : status.task_id.c_str(),
        status.rmf_status.empty() ? "unreported" : status.rmf_status.c_str(),
        elapsed);
      conclude(false, "RMF task timed out", "");
      return;
    }

    send_feedback(0.5f, status.rmf_status.empty() ? "submitted" : status.rmf_status);
  }

  /// An ordinary action reports nothing. A sensing action reports what the
  /// robot saw, and falls back to the configured answer when the fleet carried
  /// no token, saying so, because a fleet adapter that knows nothing of
  /// ePlanSys writes none and a simulated robot has nothing to sense with.
  std::string resolve_outcome(const std::string & carried)
  {
    if (!spec_.sensing) {
      return {};
    }

    if (!carried.empty()) {
      RCLCPP_INFO(
        get_logger(), "%s: observed %s", action_.c_str(), carried.c_str());
      return carried;
    }

    if (spec_.default_outcome.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "%s: sensing action carried no outcome and has no default_outcome. "
        "The policy has nothing to branch on.", action_.c_str());
      return {};
    }

    RCLCPP_WARN(
      get_logger(),
      "%s: RMF carried no outcome, falling back to default_outcome \"%s\". "
      "A fleet adapter reporting what it sensed would override this.",
      action_.c_str(), spec_.default_outcome.c_str());
    return spec_.default_outcome;
  }

  void conclude(bool success, const std::string & status, const std::string & outcome)
  {
    finish(success, 1.0, status, outcome);
    started_ = false;
    request_id_.clear();
  }

  std::string action_;
  ActionSpec spec_;
  std::shared_ptr<TaskMapping> mapping_;
  std::shared_ptr<RmfTaskClient> client_;
  double timeout_;

  bool started_{false};
  rclcpp::Time begun_;
  std::string request_id_;
};

}  // namespace eplansys_rmf_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto config = std::make_shared<rclcpp::Node>("eplansys_rmf_bridge");
  config->declare_parameter("task_map", std::string{});
  config->declare_parameter("websocket_port", 7879);
  config->declare_parameter("outcome_prefix", std::string{"eplansys.outcome="});
  config->declare_parameter("task_timeout", 120.0);

  const auto task_map = config->get_parameter("task_map").as_string();
  const auto port = static_cast<int>(
    config->get_parameter("websocket_port").as_int());
  const auto prefix = config->get_parameter("outcome_prefix").as_string();
  const auto timeout = config->get_parameter("task_timeout").as_double();

  if (task_map.empty()) {
    RCLCPP_FATAL(
      config->get_logger(),
      "no task_map parameter. It says which robot each epistemic agent is and "
      "where each action sends it, and nothing can be dispatched without it.");
    return 1;
  }

  std::shared_ptr<eplansys_rmf_bridge::TaskMapping> mapping;
  try {
    mapping = std::make_shared<eplansys_rmf_bridge::TaskMapping>(
      eplansys_rmf_bridge::TaskMapping::load(task_map));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(config->get_logger(), "%s", e.what());
    return 1;
  }

  auto client = std::make_shared<eplansys_rmf_bridge::RmfTaskClient>(
    config, port, prefix);

  // One process for every performer: they share the websocket the fleet
  // adapter dials, and a port can only be bound once.
  std::vector<std::shared_ptr<eplansys_rmf_bridge::RmfAction>> actions;
  for (const auto & name : mapping->action_names()) {
    const auto spec = mapping->action(name);
    actions.push_back(
      std::make_shared<eplansys_rmf_bridge::RmfAction>(
        name + "_rmf_node", name, *spec, mapping, client, timeout));
    RCLCPP_INFO(
      config->get_logger(), "performer for \"%s\"%s", name.c_str(),
      spec->local ? " (speech act, no RMF task)" : "");
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(config);
  for (const auto & action : actions) {
    action->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    executor.add_node(action->get_node_base_interface());
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
