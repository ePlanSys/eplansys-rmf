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
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "eplansys_rmf_bridge/Announcer.hpp"
#include "eplansys_rmf_bridge/Observations.hpp"
#include "eplansys_rmf_bridge/RmfTaskClient.hpp"
#include "eplansys_rmf_bridge/TaskMapping.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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
    std::shared_ptr<Announcer> announcer,
    double timeout,
    const std::string & observation_topic)
  : ActionExecutorClient(node_name),
    action_(action),
    spec_(std::move(spec)),
    mapping_(std::move(mapping)),
    client_(std::move(client)),
    announcer_(std::move(announcer)),
    timeout_(timeout)
  {
    set_parameter(rclcpp::Parameter("action_name", action));
    set_parameter(rclcpp::Parameter("rate", 4.0));

    // Only a sensing action has anything to do with an observation, and
    // subscribing from the others would let one action's reading answer for
    // another's.
    if (spec_.sensing && !observation_topic.empty()) {
      // Deep enough to hold one reading per place a sensing action can be
      // sent. The topic is latched, and with a depth of one a late subscriber
      // is handed only the most recent site's reading -- which for a domain
      // with several sites is an arbitrary one of them.
      rclcpp::QoS qos(16);
      qos.reliable().transient_local();
      observation_sub_ = create_subscription<std_msgs::msg::String>(
        observation_topic, qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          const auto reading = observations_.record(msg->data, now().nanoseconds());
          if (!reading.changed) {
            return;
          }
          if (reading.place.empty()) {
            RCLCPP_INFO(
              get_logger(), "%s: observation available: %s",
              action_.c_str(), reading.outcome.c_str());
          } else {
            RCLCPP_INFO(
              get_logger(), "%s: observation available at %s: %s",
              action_.c_str(), reading.place.c_str(), reading.outcome.c_str());
          }
        });
    }
  }

private:
  void do_work() override
  {
    if (!started_) {
      started_ = true;
      begun_ = now();
      request_ids_.clear();

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

  /// Which agents this action moves, and where each of them goes.
  ///
  /// An ordinary action moves the agent named by its first argument. An action
  /// declaring `movements` moves several at once, which is the only way robots
  /// move together: the executor runs policy nodes strictly in order, so
  /// concurrency has to be inside one action rather than between two.
  std::vector<std::pair<std::string, std::string>> legs() const
  {
    const auto & args = get_arguments();
    std::vector<std::pair<std::string, std::string>> out;

    if (spec_.movements.empty()) {
      out.emplace_back(
        acting_agent(), mapping_->waypoint_for(spec_, acting_agent(), args));
      return out;
    }

    for (const auto & movement : spec_.movements) {
      const auto agent_index = static_cast<std::size_t>(movement.agent_arg);
      const auto zone_index = static_cast<std::size_t>(movement.waypoint_arg);
      if (movement.agent_arg < 0 || movement.waypoint_arg < 0 ||
        agent_index >= args.size() || zone_index >= args.size())
      {
        out.emplace_back(std::string{}, std::string{});
        continue;
      }
      out.emplace_back(
        args[agent_index], mapping_->waypoint_of_zone(args[zone_index]));
    }
    return out;
  }

  bool begin_task()
  {
    request_ids_.clear();

    for (const auto & [agent, waypoint] : legs()) {
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

      if (waypoint.empty()) {
        RCLCPP_ERROR(
          get_logger(), "%s: no waypoint for agent \"%s\"",
          action_.c_str(), agent.c_str());
        return false;
      }

      const std::vector<std::string> labels{
        "eplansys.action=" + action_,
        "eplansys.agent=" + agent,
      };

      const auto request = RmfTaskClient::go_to_place(
        waypoint, spec_.orientation, labels);

      request_ids_.push_back(
        client_->submit_to_robot(binding->fleet, binding->robot, request));

      RCLCPP_INFO(
        get_logger(), "%s: %s -> %s/%s heading for %s",
        action_.c_str(), agent.c_str(), binding->fleet.c_str(),
        binding->robot.c_str(), waypoint.c_str());
    }

    if (request_ids_.size() > 1) {
      RCLCPP_INFO(
        get_logger(), "%s: %zu robots moving on one action",
        action_.c_str(), request_ids_.size());
    }
    return !request_ids_.empty();
  }

  void run_local()
  {
    const auto elapsed = (now() - begun_).seconds();
    if (elapsed < spec_.duration) {
      send_feedback(
        static_cast<float>(elapsed / spec_.duration), "speaking");
      return;
    }
    speak();
    conclude(true, "done", resolve_outcome(""));
  }

  /// The listener of a private announcement, from the argument the map names.
  std::string listener() const
  {
    const auto & args = get_arguments();
    const auto index = static_cast<std::size_t>(spec_.listener_arg);
    if (spec_.listener_arg < 0 || index >= args.size()) {
      return {};
    }
    return args[index];
  }

  /// Put the utterance on the channel the action declared.
  ///
  /// What it carries is the outcome the speaker's own sensing action reported,
  /// because that is the only form of the finding still available. eplansys's
  /// action map sends `relay-dirty` and `relay-clean` alike to `(relay ?i ?j)`,
  /// so a performer cannot tell from its own arguments which of the two is
  /// being said. The scan that produced the finding did report it, and the
  /// announcer kept it.
  void speak()
  {
    if (spec_.channel.empty() || !announcer_) {
      return;
    }

    const auto speaker = acting_agent();
    if (speaker.empty()) {
      RCLCPP_ERROR(get_logger(), "%s: no speaker argument", action_.c_str());
      return;
    }

    const auto content = announcer_->last_observed(speaker);
    if (content.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "%s: %s has sensed nothing, so the utterance carries no finding. The "
        "channel is still the one the policy chose.",
        action_.c_str(), speaker.c_str());
    }

    if (spec_.channel == "public") {
      announcer_->say_public(speaker, action_, content);
      return;
    }

    const auto heard_by = listener();
    if (heard_by.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "%s: private announcement with no listener at argument %d, so there "
        "is nobody to say it to and nothing is sent.",
        action_.c_str(), spec_.listener_arg);
      return;
    }
    announcer_->say_private(speaker, heard_by, action_, content);
  }

  void run_task()
  {
    const auto elapsed = (now() - begun_).seconds();

    // The action is over when every robot it moved is done, and it fails as
    // soon as any one of them does: half a joint move is not a state the model
    // has a world for.
    std::size_t finished = 0;
    std::string outcome;
    std::string last_status;

    for (const auto & request_id : request_ids_) {
      const auto status = client_->status(request_id);
      last_status = status.rmf_status;

      switch (status.phase) {
        case TaskPhase::Rejected:
          conclude(false, "RMF refused the task", "");
          return;

        case TaskPhase::Failed:
          conclude(false, status.error, "");
          return;

        case TaskPhase::Succeeded:
          ++finished;
          if (!status.outcome.empty()) {
            outcome = status.outcome;
          }
          break;

        case TaskPhase::Pending:
        case TaskPhase::Running:
        default:
          break;
      }
    }

    if (finished == request_ids_.size()) {
      conclude(true, "done", resolve_outcome(outcome));
      return;
    }

    if (timeout_ > 0.0 && elapsed > timeout_) {
      RCLCPP_ERROR(
        get_logger(), "%s: %zu of %zu RMF task(s) done after %.0fs, last %s",
        action_.c_str(), finished, request_ids_.size(), elapsed,
        last_status.empty() ? "unreported" : last_status.c_str());
      conclude(false, "RMF task timed out", "");
      return;
    }

    send_feedback(
      request_ids_.empty() ?
      0.5f :
      static_cast<float>(finished) / static_cast<float>(request_ids_.size()),
      last_status.empty() ? "submitted" : last_status);
  }

  /// An ordinary action reports nothing. A sensing action reports what the
  /// robot saw, and falls back to the configured answer when neither the fleet
  /// nor perception reported one, saying so, because a fleet adapter that
  /// knows nothing of ePlanSys writes no token and a simulated robot has
  /// nothing to sense with. Observations says why the order is what it is.
  std::string resolve_outcome(const std::string & carried)
  {
    if (!spec_.sensing) {
      return {};
    }

    const auto place = observed_place(spec_, get_arguments());
    const auto resolved = observations_.resolve(
      carried, place, begun_.nanoseconds(), mapping_->outcome_for(spec_, get_arguments()));

    if (!resolved.elsewhere.empty()) {
      std::string held;
      for (const auto & [where, what] : resolved.elsewhere) {
        held += (held.empty() ? "" : ", ") + where + "=" + what;
      }
      RCLCPP_WARN(
        get_logger(),
        "%s: nothing has been observed at %s. Readings are held for %s, and "
        "none of them is a reading of this action's site.",
        action_.c_str(), place.c_str(), held.c_str());
    }
    if (resolved.placeless_stale && resolved.source != OutcomeSource::Carried) {
      RCLCPP_WARN(
        get_logger(),
        "%s: the only placeless observation available was published %.1f s "
        "before this action began, so it is another action's reading and is "
        "ignored.",
        action_.c_str(), static_cast<double>(resolved.placeless_age_ns) / 1e9);
    }

    switch (resolved.source) {
      case OutcomeSource::Carried:
        RCLCPP_INFO(
          get_logger(), "%s: observed %s", action_.c_str(), resolved.outcome.c_str());
        break;
      case OutcomeSource::Place:
        RCLCPP_INFO(
          get_logger(), "%s: perception reported %s at %s",
          action_.c_str(), resolved.outcome.c_str(), place.c_str());
        break;
      case OutcomeSource::Placeless:
        RCLCPP_INFO(
          get_logger(), "%s: perception reported %s",
          action_.c_str(), resolved.outcome.c_str());
        break;
      case OutcomeSource::Map:
        RCLCPP_WARN(
          get_logger(),
          "%s: no outcome from RMF and none from perception, falling back to the "
          "map's \"%s\". A fleet adapter or a perception node reporting what it "
          "sensed would override this.",
          action_.c_str(), resolved.outcome.c_str());
        break;
      case OutcomeSource::Nothing:
        RCLCPP_ERROR(
          get_logger(),
          "%s: sensing action carried no outcome, and the map names none for "
          "these arguments. The policy has nothing to branch on.",
          action_.c_str());
        return {};
    }

    remember(resolved.outcome);
    return resolved.outcome;
  }

  /// Keep what the acting agent sensed, so that a later speech act by the same
  /// agent has a finding to carry.
  void remember(const std::string & outcome)
  {
    if (announcer_) {
      announcer_->observed(acting_agent(), outcome);
    }
  }

  void conclude(bool success, const std::string & status, const std::string & outcome)
  {
    finish(success, 1.0, status, outcome);
    started_ = false;
    request_ids_.clear();
  }

  std::string action_;
  ActionSpec spec_;
  std::shared_ptr<TaskMapping> mapping_;
  std::shared_ptr<RmfTaskClient> client_;
  std::shared_ptr<Announcer> announcer_;
  double timeout_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr observation_sub_;
  Observations observations_;

  bool started_{false};
  rclcpp::Time begun_;
  std::vector<std::string> request_ids_;
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
  config->declare_parameter("channel_prefix", std::string{"/eplansys/channel"});
  config->declare_parameter("observation_topic", std::string{"/eplansys/observation"});

  const auto task_map = config->get_parameter("task_map").as_string();
  const auto port = static_cast<int>(
    config->get_parameter("websocket_port").as_int());
  const auto prefix = config->get_parameter("outcome_prefix").as_string();
  const auto timeout = config->get_parameter("task_timeout").as_double();
  const auto channel_prefix = config->get_parameter("channel_prefix").as_string();
  const auto observation_topic =
    config->get_parameter("observation_topic").as_string();

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

  // Shared, like the task client: what the scout sensed is what the scout's
  // later relay says, and those are two performers.
  auto announcer = std::make_shared<eplansys_rmf_bridge::Announcer>(
    config, channel_prefix);

  // One process for every performer: they share the websocket the fleet
  // adapter dials, and a port can only be bound once.
  std::vector<std::shared_ptr<eplansys_rmf_bridge::RmfAction>> actions;
  for (const auto & name : mapping->action_names()) {
    const auto spec = mapping->action(name);
    actions.push_back(
      std::make_shared<eplansys_rmf_bridge::RmfAction>(
        name + "_rmf_node", name, *spec, mapping, client, announcer, timeout,
        observation_topic));

    std::string how = spec->local ? " (speech act, no RMF task)" : "";
    if (spec->channel == "public") {
      how = " (speech act, heard on " + announcer->public_topic() + ")";
    } else if (spec->channel == "private") {
      how = " (speech act, heard by its listener alone)";
    }
    RCLCPP_INFO(config->get_logger(), "performer for \"%s\"%s", name.c_str(), how.c_str());
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
