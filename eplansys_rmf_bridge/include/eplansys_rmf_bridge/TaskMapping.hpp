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

#ifndef EPLANSYS_RMF_BRIDGE__TASKMAPPING_HPP_
#define EPLANSYS_RMF_BRIDGE__TASKMAPPING_HPP_

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace eplansys_rmf_bridge
{

/// Which robot an epistemic agent is, under pinned allocation.
///
/// The planner assigns actions to named agents while it solves, and the
/// epistemic model records knowledge against those names. Binding each name to
/// one robot is what keeps the model honest: the robot that senses is the agent
/// the model credits with knowing. Role-based allocation would let RMF choose
/// and relabel afterwards, which is a different and harder problem.
struct AgentBinding
{
  std::string fleet;
  std::string robot;
};

/// One robot's part of an action that moves more than one at a time.
///
/// A policy is a chain of product updates and the executor runs it strictly in
/// order, so two consecutive move actions are two robots moving one after the
/// other. Robots move together only when the togetherness is in the model: one
/// event, applied once, that relocates several agents. This says which of that
/// action's arguments name the agents and where each of them goes.
struct Movement
{
  /// Index of the argument naming the agent that moves.
  int agent_arg{0};
  /// Index of the argument naming the zone it moves to.
  int waypoint_arg{1};
};

/// How one PlanSys2 action becomes an RMF task.
struct ActionSpec
{
  /// A speech act. RMF has no representation of saying things, and a robot
  /// does not move to say them, so these complete without a task being
  /// submitted at all. The epistemic effect is entirely the planner's.
  bool local{false};

  /// Seconds a local action takes. Purely for the look of the thing.
  double duration{1.0};

  /// RMF task category. Only go_to_place is handled today.
  std::string category{"go_to_place"};

  /// Where the robot goes. An entry in `waypoints` for the acting agent wins
  /// over this one, which is what lets three agents scan three different
  /// places under one action name.
  std::string waypoint;

  /// Per agent overrides of `waypoint`.
  std::map<std::string, std::string> waypoints;

  /// Index into the action's arguments naming the zone to go to, for an
  /// action whose destination varies. `(goto_zone inspector lobby l3_suite)`
  /// takes 2. Negative means the fixed waypoint above, which is the case for
  /// an action that always sends a robot to the same place.
  int waypoint_arg{-1};

  /// The several robots this action moves at once, if it moves more than one.
  /// Empty for the ordinary case of an action that moves its acting agent
  /// alone, which `waypoint_arg` and `waypoint` then describe.
  std::vector<Movement> movements;

  std::optional<double> orientation;

  /// True when the point of the action is to find something out. Only a
  /// sensing action has an outcome to report, and only its finish() carries
  /// one.
  bool sensing{false};

  /// Index into the arguments whose value selects which outcome the robot
  /// reports, for a sensing action that can be performed in several places.
  /// `(look_into inspector l2_suite)` takes 1. Negative falls straight to
  /// `default_outcome`.
  int outcome_arg{-1};

  /// Argument value to outcome token, consulted when `outcome_arg` is set.
  ///
  /// This is the simulator's ground truth and nothing else: which suite is
  /// actually flooded. A robot with a real sensor reports what it measured and
  /// none of this is consulted. It exists so a demo on a fleet with no water
  /// sensor can still take both branches of its own policy.
  std::map<std::string, std::string> outcomes;

  /// What to report when the action is sensing and RMF carried no token.
  ///
  /// A fleet adapter that knows nothing of ePlanSys writes no outcome, and on
  /// a simulated fleet there is no sensor to write one from. Naming the answer
  /// here runs the mission end to end with RMF driving the robots, and the
  /// bridge says plainly in the log that it fell back. Empty means an absent
  /// token fails the action, which is what a real deployment wants.
  std::string default_outcome;
};

/// The action-to-task map, read from JSON.
///
/// This is a separate file from `eplansys`'s `action_mapping.json` and keyed
/// differently on purpose. That file maps plank's grounded names to PlanSys2
/// action expressions, and it does its work before the plan leaves the
/// planner. By the time an action reaches a performer the grounded name is
/// gone and what remains is a name and arguments, so a map keyed on grounded
/// names could not be consulted here. The two files answer different questions
/// at different times.
///
///   {
///     "zones": {
///       "lobby": "lobby",
///       "l2_suite": "L2_master_suite"
///     },
///     "agents": {
///       "scout": {"fleet": "tinyRobot", "robot": "tinyRobot1"},
///       "relay": {"fleet": "tinyRobot", "robot": "tinyRobot2"}
///     },
///     "actions": {
///       "goto_site": {"category": "go_to_place", "waypoint": "pantry"},
///       "scan":      {"sensing": true, "waypoint": "pantry",
///                     "default_outcome": "e-scan-dirty"},
///       "relay":     {"local": true, "duration": 2.0},
///       "broadcast": {"local": true, "duration": 2.0}
///     }
///   }
class TaskMapping
{
public:
  /// Throws std::runtime_error if the file cannot be read or is not shaped as
  /// documented above.
  static TaskMapping load(const std::string & path);

  /// Nullopt when the agent is not bound to a robot. An unbound agent is a
  /// hole in the map: submitting its task to an arbitrary robot would credit
  /// the wrong agent with knowing what was sensed, silently.
  std::optional<AgentBinding> agent(const std::string & name) const;

  /// Nullopt when the action has no entry.
  std::optional<ActionSpec> action(const std::string & name) const;

  /// The RMF waypoint `agent` goes to for `spec`, resolving in order: the
  /// argument named by `waypoint_arg`, a per agent override, then the fixed
  /// waypoint. A zone name is translated through the `zones` table; a name
  /// with no entry is passed through, so a map may name RMF waypoints
  /// directly.
  std::string waypoint_for(
    const ActionSpec & spec,
    const std::string & agent,
    const std::vector<std::string> & arguments) const;

  /// The outcome token the simulator says this action reveals, or empty when
  /// the map does not say and the fleet has to.
  std::string outcome_for(
    const ActionSpec & spec,
    const std::vector<std::string> & arguments) const;

  /// The RMF waypoint a zone name stands for; the name itself when unlisted.
  std::string waypoint_of_zone(const std::string & zone) const;

  /// Every action name in the map, which is what the process turns into
  /// performers.
  std::vector<std::string> action_names() const;

private:
  std::map<std::string, AgentBinding> agents_;
  std::map<std::string, ActionSpec> actions_;
  std::map<std::string, std::string> zones_;
};

}  // namespace eplansys_rmf_bridge

#endif  // EPLANSYS_RMF_BRIDGE__TASKMAPPING_HPP_
