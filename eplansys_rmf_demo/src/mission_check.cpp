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

// Asks the epistemic state, once the robots have stopped, whether the mission
// actually came out the way it was supposed to.
//
// The survey's third conjunct is a negative one: the observer must not come to
// know whether the site is contaminated. A negative goal is satisfied by
// everything that does not happen, so a run that never touches the observer
// satisfies it for no reason at all, and looks exactly like a run that
// satisfies it because the planner chose the private channel. `observer` is
// bound to no robot in the office map and never moves, so that is precisely
// the situation the demo is in.
//
// This node closes that gap by asking. `epistemic_state/check_formula`
// evaluates a formula against the model as the executor left it, so the claim
// the domain is built to support can be stated as a check rather than as a
// remark in a README.
//
// It runs after the mission process exits and before the launch file shuts the
// system down, which is the only window in which the executor is finished and
// the state node is still alive.
//
// There are two kinds of claim here, and they are worth keeping apart. Asking
// the epistemic state settles where the model ended up, which is a statement
// about what the mission believes. Reading the agents' radio transcripts
// settles who was actually spoken to, which is a statement about what was
// published and received. The second is the weaker claim and the harder one to
// fake: an observer whose transcript is empty was on nobody's channel.

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

#include "plansys2_epistemic_executor/EpistemicStateClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // NOLINT (build/namespaces)

namespace
{

/// One formula, what it was expected to say, and what it said.
struct Check
{
  std::string formula;
  bool expected{true};
  bool answered{false};
  bool holds{false};
  std::string error;

  bool passed() const {return answered && holds == expected;}
};

/// Ask the state about one formula.
///
/// A call that is not answered is a failure and not a false answer. The
/// distinction is the whole point here: an epistemic state that is unreachable
/// would otherwise report that the observer knows nothing, which is the result
/// the demo is trying to establish, and it would report it about a system that
/// was never asked.
Check ask(
  const plansys2::EpistemicStateClient::Ptr & state,
  const std::string & formula,
  bool expected)
{
  Check check;
  check.formula = formula;
  check.expected = expected;

  const auto answer = state->check_formula(formula, 5s);
  if (!answer.answered) {
    check.error = answer.error.empty() ? "the epistemic state did not reply" : answer.error;
    return check;
  }
  if (!answer.success) {
    check.error = answer.error.empty() ? "the formula was refused" : answer.error;
    return check;
  }

  check.answered = true;
  check.holds = answer.holds;
  return check;
}

/// A formula nobody could be asked about.
Check unreachable(const std::string & formula, bool expected)
{
  Check check;
  check.formula = formula;
  check.expected = expected;
  check.error = "the epistemic state is not up";
  return check;
}

/// What one agent's radio recorded, or nothing if it never said.
///
/// A radio latches its transcript, so a subscriber that arrives after the
/// mission still receives it. A transcript that does not arrive at all is a
/// radio that was not running, which is not the same as an agent that heard
/// nothing and is not reported as one.
struct Transcript
{
  bool arrived{false};
  std::size_t utterances{0};
};

/// Collect the latched transcripts of the agents named.
std::map<std::string, Transcript> transcripts(
  const rclcpp::Node::SharedPtr & node,
  const std::string & prefix,
  const std::vector<std::string> & agents,
  const std::chrono::nanoseconds & patience)
{
  std::map<std::string, Transcript> out;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;

  // Depth one, matching the radios: a transcript is republished whenever it
  // grows, and only its latest revision is retained, so whatever arrives is
  // current by construction.
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();

  for (const auto & agent : agents) {
    out[agent] = Transcript{};
    subscriptions.push_back(
      node->create_subscription<std_msgs::msg::String>(
        prefix + "/heard/" + agent, qos,
        [&out, agent](const std_msgs::msg::String::SharedPtr message) {
          const auto parsed = nlohmann::json::parse(message->data, nullptr, false);
          if (parsed.is_discarded() || !parsed.contains("heard")) {
            return;
          }
          out[agent].arrived = true;
          out[agent].utterances = parsed.at("heard").size();
        }));
  }

  // A latched message arrives once the subscription has matched, which is not
  // instant. Spinning until every transcript is in, or until patience runs
  // out, is the difference between reading a transcript and reading a race.
  const auto deadline = node->now() + rclcpp::Duration(patience);
  while (rclcpp::ok() && node->now() < deadline) {
    rclcpp::spin_some(node);
    const auto missing = std::count_if(
      out.begin(), out.end(),
      [](const auto & entry) {return !entry.second.arrived;});
    if (missing == 0) {
      break;
    }
    std::this_thread::sleep_for(50ms);
  }

  return out;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mission_check");

  // The survey's three conjuncts, split by what each one claims. Parameters
  // rather than literals because the checks belong to the mission and not to
  // this node, and a different mission asks about different formulas.
  node->declare_parameter(
    "must_hold",
    std::vector<std::string>{
    "(Kw scout contaminated)",
    "(Kw relay contaminated)",
  });
  node->declare_parameter(
    "must_not_hold",
    std::vector<std::string>{
    "(Kw observer contaminated)",
  });

  // The other half of the mission's claim, and the physical one: who was
  // actually spoken to. `observer` is on no private channel, so its radio
  // should have recorded nothing at all.
  //
  // `scout` is the one spoken to, which is worth reading twice. The planner
  // sends the *relay* agent to the site and has it scan, and the speaker of a
  // private announcement is its first argument: the solution is
  // `relay-dirty_relay_scout`, so the agent named `relay` tells the agent
  // named `scout`. The names describe the roles the mission was written
  // around, not the roles the planner assigned.
  node->declare_parameter("heard_something", std::vector<std::string>{"scout"});
  node->declare_parameter("heard_nothing", std::vector<std::string>{"observer"});
  node->declare_parameter("channel_prefix", std::string{"/eplansys/channel"});

  const auto must_hold = node->get_parameter("must_hold").as_string_array();
  const auto must_not_hold = node->get_parameter("must_not_hold").as_string_array();
  const auto heard_something = node->get_parameter("heard_something").as_string_array();
  const auto heard_nothing = node->get_parameter("heard_nothing").as_string_array();
  auto prefix = node->get_parameter("channel_prefix").as_string();
  if (!prefix.empty() && prefix.back() == '/') {
    prefix.pop_back();
  }

  if (must_hold.empty() && must_not_hold.empty() &&
    heard_something.empty() && heard_nothing.empty())
  {
    RCLCPP_WARN(node->get_logger(), "nothing to check");
    rclcpp::shutdown();
    return 0;
  }

  auto state = std::make_shared<plansys2::EpistemicStateClient>("mission_check_state_client");

  // An unreachable state fails every formula as unchecked and stops nothing
  // else. The transcripts are read off the radios and have no more to do with
  // the epistemic state than the fleet does, so a state node that never came up
  // must not take the answer to "who was spoken to" down with it.
  const bool nothing_to_ask = must_hold.empty() && must_not_hold.empty();
  const bool state_up = nothing_to_ask || state->available(10s);

  if (!state_up) {
    RCLCPP_ERROR(
      node->get_logger(),
      "the epistemic state is not up, so nothing is concluded about the goal "
      "either way. What was said is still checked below.");
  }

  std::vector<Check> checks;
  for (const auto & formula : must_hold) {
    checks.push_back(state_up ? ask(state, formula, true) : unreachable(formula, true));
  }
  for (const auto & formula : must_not_hold) {
    checks.push_back(state_up ? ask(state, formula, false) : unreachable(formula, false));
  }

  std::size_t failed = 0;
  for (const auto & check : checks) {
    const auto claim = check.expected ? "holds" : "does not hold";

    if (!check.answered) {
      ++failed;
      RCLCPP_ERROR(
        node->get_logger(), "UNCHECKED %s %s: %s",
        check.formula.c_str(), claim, check.error.c_str());
      continue;
    }

    if (check.passed()) {
      RCLCPP_INFO(node->get_logger(), "ok   %s %s", check.formula.c_str(), claim);
      continue;
    }

    ++failed;
    RCLCPP_ERROR(
      node->get_logger(), "FAIL %s was to %s, and it %s",
      check.formula.c_str(), claim, check.holds ? "holds" : "does not hold");
  }

  // Who was actually spoken to. This is checked even when the formulas above
  // failed, because the two answer different questions and a mission that
  // ended in the wrong model may still have used the right channel.
  std::vector<std::string> listeners;
  listeners.insert(listeners.end(), heard_something.begin(), heard_something.end());
  listeners.insert(listeners.end(), heard_nothing.begin(), heard_nothing.end());

  std::size_t radios = 0;
  if (!listeners.empty()) {
    const auto recorded = transcripts(node, prefix, listeners, 5s);

    for (const auto & agent : heard_something) {
      ++radios;
      const auto & transcript = recorded.at(agent);
      if (!transcript.arrived) {
        ++failed;
        RCLCPP_ERROR(
          node->get_logger(),
          "UNCHECKED %s was to have been spoken to: no transcript, so its radio "
          "was not running", agent.c_str());
      } else if (transcript.utterances == 0) {
        ++failed;
        RCLCPP_ERROR(
          node->get_logger(), "FAIL %s was to have been spoken to, and heard nothing",
          agent.c_str());
      } else {
        RCLCPP_INFO(
          node->get_logger(), "ok   %s was spoken to, %zu time(s)",
          agent.c_str(), transcript.utterances);
      }
    }

    for (const auto & agent : heard_nothing) {
      ++radios;
      const auto & transcript = recorded.at(agent);
      if (!transcript.arrived) {
        ++failed;
        RCLCPP_ERROR(
          node->get_logger(),
          "UNCHECKED %s was to have been spoken to by nobody: no transcript, so "
          "its radio was not running and its silence proves nothing",
          agent.c_str());
      } else if (transcript.utterances > 0) {
        ++failed;
        RCLCPP_ERROR(
          node->get_logger(),
          "FAIL %s was to have been spoken to by nobody, and heard %zu utterance(s)",
          agent.c_str(), transcript.utterances);
      } else {
        RCLCPP_INFO(
          node->get_logger(), "ok   %s was spoken to by nobody", agent.c_str());
      }
    }
  }

  if (failed == 0) {
    RCLCPP_INFO(
      node->get_logger(),
      "the mission came out as specified: %zu formulas against the state the "
      "fleet left behind, and %zu transcript(s) of who was spoken to.",
      checks.size(), radios);
  } else {
    RCLCPP_ERROR(
      node->get_logger(), "%zu of %zu checks did not pass",
      failed, checks.size() + radios);
  }

  rclcpp::shutdown();
  return failed == 0 ? 0 : 1;
}
