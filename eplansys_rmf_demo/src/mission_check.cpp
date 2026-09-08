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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "plansys2_epistemic_executor/EpistemicStateClient.hpp"

#include "rclcpp/rclcpp.hpp"

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

  const auto must_hold = node->get_parameter("must_hold").as_string_array();
  const auto must_not_hold = node->get_parameter("must_not_hold").as_string_array();

  if (must_hold.empty() && must_not_hold.empty()) {
    RCLCPP_WARN(node->get_logger(), "no formulas to check");
    rclcpp::shutdown();
    return 0;
  }

  auto state = std::make_shared<plansys2::EpistemicStateClient>("mission_check_state_client");

  if (!state->available(10s)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "the epistemic state is not up, so the mission cannot be checked. Nothing "
      "is concluded about the goal either way.");
    rclcpp::shutdown();
    return 1;
  }

  std::vector<Check> checks;
  for (const auto & formula : must_hold) {
    checks.push_back(ask(state, formula, true));
  }
  for (const auto & formula : must_not_hold) {
    checks.push_back(ask(state, formula, false));
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

  if (failed == 0) {
    RCLCPP_INFO(
      node->get_logger(),
      "the mission came out as specified: %zu formulas checked against the "
      "state the fleet left behind.", checks.size());
  } else {
    RCLCPP_ERROR(
      node->get_logger(), "%zu of %zu checks did not pass", failed, checks.size());
  }

  rclcpp::shutdown();
  return failed == 0 ? 0 : 1;
}
