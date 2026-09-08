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

#ifndef EPLANSYS_RMF_BRIDGE__ANNOUNCER_HPP_
#define EPLANSYS_RMF_BRIDGE__ANNOUNCER_HPP_

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace eplansys_rmf_bridge
{

/// The channel a speech act goes out on.
///
/// A survey domain's difficulty is entirely in who is allowed to know what,
/// and the planner's whole job is to pick the channel whose audience does not
/// include the agent the goal excludes. Executing both channels as the same
/// two-second pause throws that away: the policy is checked against a model
/// that says the observer heard nothing, and nothing at all happens that the
/// observer could have failed to hear.
///
/// This puts an utterance on a topic whose subscribers are the audience the
/// action declared. A public announcement goes to one topic everyone listens
/// to; a private one goes to a topic named after its listener.
///
/// What that buys is addressing, not confidentiality. Any node on the graph
/// may subscribe to a private topic, so this establishes that the team used a
/// channel the observer was not on, and not that the observer could not have
/// listened. Enforcing the second needs DDS partitions or SROS 2 permissions,
/// which is a deployment concern and does not change anything here: the
/// audiences are already named, and that is what a partition would be built
/// from.
class Announcer
{
public:
  using Ptr = std::shared_ptr<Announcer>;

  /// @param node the node whose graph the topics live on
  /// @param prefix topic namespace; utterances go to `<prefix>/public` and
  ///        `<prefix>/private/<listener>`
  Announcer(rclcpp::Node::SharedPtr node, const std::string & prefix);

  /// Remember what an agent sensed, so a later speech act by that agent has
  /// something to say.
  ///
  /// The bridge has to remember because the utterance cannot carry the finding
  /// on its own. `relay-dirty` and `relay-clean` are different epistemic
  /// actions, and eplansys's action map sends both to the one PlanSys2 action
  /// `(relay ?i ?j)`: by the time a performer runs, which of the two is being
  /// said has been erased. What survives is the outcome the speaker's own
  /// sensing action reported, which is the same fact and is still here.
  void observed(const std::string & agent, const std::string & outcome);

  /// What @p agent last sensed, or empty if it has sensed nothing.
  std::string last_observed(const std::string & agent) const;

  /// Say something where everyone can hear it.
  void say_public(
    const std::string & speaker,
    const std::string & action,
    const std::string & content);

  /// Say something to one named listener.
  void say_private(
    const std::string & speaker,
    const std::string & listener,
    const std::string & action,
    const std::string & content);

  /// The topic a public announcement goes out on.
  std::string public_topic() const;

  /// The topic a private announcement to @p listener goes out on.
  std::string private_topic(const std::string & listener) const;

private:
  using Publisher = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;

  /// Publishers are made on first use and kept. A topic that no speech act
  /// ever reaches is never advertised, which keeps the graph a description of
  /// what the mission actually did.
  Publisher publisher_for(const std::string & topic);

  void say(
    const std::string & topic,
    const std::string & channel,
    const std::string & speaker,
    const std::string & listener,
    const std::string & action,
    const std::string & content);

  rclcpp::Node::SharedPtr node_;
  std::string prefix_;
  std::map<std::string, Publisher> publishers_;
  std::map<std::string, std::string> observed_;
};

}  // namespace eplansys_rmf_bridge

#endif  // EPLANSYS_RMF_BRIDGE__ANNOUNCER_HPP_
