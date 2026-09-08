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

// One agent's ears.
//
// The bridge puts a speech act on a topic whose subscribers are the audience
// the action declared. That only means something if somebody is listening, so
// each agent of the mission runs one of these: it subscribes to the public
// channel and to the private channel addressed to it, and to nothing else.
//
// The point is the transcript. `observer` runs a radio like everyone else and
// is on no private channel, so when the team relays its finding the observer's
// transcript stays empty --- and that is a fact about what was published and
// received, rather than a fact about the model. The negative conjunct of the
// goal finally has a physical counterpart, and mission_check can be pointed at
// it.
//
// What this establishes is that the team used a channel the observer was not
// addressed on. It does not establish that the observer could not have
// listened: any node may subscribe to any topic, and shutting that off is a
// DDS partition or an SROS 2 permission, not a line of code here.

#include <memory>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

/// A channel keeps its utterances: several things may be said, and a listener
/// that comes up a moment late has still heard them.
rclcpp::QoS channel_qos()
{
  rclcpp::QoS qos(10);
  qos.reliable().transient_local();
  return qos;
}

/// A transcript keeps only its latest revision. Depth one is the point rather
/// than a saving: the transcript is republished whenever it grows, and a reader
/// that received the history would have to work out which copy was current.
/// With one slot retained, whatever arrives is the answer.
rclcpp::QoS transcript_qos()
{
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();
  return qos;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("radio");

  node->declare_parameter("agent", std::string{});
  node->declare_parameter("channel_prefix", std::string{"/eplansys/channel"});

  const auto agent = node->get_parameter("agent").as_string();
  auto prefix = node->get_parameter("channel_prefix").as_string();
  if (!prefix.empty() && prefix.back() == '/') {
    prefix.pop_back();
  }

  if (agent.empty()) {
    RCLCPP_FATAL(
      node->get_logger(),
      "no agent parameter. A radio is one agent's ears and cannot be nobody's.");
    rclcpp::shutdown();
    return 1;
  }

  auto transcript = node->create_publisher<std_msgs::msg::String>(
    prefix + "/heard/" + agent, transcript_qos());

  std::vector<std::string> heard;

  const auto record =
    [&](const std::string & channel, const std_msgs::msg::String::SharedPtr message) {
      std::string speaker;
      std::string content;

      const auto utterance = nlohmann::json::parse(message->data, nullptr, false);
      if (utterance.is_discarded()) {
        // Anything can publish on a topic. A frame that is not an utterance is
        // still something this agent heard, so it is recorded and not dropped.
        RCLCPP_WARN(
          node->get_logger(), "%s heard something unparseable on the %s channel",
          agent.c_str(), channel.c_str());
      } else {
        speaker = utterance.value("speaker", std::string{});
        content = utterance.value("content", std::string{});
      }

      RCLCPP_INFO(
        node->get_logger(), "%s heard %s say \"%s\" on the %s channel",
        agent.c_str(),
        speaker.empty() ? "someone" : speaker.c_str(),
        content.c_str(), channel.c_str());

      heard.push_back(message->data);

      nlohmann::json summary{
        {"agent", agent},
        {"heard", heard},
      };
      std_msgs::msg::String out;
      out.data = summary.dump();
      transcript->publish(out);
    };

  auto public_sub = node->create_subscription<std_msgs::msg::String>(
    prefix + "/public", channel_qos(),
    [&](const std_msgs::msg::String::SharedPtr message) {record("public", message);});

  auto private_sub = node->create_subscription<std_msgs::msg::String>(
    prefix + "/private/" + agent, channel_qos(),
    [&](const std_msgs::msg::String::SharedPtr message) {record("private", message);});

  // An empty transcript is a claim too, and the strongest one this mission
  // makes. Publishing it up front means "the observer heard nothing" is
  // something that was said rather than something that failed to arrive.
  nlohmann::json empty{{"agent", agent}, {"heard", std::vector<std::string>{}}};
  std_msgs::msg::String initial;
  initial.data = empty.dump();
  transcript->publish(initial);

  RCLCPP_INFO(
    node->get_logger(), "%s is listening on %s and %s",
    agent.c_str(), (prefix + "/public").c_str(),
    (prefix + "/private/" + agent).c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
