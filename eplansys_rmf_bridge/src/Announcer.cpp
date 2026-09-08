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

#include "eplansys_rmf_bridge/Announcer.hpp"

#include <memory>
#include <string>
#include <utility>

#include "nlohmann/json.hpp"

namespace eplansys_rmf_bridge
{

Announcer::Announcer(rclcpp::Node::SharedPtr node, const std::string & prefix)
: node_(std::move(node)),
  prefix_(prefix.empty() ? std::string{"/eplansys/channel"} : prefix)
{
  if (prefix_.back() == '/') {
    prefix_.pop_back();
  }
}

void Announcer::observed(const std::string & agent, const std::string & outcome)
{
  if (agent.empty() || outcome.empty()) {
    return;
  }
  observed_[agent] = outcome;
}

std::string Announcer::last_observed(const std::string & agent) const
{
  const auto it = observed_.find(agent);
  return it == observed_.end() ? std::string{} : it->second;
}

std::string Announcer::public_topic() const
{
  return prefix_ + "/public";
}

std::string Announcer::private_topic(const std::string & listener) const
{
  return prefix_ + "/private/" + listener;
}

Announcer::Publisher Announcer::publisher_for(const std::string & topic)
{
  const auto it = publishers_.find(topic);
  if (it != publishers_.end()) {
    return it->second;
  }

  // Transient local, because a listener that comes up a moment late has still
  // heard it. A speech act happens once and is not repeated, and a mission
  // that turned on whether a subscriber was ready would be testing start up
  // order rather than the policy.
  rclcpp::QoS qos(10);
  qos.reliable().transient_local();

  auto publisher = node_->create_publisher<std_msgs::msg::String>(topic, qos);
  publishers_.emplace(topic, publisher);
  return publisher;
}

void Announcer::say(
  const std::string & topic,
  const std::string & channel,
  const std::string & speaker,
  const std::string & listener,
  const std::string & action,
  const std::string & content)
{
  nlohmann::json utterance{
    {"channel", channel},
    {"speaker", speaker},
    {"action", action},
    {"content", content},
  };
  if (!listener.empty()) {
    utterance["listener"] = listener;
  }

  std_msgs::msg::String message;
  message.data = utterance.dump();
  publisher_for(topic)->publish(message);

  RCLCPP_INFO(
    node_->get_logger(), "%s says %s on %s",
    speaker.c_str(),
    content.empty() ? "nothing in particular" : content.c_str(),
    topic.c_str());
}

void Announcer::say_public(
  const std::string & speaker,
  const std::string & action,
  const std::string & content)
{
  say(public_topic(), "public", speaker, "", action, content);
}

void Announcer::say_private(
  const std::string & speaker,
  const std::string & listener,
  const std::string & action,
  const std::string & content)
{
  say(private_topic(listener), "private", speaker, listener, action, content);
}

}  // namespace eplansys_rmf_bridge
