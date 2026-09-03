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

#include "eplansys_rmf_bridge/RmfTaskClient.hpp"

#include <chrono>
#include <random>
#include <utility>
#include <vector>

namespace eplansys_rmf_bridge
{

namespace
{

std::string make_request_id()
{
  static const char * digits = "0123456789abcdef";
  static std::mt19937 gen{std::random_device{}()};
  std::uniform_int_distribution<> dist(0, 15);

  std::string id = "eplansys_";
  for (int i = 0; i < 16; ++i) {
    id.push_back(digits[dist(gen)]);
  }
  return id;
}

bool is_terminal(const std::string & status)
{
  return status == "completed" || status == "failed" ||
         status == "canceled" || status == "killed";
}

}  // namespace

RmfTaskClient::RmfTaskClient(
  rclcpp::Node::SharedPtr node, int port, std::string prefix)
: node_(std::move(node)), prefix_(std::move(prefix))
{
  const auto qos = rclcpp::QoS(10).reliable().transient_local();
  request_pub_ = node_->create_publisher<rmf_task_msgs::msg::ApiRequest>(
    "task_api_requests", qos);

  response_sub_ = node_->create_subscription<rmf_task_msgs::msg::ApiResponse>(
    "task_api_responses", 10,
    [this](const rmf_task_msgs::msg::ApiResponse::SharedPtr msg) {
      on_response(msg);
    });

  feed_ = std::make_unique<WebsocketFeed>(
    port, [this](const nlohmann::json & msg) {on_websocket(msg);});
  feed_->start();

  fleet_sub_ = node_->create_subscription<rmf_fleet_msgs::msg::FleetState>(
    "fleet_states", 10,
    [this](const rmf_fleet_msgs::msg::FleetState::SharedPtr msg) {
      on_fleet_state(msg);
    });

  flush_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(250), [this]() {flush_unpublished();});

  RCLCPP_INFO(
    node_->get_logger(),
    "listening for task states on ws://localhost:%d; the fleet adapter needs "
    "server_uri:=\"ws://localhost:%d\"", port, port);
}

RmfTaskClient::~RmfTaskClient()
{
  if (feed_) {
    feed_->stop();
  }
}

nlohmann::json RmfTaskClient::go_to_place(
  const std::string & waypoint,
  std::optional<double> orientation,
  const std::vector<std::string> & labels)
{
  nlohmann::json description;
  description["waypoint"] = waypoint;
  if (orientation.has_value()) {
    description["orientation"] = *orientation;
  }

  nlohmann::json activity;
  activity["category"] = "go_to_place";
  activity["description"] = description;

  nlohmann::json request;
  request["category"] = "compose";
  request["description"]["category"] = "go_to_place";
  request["description"]["phases"] =
    nlohmann::json::array({nlohmann::json{{"activity", activity}}});
  /* No unix_millis_earliest_start_time. It is optional, and the bridge has no
   * business asserting one: a simulated fleet runs on /clock, where now is a
   * few seconds past zero, while this node runs on the wall clock. Stamping a
   * wall-clock time onto a task the fleet reads against sim time puts its
   * earliest start about fifty thousand years in the future, and the task sits
   * there while the robot idles. Leaving it out means as soon as possible,
   * which is what every action here wants. */
  request["labels"] = labels;
  request["requester"] = "eplansys_rmf_bridge";
  return request;
}

std::string RmfTaskClient::submit_to_robot(
  const std::string & fleet,
  const std::string & robot,
  const nlohmann::json & request)
{
  nlohmann::json payload;
  payload["type"] = "robot_task_request";
  payload["fleet"] = fleet;
  payload["robot"] = robot;
  payload["request"] = request;
  return submit(payload, fleet, robot);
}

std::string RmfTaskClient::submit_dispatched(const nlohmann::json & request)
{
  nlohmann::json payload;
  payload["type"] = "dispatch_task_request";
  payload["request"] = request;
  return submit(payload, {}, {});
}

std::string RmfTaskClient::submit(
  const nlohmann::json & payload,
  const std::string & fleet,
  const std::string & robot)
{
  const auto request_id = make_request_id();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    by_request_[request_id] = TaskStatus{};
    unpublished_[request_id] = Unpublished{payload, fleet, robot};
  }

  if (!try_publish(request_id)) {
    RCLCPP_INFO(
      node_->get_logger(),
      "%s/%s has not announced itself yet; holding the request until it does",
      fleet.c_str(), robot.c_str());
  }

  return request_id;
}

bool RmfTaskClient::try_publish(const std::string & request_id)
{
  Unpublished pending;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = unpublished_.find(request_id);
    if (it == unpublished_.end()) {
      return true;
    }
    pending = it->second;

    if (pending.robot.empty()) {
      // A dispatched request goes to the dispatcher, so a subscriber is all
      // that can be checked for.
      if (request_pub_->get_subscription_count() == 0) {
        return false;
      }
    } else if (
      known_robots_.count({pending.fleet, pending.robot}) == 0)
    {
      return false;
    }

    unpublished_.erase(it);
  }

  rmf_task_msgs::msg::ApiRequest msg;
  msg.request_id = request_id;
  msg.json_msg = pending.payload.dump();
  request_pub_->publish(msg);
  return true;
}

void RmfTaskClient::flush_unpublished()
{
  // Once a second, say what the websocket is actually carrying. Silence here
  // is the difference between a fleet that is not connected and one whose
  // frames are not being understood, and the two look identical otherwise.
  if (++flush_ticks_ % 40 == 0) {
    RCLCPP_INFO(
      node_->get_logger(),
      "websocket: %zu connection(s), %zu frame(s) in, %zu task state(s) seen",
      feed_->connections(), feed_->received(), task_to_request_.size());
  }

  std::vector<std::string> waiting;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (unpublished_.empty()) {
      return;
    }
    waiting.reserve(unpublished_.size());
    for (const auto & [request_id, pending] : unpublished_) {
      (void)pending;
      waiting.push_back(request_id);
    }
  }

  for (const auto & request_id : waiting) {
    if (try_publish(request_id)) {
      RCLCPP_INFO(node_->get_logger(), "robot is up; request sent");
    }
  }
}

void RmfTaskClient::on_fleet_state(
  const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & robot : msg->robots) {
    known_robots_.emplace(msg->name, robot.name);
  }
}

TaskStatus RmfTaskClient::status(const std::string & request_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = by_request_.find(request_id);
  if (it == by_request_.end()) {
    return TaskStatus{};
  }
  return it->second;
}

void RmfTaskClient::on_response(
  const rmf_task_msgs::msg::ApiResponse::SharedPtr msg)
{
  const auto doc = nlohmann::json::parse(msg->json_msg, nullptr, false);
  if (doc.is_discarded()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = by_request_.find(msg->request_id);
  if (it == by_request_.end()) {
    return;
  }
  auto & status = it->second;

  if (!doc.value("success", false)) {
    status.phase = TaskPhase::Rejected;
    status.error = doc.contains("errors") ? doc.at("errors").dump() : "refused";
    RCLCPP_ERROR(
      node_->get_logger(), "RMF refused the task: %s", status.error.c_str());
    return;
  }

  const auto booking = doc.value("state", nlohmann::json::object())
    .value("booking", nlohmann::json::object());
  status.task_id = booking.value("id", std::string{});
  status.phase = TaskPhase::Running;

  if (!status.task_id.empty()) {
    task_to_request_[status.task_id] = msg->request_id;
  }

  RCLCPP_INFO(
    node_->get_logger(), "RMF accepted the task as %s", status.task_id.c_str());
}

void RmfTaskClient::on_websocket(const nlohmann::json & msg)
{
  const auto type = msg.value("type", std::string{});
  if (!msg.contains("data")) {
    return;
  }

  if (type == "task_state_update") {
    on_task_state(msg.at("data"));
  } else if (type == "task_log_update") {
    on_task_log(msg.at("data"));
  }
}

void RmfTaskClient::on_task_state(const nlohmann::json & state)
{
  const auto task_id = state.value("booking", nlohmann::json::object())
    .value("id", std::string{});
  if (task_id.empty()) {
    return;
  }

  // Event detail before the status, so an outcome written on the same update
  // that completes the task is recorded before the action reads it.
  if (state.contains("phases")) {
    for (const auto & phase : state.at("phases")) {
      if (!phase.contains("events")) {
        continue;
      }
      for (const auto & event : phase.at("events")) {
        if (event.contains("detail") && event.at("detail").is_string()) {
          note_outcome(task_id, event.at("detail").get<std::string>());
        }
      }
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const auto link = task_to_request_.find(task_id);
  if (link == task_to_request_.end()) {
    return;
  }
  auto & status = by_request_.at(link->second);

  if (state.contains("assigned_to")) {
    const auto & a = state.at("assigned_to");
    status.robot = a.value("group", "") + "/" + a.value("name", "");
  }

  const auto rmf_status = state.value("status", std::string{});
  if (rmf_status.empty() || rmf_status == status.rmf_status) {
    return;
  }
  status.rmf_status = rmf_status;

  if (!is_terminal(rmf_status)) {
    return;
  }

  status.phase = (rmf_status == "completed") ?
    TaskPhase::Succeeded : TaskPhase::Failed;
  if (status.phase == TaskPhase::Failed) {
    status.error = "RMF reported " + rmf_status;
  }
}

void RmfTaskClient::on_task_log(const nlohmann::json & log)
{
  const auto task_id = log.value("task_id", std::string{});
  if (task_id.empty() || !log.contains("phases")) {
    return;
  }

  for (const auto & [pid, phase] : log.at("phases").items()) {
    (void)pid;
    if (!phase.contains("events")) {
      continue;
    }
    for (const auto & [eid, entries] : phase.at("events").items()) {
      (void)eid;
      for (const auto & entry : entries) {
        note_outcome(task_id, entry.value("text", std::string{}));
      }
    }
  }
}

void RmfTaskClient::note_outcome(
  const std::string & task_id, const std::string & text)
{
  const auto pos = text.find(prefix_);
  if (pos == std::string::npos) {
    return;
  }

  auto token = text.substr(pos + prefix_.size());
  const auto end = token.find_last_not_of(" \t\r\n");
  token = (end == std::string::npos) ? std::string{} : token.substr(0, end + 1);
  if (token.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const auto link = task_to_request_.find(task_id);
  if (link == task_to_request_.end()) {
    return;
  }

  auto & status = by_request_.at(link->second);
  if (status.outcome == token) {
    return;
  }
  status.outcome = token;

  RCLCPP_INFO(
    node_->get_logger(), "task %s carried outcome \"%s\"",
    task_id.c_str(), token.c_str());
}

}  // namespace eplansys_rmf_bridge
