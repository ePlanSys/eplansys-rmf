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

#include <random>
#include <utility>

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

  // nullopt keeps the envelope, so one server can tell task states from task
  // logs. Naming a selection would deliver msg["data"] alone.
  server_ = rmf_websocket::BroadcastServer::make(
    port,
    [this](const nlohmann::json & msg) {on_websocket(msg);},
    std::nullopt);
  server_->start();

  RCLCPP_INFO(
    node_->get_logger(),
    "listening for task states on ws://localhost:%d; the fleet adapter needs "
    "server_uri:=\"ws://localhost:%d\"", port, port);
}

RmfTaskClient::~RmfTaskClient()
{
  if (server_) {
    server_->stop();
  }
}

nlohmann::json RmfTaskClient::go_to_place(
  const std::string & waypoint,
  std::optional<double> orientation,
  int64_t start_millis,
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
  request["unix_millis_earliest_start_time"] = start_millis;
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
  return submit(payload);
}

std::string RmfTaskClient::submit_dispatched(const nlohmann::json & request)
{
  nlohmann::json payload;
  payload["type"] = "dispatch_task_request";
  payload["request"] = request;
  return submit(payload);
}

std::string RmfTaskClient::submit(const nlohmann::json & payload)
{
  const auto request_id = make_request_id();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    by_request_[request_id] = TaskStatus{};
  }

  rmf_task_msgs::msg::ApiRequest msg;
  msg.request_id = request_id;
  msg.json_msg = payload.dump();
  request_pub_->publish(msg);

  return request_id;
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

  status.phase = (rmf_status == "completed")
    ? TaskPhase::Succeeded : TaskPhase::Failed;
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
