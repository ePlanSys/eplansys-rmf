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

#ifndef EPLANSYS_RMF_BRIDGE__RMFTASKCLIENT_HPP_
#define EPLANSYS_RMF_BRIDGE__RMFTASKCLIENT_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>
#include <rmf_websocket/BroadcastServer.hpp>

namespace eplansys_rmf_bridge
{

/// One RMF task, from the bridge's point of view.
enum class TaskPhase
{
  /// Submitted, no ApiResponse yet.
  Pending,
  /// RMF refused it. The reason is in `error`.
  Rejected,
  /// Accepted and under way.
  Running,
  /// Reached `completed`.
  Succeeded,
  /// Reached failed, canceled or killed.
  Failed,
};

struct TaskStatus
{
  TaskPhase phase{TaskPhase::Pending};
  std::string task_id;
  std::string rmf_status;
  std::string robot;
  std::string outcome;
  std::string error;
};

/// Submits RMF tasks and watches what comes back.
///
/// Submission is a ROS 2 topic, `task_api_requests`, answered on
/// `task_api_responses`. Everything after that arrives over a websocket, and
/// only over a websocket: on Humble a fleet adapter publishes its task states
/// and logs to the URI given as its `server_uri` parameter and mirrors them
/// onto no ROS topic. So the bridge has to be the server the adapter dials,
/// which is what BroadcastServer provides.
///
/// The outcome is read out of that stream. RMF reports completion as a status
/// token and nothing else --- `task_state.json` has no result field and
/// `ActionExecution::finished()` takes no argument --- so a performer with
/// something to report has to write it into one of the two free-form fields
/// RMF does carry: an event's `detail` string, or a log entry. Both are
/// watched here, and a value carrying the outcome prefix is lifted out and
/// held against the task until the action asks for it.
class RmfTaskClient
{
public:
  /// \param node   used for the two ROS topics and for logging.
  /// \param port   websocket port the fleet adapter is told to dial.
  /// \param prefix marks a detail or log value as an epistemic outcome.
  RmfTaskClient(
    rclcpp::Node::SharedPtr node,
    int port,
    std::string prefix = "eplansys.outcome=");

  ~RmfTaskClient();

  /// Submit a task pinned to one robot. Returns the request id, which is what
  /// status() takes: the RMF task id is not known until the response arrives.
  std::string submit_to_robot(
    const std::string & fleet,
    const std::string & robot,
    const nlohmann::json & request);

  /// Submit a task and let RMF's dispatcher allocate. Provided for comparison;
  /// the bridge does not use it, because an agent that did not sense must not
  /// be credited with knowing.
  std::string submit_dispatched(const nlohmann::json & request);

  TaskStatus status(const std::string & request_id) const;

  /// Build the description of a go_to_place task.
  static nlohmann::json go_to_place(
    const std::string & waypoint,
    std::optional<double> orientation,
    int64_t start_millis,
    const std::vector<std::string> & labels);

private:
  std::string submit(const nlohmann::json & payload);
  void on_response(const rmf_task_msgs::msg::ApiResponse::SharedPtr msg);
  void on_websocket(const nlohmann::json & msg);
  void on_task_state(const nlohmann::json & state);
  void on_task_log(const nlohmann::json & log);
  void note_outcome(const std::string & task_id, const std::string & text);

  rclcpp::Node::SharedPtr node_;
  std::string prefix_;

  rclcpp::Publisher<rmf_task_msgs::msg::ApiRequest>::SharedPtr request_pub_;
  rclcpp::Subscription<rmf_task_msgs::msg::ApiResponse>::SharedPtr response_sub_;
  std::shared_ptr<rmf_websocket::BroadcastServer> server_;

  /// The websocket runs on its own thread, so everything below it is shared.
  mutable std::mutex mutex_;
  std::map<std::string, TaskStatus> by_request_;
  std::map<std::string, std::string> task_to_request_;
};

}  // namespace eplansys_rmf_bridge

#endif  // EPLANSYS_RMF_BRIDGE__RMFTASKCLIENT_HPP_
