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
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>
#include "eplansys_rmf_bridge/WebsocketFeed.hpp"

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
/// which is what WebsocketFeed is.
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
  ///
  /// The publish may be deferred. A performer can be dispatched its first
  /// action within a second of the process starting, well before the fleet
  /// adapter is up, and a robot_task_request published then is simply lost:
  /// only the named robot's own TaskManager handles one, the dispatcher
  /// ignores it, and no error is raised anywhere. So the request waits until
  /// that robot has announced itself on fleet_states, which is the earliest
  /// point at which its adapter is known to be running and discovered.
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
    const std::vector<std::string> & labels);

private:
  std::string submit(
    const nlohmann::json & payload,
    const std::string & fleet,
    const std::string & robot);

  /// Publishes if the request's target is ready. Returns false if it is not,
  /// leaving the request queued for the timer.
  bool try_publish(const std::string & request_id);

  /// Publishes whatever has been waiting for its robot to appear.
  void flush_unpublished();

  void on_fleet_state(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg);

  void on_response(const rmf_task_msgs::msg::ApiResponse::SharedPtr msg);
  void on_websocket(const nlohmann::json & msg);
  void on_task_state(const nlohmann::json & state);
  void on_task_log(const nlohmann::json & log);
  void note_outcome(const std::string & task_id, const std::string & text);

  rclcpp::Node::SharedPtr node_;
  std::string prefix_;

  rclcpp::Publisher<rmf_task_msgs::msg::ApiRequest>::SharedPtr request_pub_;
  rclcpp::Subscription<rmf_task_msgs::msg::ApiResponse>::SharedPtr response_sub_;
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_sub_;
  std::unique_ptr<WebsocketFeed> feed_;

  /// The websocket runs on its own thread, so everything below it is shared.
  mutable std::mutex mutex_;
  std::map<std::string, TaskStatus> by_request_;
  std::map<std::string, std::string> task_to_request_;

  /// A request built but not yet on the wire.
  struct Unpublished
  {
    nlohmann::json payload;
    /// Empty for a dispatched request, which any fleet may answer.
    std::string fleet;
    std::string robot;
  };

  std::map<std::string, Unpublished> unpublished_;
  std::set<std::pair<std::string, std::string>> known_robots_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
  std::size_t flush_ticks_{0};
};

}  // namespace eplansys_rmf_bridge

#endif  // EPLANSYS_RMF_BRIDGE__RMFTASKCLIENT_HPP_
