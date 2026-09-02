/*
 * Copyright 2026 Haniel Ulises
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Submits one RMF task bound to a named robot.
 *
 * This is the pinned allocation path. A `robot_task_request` names a fleet and
 * a robot, and that robot's own TaskManager accepts it without a bid, so the
 * robot that runs the task is the one the planner chose. `--dispatch` sends a
 * `dispatch_task_request` instead, which goes to the dispatcher and lets RMF
 * allocate, for comparison.
 *
 * The booking id is printed on the way out. That is the key the state probe
 * groups by, and the key the bridge will use to match a finished RMF task to
 * the epistemic action that asked for it.
 */

#include <chrono>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>

namespace {

std::string make_request_id()
{
  static const char* digits = "0123456789abcdef";
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dist(0, 15);

  std::string id = "eplansys_probe_";
  for (int i = 0; i < 16; ++i)
    id.push_back(digits[dist(gen)]);
  return id;
}

struct Options
{
  std::string fleet = "tinyRobot";
  std::string robot = "tinyRobot1";
  std::string place = "patrol_A1";
  std::optional<double> orientation;
  bool dispatch = false;
  double timeout = 10.0;
  bool use_sim_time = false;
};

void usage()
{
  std::cout <<
    "usage: submit_probe [-F fleet] [-R robot] [-p waypoint] [-o degrees]\n"
    "                    [--dispatch] [--timeout S] [--use-sim-time]\n"
    "\n"
    "  -F, --fleet      fleet name, default tinyRobot\n"
    "  -R, --robot      robot name, default tinyRobot1\n"
    "  -p, --place      nav graph waypoint to go to, default patrol_A1\n"
    "  -o, --orient     final orientation in degrees\n"
    "      --dispatch   let RMF allocate instead of pinning, for comparison\n"
    "      --timeout    seconds to wait for the ApiResponse, default 10\n"
    "      --use-sim-time  required when the demo runs in simulation\n";
}

}  // namespace

int main(int argc, char** argv)
{
  Options opt;

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if ((arg == "-F" || arg == "--fleet") && i + 1 < argc)
      opt.fleet = argv[++i];
    else if ((arg == "-R" || arg == "--robot") && i + 1 < argc)
      opt.robot = argv[++i];
    else if ((arg == "-p" || arg == "--place") && i + 1 < argc)
      opt.place = argv[++i];
    else if ((arg == "-o" || arg == "--orient") && i + 1 < argc)
      opt.orientation = std::stod(argv[++i]);
    else if (arg == "--dispatch")
      opt.dispatch = true;
    else if (arg == "--timeout" && i + 1 < argc)
      opt.timeout = std::stod(argv[++i]);
    else if (arg == "--use-sim-time")
      opt.use_sim_time = true;
    else if (arg == "-h" || arg == "--help")
    {
      usage();
      return 0;
    }
    else
    {
      std::cerr << "unrecognised argument: " << arg << std::endl;
      usage();
      return 1;
    }
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("eplansys_rmf_submit_probe");

  if (opt.use_sim_time)
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  const auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
  auto pub = node->create_publisher<rmf_task_msgs::msg::ApiRequest>(
    "task_api_requests", transient_qos);

  const auto request_id = make_request_id();
  std::optional<nlohmann::json> response;

  auto sub = node->create_subscription<rmf_task_msgs::msg::ApiResponse>(
    "task_api_responses", 10,
    [&response, &request_id](
      const rmf_task_msgs::msg::ApiResponse::SharedPtr msg)
    {
      if (msg->request_id != request_id)
        return;
      response = nlohmann::json::parse(msg->json_msg, nullptr, false);
    });

  const auto now = node->get_clock()->now();
  const int64_t start_millis = now.nanoseconds() / 1000000;

  nlohmann::json go_to;
  go_to["waypoint"] = opt.place;
  if (opt.orientation.has_value())
    go_to["orientation"] = *opt.orientation * M_PI / 180.0;

  nlohmann::json activity;
  activity["category"] = "go_to_place";
  activity["description"] = go_to;

  nlohmann::json request;
  request["category"] = "compose";
  request["description"]["category"] = "go_to_place";
  request["description"]["phases"] = nlohmann::json::array(
    {nlohmann::json{{"activity", activity}}});
  request["unix_millis_earliest_start_time"] = start_millis;
  /* Labels come back untouched in booking.labels of every task state, which
   * is where the bridge will carry the ePlanSys action id. */
  request["labels"] = nlohmann::json::array({"eplansys.probe=" + request_id});
  request["requester"] = "eplansys_rmf_probe";

  nlohmann::json payload;
  if (opt.dispatch)
  {
    payload["type"] = "dispatch_task_request";
  }
  else
  {
    payload["type"] = "robot_task_request";
    payload["fleet"] = opt.fleet;
    payload["robot"] = opt.robot;
  }
  payload["request"] = request;

  const std::string target = opt.dispatch
    ? "whichever fleet wins the bid"
    : opt.fleet + "/" + opt.robot;

  std::cout << "submitting " << payload["type"].get<std::string>()
            << " to " << target
            << "\n  request_id " << request_id
            << "\n  waypoint   " << opt.place
            << "\n\n" << payload.dump(2) << "\n" << std::endl;

  rmf_task_msgs::msg::ApiRequest msg;
  msg.request_id = request_id;
  msg.json_msg = payload.dump();
  pub->publish(msg);

  /* Steady clock deliberately: with use_sim_time the node clock only advances
   * while the simulation runs, and a paused sim would hang here rather than
   * time out. */
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int64_t>(opt.timeout * 1000));

  while (rclcpp::ok() && !response.has_value() &&
    std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  int code = 0;
  if (!response.has_value())
  {
    std::cout << "no response within " << opt.timeout << "s.\n"
              << "the fleet adapter may be down, or the fleet name wrong."
              << std::endl;
    code = 1;
  }
  else if (response->is_discarded())
  {
    std::cout << "response was not valid JSON." << std::endl;
    code = 1;
  }
  else
  {
    std::cout << "response:\n" << response->dump(2) << "\n" << std::endl;

    if (!response->value("success", false))
    {
      std::cout << "rejected." << std::endl;
      for (const auto& e : response->value("errors", nlohmann::json::array()))
      {
        std::cout << "  " << e.value("code", 0) << ": "
                  << e.value("detail", std::string{"?"}) << std::endl;
      }
      code = 1;
    }
    else
    {
      const auto id = response->value("state", nlohmann::json::object())
        .value("booking", nlohmann::json::object())
        .value("id", std::string{"?"});
      std::cout << "accepted. task id: " << id
                << "\nthe state probe reports this task under that id."
                << std::endl;
    }
  }

  rclcpp::shutdown();
  return code;
}
