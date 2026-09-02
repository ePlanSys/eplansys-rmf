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
 * Stands in for the RMF api server and reports what a task carries back.
 *
 * A fleet adapter publishes its task states and task logs to whatever
 * websocket it is given as `server_uri`, and on Humble to nowhere else: the
 * ROS 2 mirror of those topics exists only on rolling. This listens on that
 * socket and prints, per task, the status transitions, the event `detail`
 * strings, and the log entries.
 *
 * The question it exists to answer is whether a token written by a performer
 * survives the trip. `RobotUpdateHandle::ActionExecution::finished()` takes no
 * argument and `task_state.json` has no result field, so a performer that
 * wants to report an observation has to write it into one of the two
 * free-form fields RMF does carry. Both are reported here, and any value
 * carrying the outcome prefix is pulled out and shown against the task that
 * produced it.
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <thread>
#include <string>
#include <tuple>
#include <vector>

#include <nlohmann/json.hpp>
#include <rmf_websocket/BroadcastServer.hpp>

namespace {

std::atomic_bool g_stop{false};

void handle_signal(int)
{
  g_stop = true;
}

std::string now_stamp()
{
  const auto now = std::chrono::system_clock::now();
  const auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&t, &tm);
  std::ostringstream ss;
  ss << std::put_time(&tm, "%H:%M:%S");
  return ss.str();
}

std::string millis_stamp(int64_t unix_millis)
{
  const std::time_t t = static_cast<std::time_t>(unix_millis / 1000);
  std::tm tm{};
  localtime_r(&t, &tm);
  std::ostringstream ss;
  ss << std::put_time(&tm, "%H:%M:%S");
  return ss.str();
}

bool is_terminal(const std::string& status)
{
  return status == "completed" || status == "failed" ||
    status == "canceled" || status == "killed" || status == "skipped";
}

struct Task
{
  std::string id;
  std::string status;
  std::string category;
  std::string robot;
  std::map<uint64_t, std::string> details;
  std::set<std::tuple<std::string, std::string, uint64_t>> log_seqs;
  std::vector<std::pair<std::string, std::string>> outcomes;
};

class Probe
{
public:
  Probe(std::string outcome_prefix, bool all_logs, bool raw)
  : _outcome_prefix(std::move(outcome_prefix)),
    _all_logs(all_logs),
    _raw(raw)
  {
  }

  void receive(const nlohmann::json& msg)
  {
    if (_raw)
      std::cout << msg.dump(2) << std::endl;

    const auto type_it = msg.find("type");
    if (type_it == msg.end() || !msg.contains("data"))
      return;

    const std::string type = type_it->get<std::string>();
    if (type == "task_state_update")
      on_task_state(msg.at("data"));
    else if (type == "task_log_update")
      on_task_log(msg.at("data"));
  }

  std::size_t task_count() const
  {
    return _tasks.size();
  }

private:
  Task& task(const std::string& id)
  {
    auto it = _tasks.find(id);
    if (it == _tasks.end())
    {
      Task t;
      t.id = id;
      it = _tasks.emplace(id, std::move(t)).first;
      std::cout << "[" << now_stamp() << "] task " << id << ": first seen"
                << std::endl;
    }
    return it->second;
  }

  void on_task_state(const nlohmann::json& state)
  {
    if (!state.contains("booking") || !state.at("booking").contains("id"))
      return;

    const auto id = state.at("booking").at("id").get<std::string>();
    auto& t = task(id);

    if (state.contains("category"))
      t.category = state.at("category").get<std::string>();

    if (state.contains("assigned_to"))
    {
      const auto& a = state.at("assigned_to");
      std::string robot = a.value("group", "") + "/" + a.value("name", "");
      if (robot != t.robot)
      {
        t.robot = robot;
        std::cout << "[" << now_stamp() << "] task " << id
                  << ": assigned to " << robot << std::endl;
      }
    }

    if (state.contains("status"))
    {
      const auto status = state.at("status").get<std::string>();
      if (status != t.status)
      {
        std::cout << "[" << now_stamp() << "] task " << id << ": "
                  << (t.status.empty() ? "none" : t.status) << " -> "
                  << status << std::endl;
        t.status = status;
      }
    }

    /* Event detail is the tidier of the two carriers: a string the event
     * implementation sets through SimpleEventState::update_detail, forwarded
     * verbatim into every state update. */
    if (state.contains("phases"))
    {
      for (const auto& phase : state.at("phases"))
      {
        if (!phase.contains("events"))
          continue;

        for (const auto& event : phase.at("events"))
        {
          if (!event.contains("detail") || !event.at("detail").is_string())
            continue;

          const auto detail = event.at("detail").get<std::string>();
          const auto eid = event.value("id", uint64_t{0});
          if (t.details.count(eid) && t.details.at(eid) == detail)
            continue;

          t.details[eid] = detail;
          std::cout << "[" << now_stamp() << "] task " << id << ": event "
                    << eid << " [" << event.value("name", "") << "] detail=\""
                    << detail << "\"" << std::endl;
          check_outcome(t, detail, "detail");
        }
      }
    }

    if (is_terminal(t.status))
      report(t);
  }

  void on_task_log(const nlohmann::json& log)
  {
    if (!log.contains("task_id"))
      return;

    auto& t = task(log.at("task_id").get<std::string>());

    if (log.contains("log"))
    {
      for (const auto& entry : log.at("log"))
        on_log_entry(t, entry, "", "");
    }

    if (!log.contains("phases"))
      return;

    for (const auto& [pid, phase] : log.at("phases").items())
    {
      if (phase.contains("log"))
      {
        for (const auto& entry : phase.at("log"))
          on_log_entry(t, entry, pid, "");
      }

      if (!phase.contains("events"))
        continue;

      for (const auto& [eid, entries] : phase.at("events").items())
      {
        for (const auto& entry : entries)
          on_log_entry(t, entry, pid, eid);
      }
    }
  }

  void on_log_entry(
    Task& t,
    const nlohmann::json& entry,
    const std::string& phase,
    const std::string& event)
  {
    const auto seq = entry.value("seq", uint64_t{0});
    const auto key = std::make_tuple(phase, event, seq);
    if (!t.log_seqs.insert(key).second)
      return;

    const auto text = entry.value("text", std::string{});
    const auto tier = entry.value("tier", std::string{"info"});

    const bool matched = check_outcome(t, text, "log");
    if (!matched && !_all_logs && tier != "warning" && tier != "error")
      return;

    const auto where = event.empty()
      ? std::string{"task"}
      : "phase " + phase + " event " + event;

    std::cout << "[" << millis_stamp(entry.value("unix_millis_time", int64_t{0}))
              << "] task " << t.id << ": " << where << " [" << tier << "] "
              << text << std::endl;
  }

  bool check_outcome(Task& t, const std::string& text, const char* carrier)
  {
    const auto pos = text.find(_outcome_prefix);
    if (pos == std::string::npos)
      return false;

    auto token = text.substr(pos + _outcome_prefix.size());
    const auto end = token.find_last_not_of(" \t\r\n");
    token = (end == std::string::npos) ? std::string{} : token.substr(0, end + 1);

    t.outcomes.emplace_back(carrier, token);
    std::cout << "[" << now_stamp() << "] task " << t.id << ": OUTCOME via "
              << carrier << ": \"" << token << "\"" << std::endl;
    return true;
  }

  void report(const Task& t) const
  {
    std::cout << "\n  task     " << t.id
              << "\n  category " << t.category;
    if (!t.robot.empty())
      std::cout << "\n  robot    " << t.robot;
    std::cout << "\n  status   " << t.status;

    if (t.outcomes.empty())
    {
      std::cout << "\n  outcome  none, no value carried \""
                << _outcome_prefix << "\"";
    }
    else
    {
      for (const auto& [carrier, token] : t.outcomes)
        std::cout << "\n  outcome  \"" << token << "\" (via " << carrier << ")";
    }
    std::cout << "\n" << std::endl;
  }

  std::string _outcome_prefix;
  bool _all_logs;
  bool _raw;
  std::map<std::string, Task> _tasks;
};

}  // namespace

int main(int argc, char** argv)
{
  int port = 7879;
  std::string outcome_prefix = "eplansys.outcome=";
  bool all_logs = false;
  bool raw = false;

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--port" && i + 1 < argc)
      port = std::stoi(argv[++i]);
    else if (arg == "--outcome-prefix" && i + 1 < argc)
      outcome_prefix = argv[++i];
    else if (arg == "--all-logs")
      all_logs = true;
    else if (arg == "--raw")
      raw = true;
    else if (arg == "-h" || arg == "--help")
    {
      std::cout <<
        "usage: state_probe [--port N] [--outcome-prefix S] [--all-logs] [--raw]\n"
        "\n"
        "  --port            websocket port to listen on, default 7879.\n"
        "                    7878 is the rmf_demos panel, so leave it alone.\n"
        "  --outcome-prefix  a detail or log value carrying this prefix is\n"
        "                    read as an epistemic outcome token.\n"
        "  --all-logs        print every log entry, not only warnings,\n"
        "                    errors and outcomes.\n"
        "  --raw             dump every frame as received.\n";
      return 0;
    }
    else
    {
      std::cerr << "unrecognised argument: " << arg << std::endl;
      return 1;
    }
  }

  Probe probe{outcome_prefix, all_logs, raw};

  /* nullopt asks for every message with its envelope intact. Passing a
   * selection would hand the callback msg["data"] alone, which would leave
   * task states and task logs indistinguishable. */
  const auto server = rmf_websocket::BroadcastServer::make(
    port,
    [&probe](const nlohmann::json& msg) { probe.receive(msg); },
    std::nullopt);

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  server->start();
  std::cout << "[" << now_stamp() << "] listening on ws://localhost:" << port
            << "\n[" << now_stamp() << "] launch the fleet adapter with "
            << "server_uri:=\"ws://localhost:" << port << "\"" << std::endl;

  while (!g_stop)
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

  server->stop();
  std::cout << "\n[" << now_stamp() << "] saw " << probe.task_count()
            << " task(s)" << std::endl;
  return 0;
}
