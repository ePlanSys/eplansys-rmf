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

#include "eplansys_rmf_bridge/TaskMapping.hpp"

#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace eplansys_rmf_bridge
{

TaskMapping TaskMapping::load(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot read task map: " + path);
  }

  nlohmann::json doc = nlohmann::json::parse(file, nullptr, false);
  if (doc.is_discarded() || !doc.is_object()) {
    throw std::runtime_error("task map is not a JSON object: " + path);
  }

  TaskMapping map;

  if (doc.contains("agents")) {
    for (const auto & [name, value] : doc.at("agents").items()) {
      if (!value.is_object() || !value.contains("robot")) {
        throw std::runtime_error(
                "agent \"" + name + "\" needs an object with a robot: " + path);
      }
      AgentBinding binding;
      binding.fleet = value.value("fleet", std::string{});
      binding.robot = value.at("robot").get<std::string>();
      map.agents_.emplace(name, std::move(binding));
    }
  }

  if (!doc.contains("actions")) {
    throw std::runtime_error("task map has no actions: " + path);
  }

  for (const auto & [name, value] : doc.at("actions").items()) {
    if (!value.is_object()) {
      throw std::runtime_error("action \"" + name + "\" is not an object: " + path);
    }

    ActionSpec spec;
    spec.local = value.value("local", false);
    spec.duration = value.value("duration", 1.0);
    spec.category = value.value("category", std::string{"go_to_place"});
    spec.waypoint = value.value("waypoint", std::string{});
    spec.sensing = value.value("sensing", false);
    spec.default_outcome = value.value("default_outcome", std::string{});

    if (value.contains("orientation")) {
      spec.orientation = value.at("orientation").get<double>();
    }

    if (value.contains("waypoints")) {
      for (const auto & [agent, wp] : value.at("waypoints").items()) {
        spec.waypoints.emplace(agent, wp.get<std::string>());
      }
    }

    if (!spec.local && spec.waypoint.empty() && spec.waypoints.empty()) {
      throw std::runtime_error(
              "action \"" + name + "\" moves a robot but names no waypoint: " + path);
    }

    map.actions_.emplace(name, std::move(spec));
  }

  return map;
}

std::optional<AgentBinding> TaskMapping::agent(const std::string & name) const
{
  const auto it = agents_.find(name);
  if (it == agents_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<ActionSpec> TaskMapping::action(const std::string & name) const
{
  const auto it = actions_.find(name);
  if (it == actions_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::string TaskMapping::waypoint_for(
  const ActionSpec & spec, const std::string & agent) const
{
  const auto it = spec.waypoints.find(agent);
  if (it != spec.waypoints.end()) {
    return it->second;
  }
  return spec.waypoint;
}

std::vector<std::string> TaskMapping::action_names() const
{
  std::vector<std::string> names;
  names.reserve(actions_.size());
  for (const auto & [name, spec] : actions_) {
    (void)spec;
    names.push_back(name);
  }
  return names;
}

}  // namespace eplansys_rmf_bridge
