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

#include "eplansys_rmf_bridge/Observations.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace eplansys_rmf_bridge
{

std::string observed_place(const ActionSpec & spec, const std::vector<std::string> & arguments)
{
  for (const auto index : {spec.outcome_arg, spec.waypoint_arg}) {
    if (index >= 0 && static_cast<std::size_t>(index) < arguments.size()) {
      return arguments[index];
    }
  }
  return {};
}

Reading Observations::record(const std::string & message, std::int64_t received_ns)
{
  Reading reading;

  const auto space = message.find(' ');
  if (space != std::string::npos) {
    reading.place = message.substr(0, space);
    reading.outcome = message.substr(space + 1);
    auto & held = at_place_[reading.place];
    reading.changed = held != reading.outcome;
    held = reading.outcome;
    return reading;
  }

  reading.outcome = message;
  reading.changed = placeless_ != message;
  placeless_ = message;
  placeless_at_ns_ = received_ns;
  return reading;
}

ResolvedOutcome Observations::resolve(
  const std::string & carried,
  const std::string & place,
  std::int64_t begun_ns,
  const std::string & configured) const
{
  ResolvedOutcome resolved;

  // A token the fleet carried is the robot's own report, and nothing is more
  // direct than that.
  if (!carried.empty()) {
    resolved.outcome = carried;
    resolved.source = OutcomeSource::Carried;
    return resolved;
  }

  // Ahead of the map: a value measured at the site is an observation, and a
  // value read from the task map is a stand-in for one.
  if (!place.empty()) {
    const auto found = at_place_.find(place);
    if (found != at_place_.end() && !found->second.empty()) {
      resolved.outcome = found->second;
      resolved.source = OutcomeSource::Place;
      return resolved;
    }
    resolved.elsewhere = at_place_;
  }

  if (!placeless_.empty()) {
    if (placeless_at_ns_ >= begun_ns) {
      resolved.outcome = placeless_;
      resolved.source = OutcomeSource::Placeless;
      return resolved;
    }
    // Published before this action began, so it is another action's reading.
    resolved.placeless_stale = true;
    resolved.placeless_age_ns = begun_ns - placeless_at_ns_;
  }

  if (!configured.empty()) {
    resolved.outcome = configured;
    resolved.source = OutcomeSource::Map;
  }
  return resolved;
}

}  // namespace eplansys_rmf_bridge
