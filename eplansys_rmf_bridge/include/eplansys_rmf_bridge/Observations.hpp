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

#ifndef EPLANSYS_RMF_BRIDGE__OBSERVATIONS_HPP_
#define EPLANSYS_RMF_BRIDGE__OBSERVATIONS_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "eplansys_rmf_bridge/TaskMapping.hpp"

namespace eplansys_rmf_bridge
{

/// One message from the observation topic, as read.
struct Reading
{
  /// Empty for a reading that named no place.
  std::string place;
  std::string outcome;
  /// Whether it changed what was held, which is what is worth logging.
  bool changed{false};
};

/// Where a sensing action's outcome came from, most trusted first.
enum class OutcomeSource
{
  /// The fleet carried a token in the task state.
  Carried,
  /// A perception node reported this action's own site.
  Place,
  /// A perception node that names no site reported during the action.
  Placeless,
  /// The task map's stand-in for a sensor.
  Map,
  /// None of them had anything.
  Nothing,
};

struct ResolvedOutcome
{
  std::string outcome;
  OutcomeSource source{OutcomeSource::Nothing};

  /// The readings held for other sites, when the action named a site and none
  /// of them was for it. Worth reporting: the perception node is publishing,
  /// just not about this place.
  std::map<std::string, std::string> elsewhere;

  /// Set when a placeless reading was passed over as older than the action,
  /// with how much older, in nanoseconds.
  bool placeless_stale{false};
  std::int64_t placeless_age_ns{0};
};

/// The place a sensing action is about, in the vocabulary perception
/// publishes: the action's own argument, before the zones table turns it into
/// an RMF waypoint.
///
/// `outcome_arg` first, because that is the argument the map already declares
/// as the one selecting what is found; `waypoint_arg` after it, so an action
/// that says where it goes but not what it finds still names a place. Empty for
/// an action with neither, which senses one fixed site and has nothing to match.
std::string observed_place(const ActionSpec & spec, const std::vector<std::string> & arguments);

/// What perception has reported, held for the sensing actions to read.
///
/// There is one performer per action name and one observation topic, so every
/// scan of every site runs through one of these. The topic is latched, so the
/// second scan is handed the first one's answer the moment it subscribes.
/// Readings are therefore kept by place, and an action reads the entry for its
/// own site: two scans of two places cannot answer for each other however they
/// are ordered in time.
///
/// Timing cannot substitute for place. A robot commonly arrives at its site
/// during the `goto` that precedes the scan, so the correct reading is already
/// seconds old when the scan begins, and a rule that accepted only readings
/// newer than the action would throw the right answer away. A reading that
/// names no place has nothing else to be matched on, and only for those is
/// time the rule.
class Observations
{
public:
  /// Take one message. "<place> <outcome>" is a reading of a named place, and
  /// is the form a domain with more than one site has to use; a bare
  /// "<outcome>" is a reading of wherever the fleet is, which is what a
  /// single-site perception node publishes. `received_ns` stamps a placeless
  /// reading.
  Reading record(const std::string & message, std::int64_t received_ns);

  /// The outcome a sensing action reports, trying in order: the token the
  /// fleet carried, the reading for `place`, a placeless reading received at or
  /// after `begun_ns`, and `configured`, the task map's stand-in.
  ResolvedOutcome resolve(
    const std::string & carried,
    const std::string & place,
    std::int64_t begun_ns,
    const std::string & configured) const;

private:
  std::map<std::string, std::string> at_place_;
  std::string placeless_;
  /// Zero until a placeless reading arrives, which is before any action
  /// begins and so reads as nothing observed yet.
  std::int64_t placeless_at_ns_{0};
};

}  // namespace eplansys_rmf_bridge

#endif  // EPLANSYS_RMF_BRIDGE__OBSERVATIONS_HPP_
