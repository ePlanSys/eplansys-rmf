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

#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "eplansys_rmf_bridge/TaskMapping.hpp"

using eplansys_rmf_bridge::TaskMapping;

namespace
{

class TempMap
{
public:
  explicit TempMap(const std::string & contents)
  {
    std::vector<char> tmpl{'/', 't', 'm', 'p', '/', 'e', 'p', 'r', 'm', 'f',
      'X', 'X', 'X', 'X', 'X', 'X', '\0'};
    const int fd = mkstemp(tmpl.data());
    close(fd);
    path_ = tmpl.data();

    std::ofstream out(path_);
    out << contents;
  }

  ~TempMap()
  {
    std::remove(path_.c_str());
  }

  const std::string & path() const {return path_;}

private:
  std::string path_;
};

const char * kHotel =
  R"({
  "zones": {
    "lobby": "lobby",
    "l2_suite": "L2_master_suite",
    "l3_suite": "L3_master_suite"
  },
  "agents": {
    "inspector": {"fleet": "tinyRobot", "robot": "tinyBot_1"}
  },
  "actions": {
    "goto_zone":  {"waypoint_arg": 2},
    "look_into":  {"waypoint_arg": 1, "sensing": true, "outcome_arg": 1,
                   "outcomes": {"l2_suite": "e-inspect-wet"},
                   "default_outcome": "e-inspect-dry"},
    "radio":      {"local": true}
  }
})";

const char * kSurvey =
  R"({
  "agents": {
    "scout": {"fleet": "tinyRobot", "robot": "tinyRobot1"},
    "relay": {"fleet": "tinyRobot", "robot": "tinyRobot2"}
  },
  "actions": {
    "goto_site": {"waypoint": "pantry"},
    "scan": {"waypoint": "pantry", "sensing": true,
             "default_outcome": "e-scan-dirty",
             "waypoints": {"scout": "lounge"}},
    "relay": {"local": true, "duration": 2.0}
  }
})";

}  // namespace

TEST(TaskMapping, BindsAgentsToRobots)
{
  TempMap file{kSurvey};
  const auto map = TaskMapping::load(file.path());

  const auto relay = map.agent("relay");
  ASSERT_TRUE(relay.has_value());
  EXPECT_EQ(relay->fleet, "tinyRobot");
  EXPECT_EQ(relay->robot, "tinyRobot2");
}

// An unbound agent has to stay unbound. Falling back to any robot would let
// the epistemic model credit an agent with sensing something it never saw,
// and nothing downstream would notice.
TEST(TaskMapping, UnboundAgentIsNullopt)
{
  TempMap file{kSurvey};
  const auto map = TaskMapping::load(file.path());
  EXPECT_FALSE(map.agent("observer").has_value());
}

TEST(TaskMapping, SpeechActsSubmitNoTask)
{
  TempMap file{kSurvey};
  const auto map = TaskMapping::load(file.path());

  const auto relay = map.action("relay");
  ASSERT_TRUE(relay.has_value());
  EXPECT_TRUE(relay->local);
  EXPECT_DOUBLE_EQ(relay->duration, 2.0);

  const auto goto_site = map.action("goto_site");
  ASSERT_TRUE(goto_site.has_value());
  EXPECT_FALSE(goto_site->local);
}

TEST(TaskMapping, SensingCarriesADefaultOutcome)
{
  TempMap file{kSurvey};
  const auto map = TaskMapping::load(file.path());

  const auto scan = map.action("scan");
  ASSERT_TRUE(scan.has_value());
  EXPECT_TRUE(scan->sensing);
  EXPECT_EQ(scan->default_outcome, "e-scan-dirty");

  const auto goto_site = map.action("goto_site");
  ASSERT_TRUE(goto_site.has_value());
  EXPECT_FALSE(goto_site->sensing);
}

TEST(TaskMapping, PerAgentWaypointWins)
{
  TempMap file{kSurvey};
  const auto map = TaskMapping::load(file.path());

  const auto scan = *map.action("scan");
  EXPECT_EQ(map.waypoint_for(scan, "scout", {}), "lounge");
  EXPECT_EQ(map.waypoint_for(scan, "relay", {}), "pantry");
}

TEST(TaskMapping, ListsEveryActionForAPerformer)
{
  TempMap file{kSurvey};
  const auto map = TaskMapping::load(file.path());
  EXPECT_EQ(map.action_names().size(), 3u);
}

// A moving action with nowhere to go is a hole in the map, and it should say
// so at start up instead of at the moment a robot is asked to move.
TEST(TaskMapping, MovingActionWithoutAWaypointIsRejected)
{
  TempMap file{R"({"actions": {"goto_site": {"category": "go_to_place"}}})"};
  EXPECT_THROW(TaskMapping::load(file.path()), std::runtime_error);
}

TEST(TaskMapping, MissingFileIsRejected)
{
  EXPECT_THROW(
    TaskMapping::load("/nonexistent/task_map.json"), std::runtime_error);
}

TEST(TaskMapping, MapWithoutActionsIsRejected)
{
  TempMap file{R"({"agents": {"scout": {"robot": "tinyRobot1"}}})"};
  EXPECT_THROW(TaskMapping::load(file.path()), std::runtime_error);
}

// The destination of a move is an argument, not a fixture: one `goto_zone`
// performer serves every zone in the building.
TEST(TaskMapping, WaypointComesFromTheNamedArgument)
{
  TempMap file{kHotel};
  const auto map = TaskMapping::load(file.path());
  const auto go = *map.action("goto_zone");

  EXPECT_EQ(
    map.waypoint_for(go, "inspector", {"inspector", "lobby", "l3_suite"}),
    "L3_master_suite");
  EXPECT_EQ(
    map.waypoint_for(go, "inspector", {"inspector", "l3_suite", "lobby"}),
    "lobby");
}

// A zone the table does not name is passed through, so a map may name RMF
// waypoints directly and skip the indirection.
TEST(TaskMapping, UnlistedZoneIsItsOwnWaypoint)
{
  TempMap file{kHotel};
  const auto map = TaskMapping::load(file.path());
  EXPECT_EQ(map.waypoint_of_zone("restaurant"), "restaurant");
}

// Which suite is flooded is the simulator's ground truth, and a sensing action
// performed in several places has to report a different answer in each.
TEST(TaskMapping, OutcomeDependsOnWhereItLooked)
{
  TempMap file{kHotel};
  const auto map = TaskMapping::load(file.path());
  const auto look = *map.action("look_into");

  EXPECT_EQ(map.outcome_for(look, {"inspector", "l2_suite"}), "e-inspect-wet");
  EXPECT_EQ(map.outcome_for(look, {"inspector", "l3_suite"}), "e-inspect-dry");
}

// Too few arguments must not read off the end.
TEST(TaskMapping, ShortArgumentListFallsBack)
{
  TempMap file{kHotel};
  const auto map = TaskMapping::load(file.path());

  EXPECT_EQ(map.outcome_for(*map.action("look_into"), {}), "e-inspect-dry");
  EXPECT_TRUE(map.waypoint_for(*map.action("goto_zone"), "inspector", {}).empty());
}

// An action whose destination is an argument names no fixed waypoint, and
// must not be rejected for it.
TEST(TaskMapping, ArgumentDrivenMoveNeedsNoFixedWaypoint)
{
  TempMap file{kHotel};
  EXPECT_NO_THROW(TaskMapping::load(file.path()));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
