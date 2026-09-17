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

// Which reading a sensing action reports. These are the rules a green demo
// run cannot show are right: a run that attributes a genuine measurement to
// the wrong site still takes a branch, updates the model and checks the goal.

#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "eplansys_rmf_bridge/Observations.hpp"

using eplansys_rmf_bridge::ActionSpec;
using eplansys_rmf_bridge::Observations;
using eplansys_rmf_bridge::OutcomeSource;
using eplansys_rmf_bridge::observed_place;

namespace
{

constexpr std::int64_t kSecond = 1000000000;

}  // namespace

TEST(ObservedPlace, TheOutcomeArgumentNamesThePlace)
{
  ActionSpec spec;
  spec.outcome_arg = 1;
  spec.waypoint_arg = 2;
  EXPECT_EQ(observed_place(spec, {"scout", "a07", "dock"}), "a07");
}

TEST(ObservedPlace, TheWaypointArgumentServesWhenNoOutcomeArgumentIsDeclared)
{
  ActionSpec spec;
  spec.waypoint_arg = 1;
  EXPECT_EQ(observed_place(spec, {"scout", "a15"}), "a15");
}

TEST(ObservedPlace, AnActionWithOneFixedSiteNamesNone)
{
  ActionSpec spec;
  EXPECT_EQ(observed_place(spec, {"scout"}), "");

  spec.outcome_arg = 3;   // past the end of what was dispatched
  EXPECT_EQ(observed_place(spec, {"scout"}), "");
}

TEST(Observations, AReadingIsSplitIntoPlaceAndOutcomeAtTheFirstSpace)
{
  Observations held;
  const auto reading = held.record("a07 e-scan-dirty", 0);
  EXPECT_EQ(reading.place, "a07");
  EXPECT_EQ(reading.outcome, "e-scan-dirty");
  EXPECT_TRUE(reading.changed);

  EXPECT_FALSE(held.record("a07 e-scan-dirty", kSecond).changed);
  EXPECT_TRUE(held.record("a07 e-scan-clean", kSecond).changed);

  const auto bare = held.record("e-scan-clean", 0);
  EXPECT_EQ(bare.place, "");
  EXPECT_EQ(bare.outcome, "e-scan-clean");
}

TEST(Observations, WhatTheFleetCarriedOutranksEveryReading)
{
  Observations held;
  held.record("a07 e-scan-clean", 5 * kSecond);
  held.record("e-scan-clean", 5 * kSecond);

  const auto resolved = held.resolve("e-scan-dirty", "a07", 0, "e-scan-clean");
  EXPECT_EQ(resolved.outcome, "e-scan-dirty");
  EXPECT_EQ(resolved.source, OutcomeSource::Carried);
}

// The failure the place key exists for: two robots at two sites, one latched
// topic, and the second scan subscribing after both have reported.
TEST(Observations, TwoScansOfTwoSitesEachReadTheirOwn)
{
  for (const bool dirty_first : {true, false}) {
    Observations held;
    if (dirty_first) {
      held.record("a15 e-scan-dirty", 1 * kSecond);
      held.record("a07 e-scan-clean", 2 * kSecond);
    } else {
      held.record("a07 e-scan-clean", 1 * kSecond);
      held.record("a15 e-scan-dirty", 2 * kSecond);
    }

    EXPECT_EQ(held.resolve("", "a15", 10 * kSecond, "").outcome, "e-scan-dirty");
    EXPECT_EQ(held.resolve("", "a07", 10 * kSecond, "").outcome, "e-scan-clean");
  }
}

// A robot reaches its site during the goto before the scan, so the right
// reading is older than the scan. Timing would discard it.
TEST(Observations, AReadingOfThisSiteCountsHoweverOldItIs)
{
  Observations held;
  held.record("a07 e-scan-dirty", 1 * kSecond);

  const auto resolved = held.resolve("", "a07", 30 * kSecond, "e-scan-clean");
  EXPECT_EQ(resolved.outcome, "e-scan-dirty");
  EXPECT_EQ(resolved.source, OutcomeSource::Place);
  EXPECT_TRUE(resolved.elsewhere.empty());
}

TEST(Observations, AReadingOfAnotherSiteIsNeverThisSitesAnswer)
{
  Observations held;
  held.record("a15 e-scan-dirty", 1 * kSecond);

  const auto resolved = held.resolve("", "a23", 0, "e-scan-clean");
  EXPECT_EQ(resolved.outcome, "e-scan-clean");
  EXPECT_EQ(resolved.source, OutcomeSource::Map);
  ASSERT_EQ(resolved.elsewhere.size(), 1u);
  EXPECT_EQ(resolved.elsewhere.at("a15"), "e-scan-dirty");
}

TEST(Observations, AnEmptyReadingOfThisSiteIsNoReading)
{
  Observations held;
  held.record("a07 ", 1 * kSecond);

  const auto resolved = held.resolve("", "a07", 0, "e-scan-clean");
  EXPECT_EQ(resolved.source, OutcomeSource::Map);
}

TEST(Observations, APlacelessReadingCountsOnlyFromTheActionOnward)
{
  Observations held;
  held.record("e-scan-dirty", 10 * kSecond);

  const auto during = held.resolve("", "", 10 * kSecond, "e-scan-clean");
  EXPECT_EQ(during.outcome, "e-scan-dirty");
  EXPECT_EQ(during.source, OutcomeSource::Placeless);

  const auto after = held.resolve("", "", 12 * kSecond, "e-scan-clean");
  EXPECT_EQ(after.outcome, "e-scan-clean");
  EXPECT_EQ(after.source, OutcomeSource::Map);
  EXPECT_TRUE(after.placeless_stale);
  EXPECT_EQ(after.placeless_age_ns, 2 * kSecond);
}

TEST(Observations, ASiteWithNoReadingStillTakesAPlacelessOne)
{
  Observations held;
  held.record("e-scan-dirty", 10 * kSecond);

  const auto resolved = held.resolve("", "a07", 5 * kSecond, "");
  EXPECT_EQ(resolved.outcome, "e-scan-dirty");
  EXPECT_EQ(resolved.source, OutcomeSource::Placeless);
}

TEST(Observations, WithNothingObservedTheMapIsTheLastResortAndMayBeSilent)
{
  Observations held;
  EXPECT_EQ(held.resolve("", "a07", 0, "e-scan-clean").source, OutcomeSource::Map);

  const auto nothing = held.resolve("", "a07", 0, "");
  EXPECT_EQ(nothing.source, OutcomeSource::Nothing);
  EXPECT_EQ(nothing.outcome, "");
}
