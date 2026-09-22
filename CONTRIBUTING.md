# Contributing

## Code style

The tree follows the ament default that `ament_uncrustify` enforces: K&R
bracing, `const auto & x` spacing, four-column continuations. This matters
because Open-RMF itself is written in Allman style, so code copied or adapted
from `rmf_ros2` will not match this tree and CI will reject it. Convert it
rather than reconfiguring the linter, so the tree reads one way throughout.

Copyright notices go in `//` line comments, not `/* */` blocks. `ament_copyright`
does not recognise the block form and reports the file as having no notice at
all, which reads as a missing licence rather than a formatting nit.

## Checking before you push

The `lint` job runs no build, so it is cheap to reproduce in full:

```
source /opt/ros/humble/setup.bash
ament_uncrustify eplansys_rmf_bridge eplansys_rmf_probe eplansys_rmf_demo
ament_copyright eplansys_rmf_bridge eplansys_rmf_probe eplansys_rmf_demo
```

`--reformat` fixes the first one in place. Both need the linters installed:

```
sudo apt install ros-humble-ament-cmake-uncrustify \
                 ros-humble-ament-cmake-copyright
```

`eplansys_rmf_demo` is linted but not built by CI: `mission_check` needs
eplansys, which the `build and test` image does not have.

The `build and test` job is the ordinary build plus the unit tests and the
websocket smoke test. The smoke test needs `install/setup.bash` sourced on top
of the ROS environment, because it drives the built `state_probe`:

```
colcon build --packages-select eplansys_rmf_bridge eplansys_rmf_probe
colcon test --packages-select eplansys_rmf_bridge
colcon test-result --verbose
source install/setup.bash
.github/workflows/smoke_test.sh
```

The `mission end to end` job builds eplansys at the release named in the
workflow, so `rmf_action_node` exists, and drives it from both sides:
`test/integration/drive_mission.py` is the executor on `actions_hub` and the
fleet adapter on `task_api_requests` and the websocket. It asserts the five
things the README promises about a mission --- the pinned robot and its mapped
waypoint, an outcome the fleet reported, a reading matched to the site it was
taken at, a private channel reaching its listener alone, and an unbound agent
failing rather than guessing.

Locally it needs a workspace carrying both repositories:

```
colcon build --packages-up-to eplansys_rmf_bridge
source install/setup.bash
eplansys_rmf_bridge/test/integration/run.sh
```

## What CI does not cover

Gazebo, the fleet adapter and the planner. The mission job fakes RMF rather
than running it, so what is checked is the bridge's side of every exchange:
what it submits, what it does with what comes back, and what it says to whom.
That a robot actually crosses the floor, that RMF's traffic management routes
it, and that a policy from the planner selects these actions in this order are
still shown by running the demo.

Logic a performer needs to get right belongs in the library rather than the
node, so the unit tests can reach it: the performer should log what it chose
and call `finish()`. `eplansys_rmf_demo` is guarded the same way --
`mission_check` needs eplansys -- and its launch files install without it.
