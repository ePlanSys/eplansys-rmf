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
eplansys, which the image does not have.

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

## What CI does not cover

`eplansys` is deliberately absent from the CI image. `eplansys_rmf_bridge`
finds plansys2 with `QUIET` and builds its library either way, so CI exercises
the whole RMF half -- the task map, the websocket, the submission path, and
which reading a sensing action reports -- and none of the performers. Logic a
performer needs to get right belongs in the library for that reason: the
performer should only log it and call `finish()`. `eplansys_rmf_demo` guards `mission_check` the same
way and installs the launch files without it. Changes to the performers and to
`mission_check` are covered by running the demo, not by a green tick.
