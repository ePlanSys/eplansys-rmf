# eplansys_rmf_demo

The site survey of `eplansys`, run over an Open-RMF fleet.

```
ros2 launch eplansys_rmf_demo survey_rmf_launch.py
ros2 launch eplansys_rmf_demo survey_rmf_launch.py site:=clean
```

Same mission, same EPDDL, same policy. What changes is who moves the robots:
`eplansys_demo`'s performers wait out a duration, and these submit RMF tasks
and wait for the fleet to report them done.

## The map

`config/office_survey.json` binds each epistemic agent to one robot of the
`rmf_demos` office fleet and says where each action sends it.

| agent | robot |
| --- | --- |
| `scout` | `tinyRobot1` |
| `relay` | `tinyRobot2` |
| `observer` | none |

`observer` is deliberately unbound. The office fleet has two robots, and the
mission's third conjunct is that the observer must not come to know, so the
planner does not send it anywhere. If a change to the goal ever made the
planner choose to move it, the bridge refuses the action and says why, which
is the behaviour worth having: a silent substitution would credit `observer`
with sensing something it never saw, and no error would be raised.

`goto_site` and `scan` both go to `pantry`. `scan` names the same waypoint the
robot has already reached, so RMF completes it almost at once; it stands in
for a sensing task, and a deployment would use a `perform_action` the fleet
declares. `relay` and `broadcast` are speech acts and submit nothing, because
RMF has no representation of saying things and no robot moves to say them.

## The outcome

`scan` is the one action whose result the policy branches on. RMF carries no
result payload of its own, so the token travels in a task's `detail` string or
a log entry, and `site:=dirty|clean` sets the `default_outcome` the bridge
falls back to when the fleet reports none. The fallback is logged as a
warning every time it happens.

To see the real path instead, have the fleet adapter's action executor write
the token. In `rmf_demos_fleet_adapter/fleet_adapter.py` that is one line on
the `execution` handle:

```python
execution.underway("eplansys.outcome=e-scan-dirty")
```

The bridge then reports what the fleet observed and ignores the default.

## Ports

The bridge is the websocket server the fleet adapter dials, on 7879. The
`rmf_demos` panel owns 7878 and is switched off here, since an adapter has one
`server_uri` and cannot feed both.

`office_fleet.launch.xml` exists because `rmf_demos_gz_classic/office.launch.xml`
forwards neither `server_uri` nor `use_rmf_panel` down to `common.launch.xml`.
Setting them on its command line does nothing, silently. This assembles the
same demo from the same pieces and passes both.
