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

## Checking that it came out

The goal has three conjuncts, and the third is a negative one: the observer
must not come to know whether the site is contaminated. A negative goal is
satisfied by everything that fails to happen, and `observer` is bound to no
robot and never moves, so the demo satisfies it whatever it does. A run that
honoured the private channel and a run that achieved nothing look the same
from the outside.

`mission_check` asks instead. It runs once the mission process has exited and
before the launch file shuts the system down, which is the only window in
which the executor is finished and the epistemic state is still alive, and it
puts each conjunct to `epistemic_state/check_formula`:

```
ok   (Kw scout contaminated) holds
ok   (Kw relay contaminated) holds
ok   (Kw observer contaminated) does not hold
```

The formulas are the `must_hold` and `must_not_hold` parameters, defaulting to
the survey's three conjuncts, because the claims belong to the mission and not
to the node. `check:=false` leaves the whole thing out.

A call that is not answered is reported as `UNCHECKED` and counts as a
failure, never as a false answer. The distinction is the point: an epistemic
state that is unreachable would otherwise report that the observer knows
nothing, which is the very result the demo exists to establish, and it would
report it about a system nobody asked.

Note what this does and does not settle. It establishes that the model ended
where the goal wanted it, which is a statement about the epistemic state after
execution. It is not evidence that the observer could not physically have
learnt the finding: the speech acts are still `local` and submit no RMF task,
so there is no channel for anything to leak through yet.

## Ports

The bridge is the websocket server the fleet adapter dials, on 7879. The
`rmf_demos` panel owns 7878 and is switched off here, since an adapter has one
`server_uri` and cannot feed both.

`office_fleet.launch.xml` exists because `rmf_demos_gz_classic/office.launch.xml`
forwards neither `server_uri` nor `use_rmf_panel` down to `common.launch.xml`.
Setting them on its command line does nothing, silently. This assembles the
same demo from the same pieces and passes both.
