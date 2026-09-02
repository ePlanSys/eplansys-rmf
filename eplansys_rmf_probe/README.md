# eplansys_rmf_probe

Two diagnostics that establish the bridge's two ends against a real fleet
before any `ActionExecutorClient` exists.

## Why

Open-RMF reports a task's completion as a status token. `task_state.json`
has no result field, and `RobotUpdateHandle::ActionExecution::finished()`
takes no argument, so a performer that senses something has no sanctioned
way to say what it sensed. Two free-form fields do travel with the task:
an event's `detail` string, and the log entries a performer writes through
`underway()` and friends.

`state_probe` reads both and reports any value carrying a known prefix. That
settles whether an epistemic outcome token survives the trip.

On Humble the task state and log stream leaves the fleet adapter over the
websocket named by the adapter's `server_uri` parameter, and over nothing
else: the ROS 2 mirror of those topics exists only on rolling. `state_probe`
is therefore a websocket server, built on the bridge's `WebsocketFeed`.

Not on `rmf_websocket::BroadcastServer`, which is the obvious choice and does
not work. Measured against 2.1.8: an adapter reports its connection open and
publishes without error while that server's callback never fires, and a plain
websocket server on the same port receives every frame from the same adapter.
It also aborts the process on the first non-JSON frame, and `BroadcastClient`
opens by sending a bare `Hello`.

## Running

The `rmf_demos` panel occupies port 7878, so give the probe its own port and
point the adapter at it. The panel cannot also receive the stream, since the
adapter has one `server_uri`.

```
ros2 run eplansys_rmf_probe state_probe --port 7879
```

```
ros2 launch eplansys_rmf_demo office_fleet.launch.xml \
    use_rmf_panel:=false server_uri:="ws://localhost:7879"
```

`office_fleet.launch.xml` and not `rmf_demos_gz_classic/office.launch.xml`:
that one forwards neither `server_uri` nor `use_rmf_panel` down to
`common.launch.xml`, so passing them to it has no effect at all.

Then submit a task pinned to a named robot:

```
ros2 run eplansys_rmf_probe submit_probe -F tinyRobot -R tinyRobot1 \
    -p patrol_A1
```

`submit_probe` prints the booking id it was given. `state_probe` groups
everything it reports under that same id, which is the key the bridge will
use to match a finished RMF task to the epistemic action that asked for it.

Pass `--dispatch` to send a `dispatch_task_request` instead and let RMF's
dispatcher choose the robot, for comparison against the pinned path.

## Reading the output

```
[11:32:59] task compose.dispatch-0: assigned to tinyRobot/tinyRobot1
[11:32:59] task compose.dispatch-0: none -> underway
[11:33:00] task compose.dispatch-0: OUTCOME via log: "e-scan-dirty"
[11:33:01] task compose.dispatch-0: underway -> completed

  task     compose.dispatch-0
  category compose
  robot    tinyRobot/tinyRobot1
  status   completed
  outcome  "e-scan-dirty" (via log)
```

A stock `go_to_place` task carries no outcome, and the summary says so. To
see one, have the fleet adapter's action executor write the token. In
`rmf_demos_fleet_adapter/fleet_adapter.py` that is a single line on the
`execution` handle:

```python
execution.underway("eplansys.outcome=e-scan-dirty")
```

## Notes

`PYTHONNOUSERSITE=1` is needed on this machine for anything that launches
`rmf_demos_panel`, whose Flask is shadowed by a pip `--user` Werkzeug.
Neither probe is affected.
