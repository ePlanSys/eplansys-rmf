# eplansys-rmf

An Open-RMF execution interface for [ePlanSys](https://github.com/ePlanSys/eplansys).
Epistemic policies are dispatched as fleet-level RMF tasks, and the observations
the robots make while executing them are returned to the epistemic state.

Status: the survey mission runs end to end over the `rmf_demos` office fleet, in
both of its branches, and the two-site survey runs over it with both robots
driving at once. CI drives the performers on every push, with a fake
executor on one side and a fake fleet on the other, so what the mission claims
is checked without Gazebo; see
[`eplansys_rmf_bridge/test/integration`](eplansys_rmf_bridge/test/integration).

## Separation from the planning stack

Open-RMF is a substantial dependency. Requiring it from `eplansys` would render
the planning stack unusable for installations that do not operate a fleet. The
bridge is therefore maintained as a separate package, following the arrangement
already used by `plansys2_aletheia_plan_solver` for its external binary.

## Division of responsibility

The two systems decide disjoint things. ePlanSys determines what has to be
found out, which agent should act, and which agents may come to know the result.
Open-RMF determines the allocation of floor space, lifts and doors, and which
robot is available to perform the work.

Neither system covers the other's domain. Open-RMF holds no representation of
knowledge, and ePlanSys holds no representation of shared physical resources.
The bridge supplies the interface between them.

## Architecture

<p align="center">
  <img src="docs/pipeline.png" width="820"
       alt="EPDDL to Open-RMF execution pipeline">
</p>

The bridge is an `ActionExecutorClient`. It accepts a dispatched action, submits
the corresponding RMF task, waits for completion, and calls `finish()` with the
outcome the robot observed. `plansys2_msgs/ActionExecution` provides an
`outcome` field for this purpose, on which the policy branches.

## Decisions

### 1. Allocation authority

Both systems perform allocation, and their decisions need not coincide. The
planner assigns actions to named agents during search, and the epistemic model
records knowledge against those names. The RMF dispatcher receives bids and
selects a robot. Where the two selections differ, the model records that an
agent knows something it never sensed, and no error is raised.

**Pinned.** The bridge binds each RMF task to the robot the planner named,
through the `agents` table of its task map. The mechanism is
`robot_task_request`, which carries a fleet and a robot and is handled directly
by that robot's own `TaskManager` without a bid. RMF's traffic management,
lifts and doors are retained in full; only its allocation is forgone.

An agent bound to no robot is a hard error, and the bridge says so and refuses
the action. The alternative, letting RMF choose, is precisely the failure this
decision exists to prevent.

The role-based scheme, in which the planner reasons over abstract agents and
the bridge relabels the epistemic agent after dispatch, remains the research
variant. Relabelling an agent in a Kripke model during execution is not a
trivial operation.

### 2. Outcome transport

Settled by reading the sources, since Open-RMF's own documentation does not
address it: **a task carries no result payload.**

`rmf_api_msgs`' `task_state.json` has no result, return or output property at
any level, and its completion-bearing fields are a `status` token from a fixed
enumeration and a finish time.
`RobotUpdateHandle::ActionExecution::finished()` takes no argument, which is
the exact API a performer would report through. The newer `DynamicEvent`
action result carries a failure string, a status string and an event id, and
nothing of the domain.

Two free-form fields do travel with a task, and the bridge reads both:

- an event's `detail` string, set through `SimpleEventState::update_detail`
  and forwarded verbatim into every state update;
- log entries, which a performer writes through `underway()` and its
  neighbours.

A value carrying the prefix `eplansys.outcome=` in either is read as the token
the policy branches on. The log route is the one reachable from an ordinary
`perform_action` callback; `detail` is tidier and needs a custom
`rmf_task_sequence` event.

On Humble this stream leaves the fleet adapter over the websocket named by the
adapter's `server_uri` parameter and over nothing else. `StandardNames.hpp`
declares only `task_api_requests` and `task_api_responses`; the ROS 2 mirror of
`task_state_update` and `task_log_update` exists on rolling and not here. So
the bridge is the websocket server the adapter dials.

### 3. Action to task mapping

**A separate file**, keyed differently.

`eplansys`'s `action_mapping.json` maps plank's grounded names to PlanSys2
action expressions, and does its work inside the planner. By the time an action
reaches a performer the grounded name is gone and what remains is a name and
arguments, so a map keyed on grounded names could not be consulted here. The
two files answer different questions at different times, and collapsing them
would mean the bridge could not look anything up.

The bridge's map binds agents to robots and says where each action sends one,
and for a speech act which audience it reaches.
`eplansys_rmf_demo/config/office_survey.json` is the worked example, and
`office_sites.json` is the two-site one: two agents bound to the two office
robots, two sites at different waypoints, and the relay and the observer bound
to nothing because neither ever moves.

### 4. Speech acts

A private announcement and a public one differ only in who hears them, and
executing both as the same pause discards the one thing the survey domain is
about. **The map names the audience**, and the bridge puts the utterance on a
topic whose subscribers are that audience: `public` reaches every agent,
`private` reaches the listener the action names and nobody else.

This realises the addressing and not the confidentiality. Any node may
subscribe to a private topic, so what the demo establishes is that the team
used a channel the observer was not on. Enforcing that it could not have
listened is a DDS partition or an SROS 2 permission, built out of these same
audiences, and belongs to a deployment rather than to the bridge.

The utterance carries the outcome the speaker's own sensing action reported,
because that is all that is left: `eplansys`'s action map sends `relay-dirty`
and `relay-clean` alike to `(relay ?i ?j)`, so a performer cannot tell from its
arguments which of the two it is saying.

### 5. Interrupting the link

A fleet that stops answering is not the same thing as a fleet that has stopped.
Publishing `false` on `/eplansys/rmf_link` (the `link_topic` parameter) makes the
bridge drop what arrives over the websocket from then on: the socket stays up,
the frames are still counted, the requests still go out, and what is lost is the
road back. `true` reads it again.

It exists for experiments. A mission met with an outage has its action time out
and, if it replans, finds a fleet that is still there to answer the next
attempt. Killing the adapter instead measures something else: it comes back
believing its robots are at their chargers, and the tasks it accepts afterwards
never finish.

## Packages

| package | contents |
| --- | --- |
| `eplansys_rmf_bridge` | the `ActionExecutorClient` that submits RMF tasks |
| `eplansys_rmf_demo` | the survey missions over an RMF fleet, one site and two |
| `eplansys_rmf_probe` | diagnostics: submit one task, and watch what returns |

`eplansys_rmf_demo` also holds `radio`, one agent's ears on the speech-act
channels, and `mission_check`, which asks once the robots have stopped whether
the goal actually came out --- of the epistemic state, and of the transcripts
of who was spoken to.

## Reference scenarios

The first is the survey domain of `eplansys`: three robots and a site that may
be contaminated, under a goal of three conjuncts. The scout is required to find
out whether the site is contaminated, the relay is required to come to know the
result, and an observer is required not to. The planner declines to broadcast
and uses a private channel instead, since broadcasting would falsify the third
conjunct. One robot moves.

The second is the two-site survey: the same mission twice over, with each scout
carrying the instrument its own site reads. Six conjuncts, three per site, and
two halves that share no agent, no atom and no precondition. Both robots move,
and where the policy's independent runs are dispatched together they move at the
same time, which is where Open-RMF's traffic management is doing something the
mission could not do for itself.

## Running it

```
ros2 launch eplansys_rmf_demo survey_rmf_launch.py
ros2 launch eplansys_rmf_demo survey_rmf_launch.py site:=clean

ros2 launch eplansys_rmf_demo survey_sites_rmf_launch.py
ros2 launch eplansys_rmf_demo survey_sites_rmf_launch.py north:=clean south:=dirty
ros2 launch eplansys_rmf_demo survey_sites_rmf_launch.py parallel:=false
```

One command brings up the office fleet, the planning system and the bridge.
`site:` chooses what the scout turns out to find, and the policy takes a
different branch for each: `relay-dirty_relay_scout` against
`relay-clean_relay_scout`, with the robot driven by RMF either way.

The two-site mission takes `north:` and `south:` for the same reason, one per
site, and `parallel:` for how its policy is dispatched. Measured on the office
fleet, north dirty and south clean:

| dispatch | mission |
| --- | --- |
| a node at a time | 67.5 s |
| independent runs together | 51.5 s |

The saving is one drive across the office. With `parallel:=true` both `go_to_place`
tasks are submitted in the same instant and RMF routes two robots at once; with
it off the second waits for the first to arrive, scan and report.

`rmf:=false` leaves the fleet to another terminal, and `headless:=true` runs
Gazebo without a window. When the robots stop, `mission_check` puts the goal's
three conjuncts to the epistemic state, and reads the agents' radio transcripts
to see who was actually spoken to; `check:=false` leaves it out.

## Building

`eplansys_rmf_bridge` looks for plansys2 with `QUIET` and builds its library
without it, so the RMF half compiles and its tests run on a machine that has
Open-RMF and no `eplansys`. The performers need both.

```
sudo apt install ros-humble-rmf-dev libwebsocketpp-dev libboost-system-dev
colcon build
```

`rmf_demos` is not released into Humble and has to be built from source for
the demo; its `humble` branch is the one to use.

## Dependencies

- ROS 2 Humble or Rolling
- [eplansys](https://github.com/ePlanSys/eplansys)
- [Open-RMF](https://github.com/open-rmf)

## Contributing

The tree follows the ament default style rather than Open-RMF's, and the CI
lint job is reproducible locally in three commands. See
[CONTRIBUTING.md](CONTRIBUTING.md).

## Licence

Apache-2.0