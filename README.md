# eplansys-rmf

An Open-RMF execution interface for [ePlanSys](https://github.com/ePlanSys/eplansys).
Epistemic policies are dispatched as fleet-level RMF tasks, and the observations
the robots make while executing them are returned to the epistemic state.

Status: design. No implementation exists.

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

## Open questions

### 1. Allocation authority

Both systems perform allocation, and their decisions need not coincide. The
planner assigns actions to named agents during search, and the epistemic model
records knowledge against those names. The RMF dispatcher receives bids and
selects a robot. Where the two selections differ, the model records that an
agent knows something it never sensed, and no error is raised.

Two schemes are available.

Under the pinned scheme, the bridge binds each RMF task to the robot named by
the planner. This is sound and simple. It forgoes RMF's allocation while
retaining its traffic management, and is the intended starting point.

Under the role-based scheme, the planner reasons over abstract agents, RMF
selects the robot, and the bridge relabels the epistemic agent after dispatch.
Relabelling an agent in a Kripke model during execution is not a trivial
operation. This is the research variant.

The choice is to be recorded here once settled.

### 2. Outcome transport

It must be established how an RMF task reports completion, and whether a result
payload may accompany that report. The relevant interfaces are `rmf_task_msgs`,
the dispatcher node, and the websocket API in `rmf_api_msgs`.

If a payload can be returned, the bridge reads the outcome from it. If
completion is reported as a bare success or failure, the outcome requires a
separate channel, and the architecture given above is altered accordingly.

The project rests on this assumption, and it should be resolved before any other
work begins.

### 3. Action to task mapping

`eplansys` maps grounded epistemic action names to dispatchable expressions
through an `action_mapping` JSON file. The corresponding mapping here targets
RMF task descriptions. It remains to be decided whether to reuse that file
format against a different target or to define a separate one.

## Packages

| package | contents |
| --- | --- |
| `eplansys_rmf_bridge` | the `ActionExecutorClient` that submits RMF tasks |
| `eplansys_rmf_demo` | the survey mission over an RMF fleet |

## Reference scenario

The target scenario is the survey domain of `eplansys`. It comprises three
robots and a site that may be contaminated, under a goal of three conjuncts.
The scout is required to find out whether the site is contaminated, the relay is
required to come to know the result, and an observer is required not to. The
planner declines to broadcast and uses a private channel instead, since
broadcasting would falsify the third conjunct.

Executing this domain over an RMF fleet, with RMF driving the robots,
constitutes the intended demonstration of the bridge.

## Dependencies

- ROS 2 Humble or Rolling
- [eplansys](https://github.com/ePlanSys/eplansys)
- [Open-RMF](https://github.com/open-rmf)

## Licence

Apache-2.0