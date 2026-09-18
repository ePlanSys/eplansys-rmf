#!/usr/bin/env bash
#
# The mission, without Gazebo, a fleet or a planner: rmf_action_node is the
# real thing, and drive_mission.py is the executor and the fleet adapter on
# either side of it. See that file for what is asserted and why.
#
# Needs a workspace with eplansys in it, since the performers link
# plansys2_executor. Sourcing that workspace is the caller's job.

set -euo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PORT=${BRIDGE_WEBSOCKET_PORT:-7891}
LOG=$(mktemp)

cleanup() {
  if [[ -n "${BRIDGE_PID:-}" ]]; then
    kill -9 "$BRIDGE_PID" 2>/dev/null || true
  fi
  rm -f "$LOG"
}
trap cleanup EXIT

if ! ros2 pkg executables eplansys_rmf_bridge 2>/dev/null | grep -q rmf_action_node; then
  echo "FAIL: rmf_action_node is not built. It needs plansys2_executor, so the"
  echo "      workspace has to carry eplansys as well as this repository."
  exit 1
fi

# Run the executable rather than `ros2 run`, which spawns it as a child: the
# wrapper dies on cleanup and the performers keep the websocket port.
BRIDGE_BIN="$(ros2 pkg prefix eplansys_rmf_bridge)/lib/eplansys_rmf_bridge/rmf_action_node"

"$BRIDGE_BIN" \
  --ros-args \
  -p task_map:="$HERE/mission_map.json" \
  -p websocket_port:="$PORT" \
  -p task_timeout:=20.0 \
  > "$LOG" 2>&1 &
BRIDGE_PID=$!

for _ in $(seq 1 60); do
  grep -q "performer for" "$LOG" && break
  sleep 0.25
done

if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  echo "--- rmf_action_node output ---"
  cat "$LOG"
  echo "FAIL: the performers died on startup"
  exit 1
fi

set +e
BRIDGE_WEBSOCKET_PORT="$PORT" python3 "$HERE/drive_mission.py"
RESULT=$?
set -e

echo "--- rmf_action_node output ---"
cat "$LOG"

if [[ $RESULT -ne 0 ]]; then
  echo "FAIL: the mission did not come out"
  exit $RESULT
fi

echo "PASS"
