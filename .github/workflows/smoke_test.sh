#!/usr/bin/env bash
#
# Drives state_probe the way a fleet adapter does and checks that an outcome
# token survives the trip.
#
# The two faults this catches are the two that actually happened. A websocket
# server that accepts a connection and never delivers a frame looks exactly
# like a fleet that is not running, and a non-JSON frame -- BroadcastClient
# opens by sending a bare `Hello` -- used to take the whole process down.

set -euo pipefail

PORT=${PORT:-7897}
LOG=$(mktemp)
CLIENT=$(mktemp --suffix=.py)

cleanup() {
  [[ -n "${PROBE_PID:-}" ]] && kill -9 "$PROBE_PID" 2>/dev/null || true
  rm -f "$LOG" "$CLIENT"
}
trap cleanup EXIT

cat > "$CLIENT" <<'PY'
"""A minimal RFC 6455 client, on the standard library alone.

The websockets package that jammy ships is 9.1, whose client cannot run on
Python 3.10 at all: it passes a loop= to asyncio.Lock, which 3.10 removed. A
handshake and a masked text frame are little enough code to not need it.
"""
import base64
import json
import os
import socket
import struct
import sys
import time

TASK = "smoke.task-0"


def state(status, detail=None):
    event = {"id": 3, "status": status, "name": "scan site"}
    if detail:
        event["detail"] = detail
    return {"type": "task_state_update", "data": {
        "booking": {"id": TASK},
        "category": "compose",
        "status": status,
        "assigned_to": {"group": "tinyRobot", "name": "tinyRobot1"},
        "phases": {"1": {"id": 1, "events": {"3": event}}}}}


def log(seq, text):
    return {"type": "task_log_update", "data": {"task_id": TASK, "phases": {
        "1": {"events": {"3": [{"seq": seq, "tier": "info",
                                "unix_millis_time": 0, "text": text}]}}}}}


def frame(payload):
    mask = os.urandom(4)
    masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    n = len(payload)
    if n < 126:
        header = struct.pack("!BB", 0x81, 0x80 | n)
    elif n < 65536:
        header = struct.pack("!BBH", 0x81, 0x80 | 126, n)
    else:
        header = struct.pack("!BBQ", 0x81, 0x80 | 127, n)
    return header + mask + masked


host, port = sys.argv[1], int(sys.argv[2])
sock = socket.create_connection((host, port), timeout=10)
key = base64.b64encode(os.urandom(16)).decode()
sock.sendall((
    "GET / HTTP/1.1\r\n"
    f"Host: {host}:{port}\r\n"
    "Upgrade: websocket\r\n"
    "Connection: Upgrade\r\n"
    f"Sec-WebSocket-Key: {key}\r\n"
    "Sec-WebSocket-Version: 13\r\n\r\n"
).encode())

response = sock.recv(4096).decode(errors="replace")
status_line = response.split("\r\n")[0]
if "101" not in status_line:
    raise SystemExit(f"handshake refused: {status_line}")

# Exactly what BroadcastClient sends first, and not JSON.
sock.sendall(frame(b"Hello"))
time.sleep(0.2)

for message in [state("underway"),
                log(0, "moving to site"),
                log(1, "eplansys.outcome=e-scan-dirty"),
                state("completed", "eplansys.outcome=e-scan-dirty")]:
    sock.sendall(frame(json.dumps(message).encode()))
    time.sleep(0.2)

time.sleep(0.5)
sock.close()
PY

ros2 run eplansys_rmf_probe state_probe --port "$PORT" > "$LOG" 2>&1 &
PROBE_PID=$!

for _ in $(seq 1 40); do
  grep -q "listening on" "$LOG" && break
  sleep 0.25
done

python3 "$CLIENT" localhost "$PORT"
sleep 1

echo "--- state_probe output ---"
cat "$LOG"

kill -0 "$PROBE_PID" 2>/dev/null \
  || { echo "FAIL: state_probe died, probably on the non-JSON frame"; exit 1; }

grep -q 'OUTCOME via log: "e-scan-dirty"' "$LOG" \
  || { echo "FAIL: no outcome recovered from a log entry"; exit 1; }

grep -q 'OUTCOME via detail: "e-scan-dirty"' "$LOG" \
  || { echo "FAIL: no outcome recovered from an event detail"; exit 1; }

grep -q "underway -> completed" "$LOG" \
  || { echo "FAIL: terminal status not reported"; exit 1; }

echo "PASS"
