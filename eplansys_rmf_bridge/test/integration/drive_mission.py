#!/usr/bin/env python3
# Copyright 2026 Haniel Ulises
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Drive the real performers from both sides, with no fleet and no planner.

What the demo shows, and what a person has had to watch it show, is a mission:
an action is dispatched, a robot is sent to a place, what it found comes back,
and a speech act reaches the audience it declared. Every one of those is a
claim about `rmf_action_node`, which CI does not build, so the claims were
checked by launching Gazebo and reading the console.

Neither end of the bridge needs to be real for the claims to be testable. On
one side the executor is a few messages on `actions_hub`; on the other a fleet
adapter is a subscriber to `task_api_requests` and a websocket client. This is
both of them, and it asserts what the README promises:

  1. a movement action goes to the robot its agent is bound to, at the
     waypoint the zones table names, and not to whichever robot RMF fancies;
  2. a sensing action finishes carrying what the fleet reported;
  3. with the fleet silent, it carries the reading for its own site, not the
     reading some other site happened to publish;
  4. a private speech act reaches its listener's channel and no other, with
     the finding the speaker sensed;
  5. an agent bound to no robot fails the action instead of guessing.

Exits non-zero on the first broken promise, naming it.
"""

import base64
import json
import os
import socket
import struct
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from plansys2_msgs.msg import ActionExecution
from rmf_fleet_msgs.msg import FleetState, RobotState
from rmf_task_msgs.msg import ApiRequest, ApiResponse
from std_msgs.msg import String

PORT = int(os.environ.get('BRIDGE_WEBSOCKET_PORT', '7891'))
TIMEOUT = float(os.environ.get('MISSION_TEST_TIMEOUT', '30'))


class Websocket:
    """A minimal RFC 6455 client, on the standard library alone.

    The same reason smoke_test.sh has one: jammy's websockets package is 9.1,
    whose client cannot run on Python 3.10 at all.
    """

    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=10)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((
            'GET / HTTP/1.1\r\n'
            f'Host: {host}:{port}\r\n'
            'Upgrade: websocket\r\n'
            'Connection: Upgrade\r\n'
            f'Sec-WebSocket-Key: {key}\r\n'
            'Sec-WebSocket-Version: 13\r\n\r\n'
        ).encode())
        response = self.sock.recv(4096).decode(errors='replace')
        if '101' not in response.split('\r\n')[0]:
            raise SystemExit(f'websocket handshake refused: {response!r}')

    def send(self, message):
        payload = json.dumps(message).encode()
        mask = os.urandom(4)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        n = len(payload)
        if n < 126:
            header = struct.pack('!BB', 0x81, 0x80 | n)
        elif n < 65536:
            header = struct.pack('!BBH', 0x81, 0x80 | 126, n)
        else:
            header = struct.pack('!BBQ', 0x81, 0x80 | 127, n)
        self.sock.sendall(header + mask + masked)

    def close(self):
        self.sock.close()


class Driver(Node):
    """The executor and the fleet, as far as the performers can tell."""

    def __init__(self):
        super().__init__('mission_driver')

        hub = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=16,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.hub_pub = self.create_publisher(ActionExecution, 'actions_hub', hub)
        self.create_subscription(ActionExecution, 'actions_hub', self.on_hub, hub)

        self.fleet_pub = self.create_publisher(FleetState, 'fleet_states', 10)
        self.response_pub = self.create_publisher(ApiResponse, 'task_api_responses', 10)
        self.create_subscription(ApiRequest, 'task_api_requests', self.on_request, 10)

        self.observation_pub = self.create_publisher(String, '/eplansys/observation', latched)
        self.create_subscription(
            String, '/eplansys/channel/public', self.on_public, latched)
        self.heard_private = {}
        for listener in ('scout', 'relay', 'observer'):
            self.create_subscription(
                String, f'/eplansys/channel/private/{listener}', self.on_private(listener),
                latched)

        self.lock = threading.Lock()
        self.responses = []      # performers offering to run the current action
        self.finishes = []       # ActionExecution FINISH messages
        self.submissions = []    # what the fleet was asked to do
        self.public = []
        self.task_counter = 0

        # A robot the fleet has never announced is a robot the bridge holds its
        # request for, so the fleet says who it has before anything is asked.
        self.create_timer(0.2, self.announce_fleet)

    # -- the fleet ---------------------------------------------------------

    def announce_fleet(self):
        state = FleetState()
        state.name = 'tinyRobot'
        for name in ('tinyRobot1', 'tinyRobot2', 'tinyRobot3'):
            robot = RobotState()
            robot.name = name
            state.robots.append(robot)
        self.fleet_pub.publish(state)

    def on_request(self, msg):
        payload = json.loads(msg.json_msg)
        with self.lock:
            self.task_counter += 1
            task_id = f'fake.task-{self.task_counter}'
            self.submissions.append({'payload': payload, 'task_id': task_id,
                                     'request_id': msg.request_id})

        response = ApiResponse()
        response.type = ApiResponse.TYPE_RESPONDING
        response.request_id = msg.request_id
        response.json_msg = json.dumps(
            {'success': True, 'state': {'booking': {'id': task_id}}})
        self.response_pub.publish(response)

    # -- the executor ------------------------------------------------------

    def on_hub(self, msg):
        with self.lock:
            if msg.type == ActionExecution.RESPONSE:
                self.responses.append(msg)
            elif msg.type == ActionExecution.FINISH:
                self.finishes.append(msg)

    def request(self, action, arguments):
        msg = ActionExecution()
        msg.type = ActionExecution.REQUEST
        msg.action = action
        msg.arguments = list(arguments)
        self.hub_pub.publish(msg)

    def confirm(self, node_id, action, arguments):
        msg = ActionExecution()
        msg.type = ActionExecution.CONFIRM
        msg.node_id = node_id
        msg.action = action
        msg.arguments = list(arguments)
        self.hub_pub.publish(msg)

    # -- the audience ------------------------------------------------------

    def on_public(self, msg):
        with self.lock:
            self.public.append(msg.data)

    def on_private(self, listener):
        def callback(msg):
            with self.lock:
                self.heard_private.setdefault(listener, []).append(msg.data)
        return callback


class Mission:
    """Runs one action at a time, the way the executor does."""

    def __init__(self, driver, executor, socket_):
        self.driver = driver
        self.executor = executor
        self.socket = socket_

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)

    def wait_for(self, predicate, what, timeout=TIMEOUT):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            value = predicate()
            if value:
                return value
        fail(f'timed out waiting for {what}')

    def dispatch(self, action, arguments):
        """REQUEST, RESPONSE, CONFIRM: the executor's half of the handshake."""
        with self.driver.lock:
            self.driver.responses.clear()
            self.driver.finishes.clear()
            # The previous action's task is not this one's: a stale entry here
            # would have the fleet completing a task nobody had asked for yet.
            self.driver.submissions.clear()

        def offered():
            with self.driver.lock:
                return next((m for m in self.driver.responses
                             if m.action == action), None)

        # The performers subscribe on their own schedule, so the request is
        # repeated until one of them answers rather than sent once into a void.
        deadline = time.monotonic() + TIMEOUT
        offer = None
        while offer is None and time.monotonic() < deadline:
            self.driver.request(action, arguments)
            self.spin(0.5)
            offer = offered()
        if offer is None:
            fail(f'no performer offered to run ({action} {" ".join(arguments)})')

        self.driver.confirm(offer.node_id, action, arguments)
        return offer.node_id

    def submission(self):
        def latest():
            with self.driver.lock:
                return self.driver.submissions[-1] if self.driver.submissions else None
        return self.wait_for(latest, 'the fleet to be asked for a task')

    def complete(self, task_id, detail=None):
        """Report the task done, carrying an outcome when there is one."""
        event = {'id': 3, 'status': 'completed', 'name': 'go to place'}
        if detail:
            event['detail'] = detail
        self.socket.send({'type': 'task_state_update', 'data': {
            'booking': {'id': task_id},
            'category': 'compose',
            'status': 'completed',
            'assigned_to': {'group': 'tinyRobot', 'name': 'tinyRobot1'},
            'phases': {'1': {'id': 1, 'events': {'3': event}}}}})

    def finished(self):
        def done():
            with self.driver.lock:
                return self.driver.finishes[-1] if self.driver.finishes else None
        return self.wait_for(done, 'the performer to finish')


def fail(message):
    print(f'FAIL: {message}', flush=True)
    sys.exit(1)


def check(condition, message):
    if not condition:
        fail(message)


def main():
    rclpy.init()
    driver = Driver()
    executor = SingleThreadedExecutor()
    executor.add_node(driver)

    # Let the fleet announce itself and the performers come up.
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)

    socket_ = Websocket('localhost', PORT)
    mission = Mission(driver, executor, socket_)
    mission.spin(1.0)

    # 1. A movement action, pinned to the agent's own robot.
    mission.dispatch('goto_site', ['scout', 'a07'])
    submitted = mission.submission()
    payload = submitted['payload']
    check(payload.get('type') == 'robot_task_request',
          f'a movement action was not pinned to a robot: {payload.get("type")}')
    check(payload.get('robot') == 'tinyRobot1',
          f'scout is bound to tinyRobot1, and the task went to {payload.get("robot")}')
    check(payload.get('fleet') == 'tinyRobot', 'the task named the wrong fleet')
    description = payload['request']['description']['phases'][0]['activity']['description']
    check(description.get('waypoint') == 'pantry',
          f'the zone a07 should have become the waypoint pantry, not '
          f'{description.get("waypoint")!r}')
    mission.complete(submitted['task_id'])
    done = mission.finished()
    check(done.success, 'a movement action that RMF completed was reported as failed')
    check(done.outcome == '',
          f'an ordinary action reported an outcome: {done.outcome!r}')
    print('ok: the task went to the robot the planner named, at the mapped waypoint')

    # 2. A sensing action carries what the fleet reported.
    mission.dispatch('scan', ['scout', 'a15'])
    submitted = mission.submission()
    mission.complete(submitted['task_id'], detail='eplansys.outcome=e-scan-dirty')
    done = mission.finished()
    check(done.success, 'a completed scan was reported as failed')
    check(done.outcome == 'e-scan-dirty',
          f'the scan should carry what the fleet reported, and carried {done.outcome!r}')
    print('ok: the outcome the fleet reported reached the executor')

    # 3. With the fleet silent, the reading for this action's own site --- not
    #    the one another site published, and not the task map's stand-in.
    for reading in ('a15 e-scan-dirty', 'a07 e-scan-clean'):
        driver.observation_pub.publish(String(data=reading))
    mission.spin(1.0)

    mission.dispatch('scan', ['scout', 'a07'])
    submitted = mission.submission()
    mission.complete(submitted['task_id'])
    done = mission.finished()
    check(done.outcome == 'e-scan-clean',
          f'the scan of a07 should report a07\'s reading e-scan-clean, and reported '
          f'{done.outcome!r}')
    print('ok: a scan reports its own site\'s reading')

    # 4. A private speech act, to its listener's channel and no other, carrying
    #    what the speaker sensed.
    mission.dispatch('relay', ['scout', 'relay'])
    done = mission.finished()
    check(done.success, 'a speech act failed')

    def heard():
        with driver.lock:
            return list(driver.heard_private.get('relay', []))
    said = mission.wait_for(heard, 'the relay to hear the private announcement')
    check(any('e-scan-clean' in utterance for utterance in said),
          f'the utterance should carry what the scout sensed: {said}')
    with driver.lock:
        check(not driver.public,
              f'a private announcement was also broadcast: {driver.public}')
        check('observer' not in driver.heard_private,
              'a private announcement reached an agent it was not addressed to')
    print('ok: the private channel reached its listener and nobody else')

    # 5. An agent bound to no robot is a hole in the map, not a guess.
    mission.dispatch('goto_site', ['observer', 'a07'])
    done = mission.finished()
    check(not done.success,
          'an action for an agent bound to no robot was reported as done')
    print('ok: an unbound agent fails the action rather than moving some other robot')

    socket_.close()
    driver.destroy_node()
    rclpy.shutdown()
    print('PASS')


if __name__ == '__main__':
    main()
