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

"""
The two-site survey over an Open-RMF fleet.

    ros2 launch eplansys_rmf_demo survey_sites_rmf_launch.py
    ros2 launch eplansys_rmf_demo survey_sites_rmf_launch.py north:=clean south:=dirty
    ros2 launch eplansys_rmf_demo survey_sites_rmf_launch.py parallel:=false

The site survey twice over, in different hands. `north` carries the instrument
the north site reads and `south` the one the south site reads, so neither half
of the mission can be done by the other's robot and both robots drive. `relay`
is spoken to twice and never moves; `observer` is the machine the mission keeps
in the dark.

This is the mission the one-site survey cannot be: two robots crossing the same
floor at the same time, with Open-RMF deciding who yields. `parallel:=false`
dispatches the policy a node at a time instead, which is the same mission with
one robot moving and the other waiting for it.

The mission, the EPDDL and the policy are `eplansys_demo`'s. What this file
changes is the performers, which submit RMF tasks instead of waiting out a
duration.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


OUTCOMES = {
    'north': {'dirty': 'e-scan-north-dirty', 'clean': 'e-scan-north-clean'},
    'south': {'dirty': 'e-scan-south-dirty', 'clean': 'e-scan-south-clean'},
}

# One away from the one-site demo's, so that both can be up at once and neither
# adapter is pointed at the other's bridge.
WEBSOCKET_PORT = 7880

# Every agent of the mission, including the one the goal excludes. `observer`
# runs a radio precisely so that its silence is recorded rather than assumed,
# and `relay` runs two because both findings are told to it.
AGENTS = ('north', 'south', 'relay', 'observer')

CHANNEL_PREFIX = '/eplansys/channel'


def launch_setup(context, *args, **kwargs):
    demo = get_package_share_directory('eplansys_demo')
    here = get_package_share_directory('eplansys_rmf_demo')

    found = {}
    for site in OUTCOMES:
        choice = LaunchConfiguration(site).perform(context)
        if choice not in OUTCOMES[site]:
            raise RuntimeError(
                f'{site}:={choice} is not one of {sorted(OUTCOMES[site])}. It is '
                'what that site turns out to hold.')
        found[site] = OUTCOMES[site][choice]

    parallel = LaunchConfiguration('parallel').perform(context).lower()
    if parallel not in ('true', 'false'):
        raise RuntimeError(
            f'parallel:={parallel} is neither true nor false. It says whether '
            'the two halves of the policy are dispatched together.')

    domain = os.path.join(demo, 'epddl', 'survey-sites.epddl')
    problem = os.path.join(demo, 'epddl', 'survey-sites-problem.epddl')
    mapping = os.path.join(demo, 'pddl', 'survey-sites-mapping.json')
    model = os.path.join(demo, 'pddl', 'survey-sites.pddl')

    with open(os.path.join(demo, 'params', 'survey_sites.yaml')) as handle:
        params = handle.read()
    params = (params
              .replace('EPDDL_DOMAIN', domain)
              .replace('EPDDL_PROBLEM', problem)
              .replace('MAPPING_FILE', mapping)
              .replace('PARALLEL_DISPATCH', parallel))

    filled = tempfile.NamedTemporaryFile(
        mode='w', suffix='_survey_sites_rmf.yaml', delete=False)
    filled.write(params)
    filled.close()

    # The task map is read at start up, and north:/south: select what each scan
    # falls back to when the fleet reports nothing. Writing it out here keeps
    # the checked-in map free of values that are really launch arguments.
    with open(os.path.join(here, 'config', 'office_sites.json')) as handle:
        task_map = handle.read()
    for site, outcome in found.items():
        task_map = task_map.replace(f'"e-scan-{site}-dirty"', f'"{outcome}"')
        task_map = task_map.replace(f'"e-scan-{site}-clean"', f'"{outcome}"')

    map_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='_office_sites.json', delete=False)
    map_file.write(task_map)
    map_file.close()

    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': model,
            'params_file': filled.name,
            'epistemic_state': 'True',
        }.items())

    bridge = Node(
        package='eplansys_rmf_bridge',
        executable='rmf_action_node',
        name='eplansys_rmf_bridge',
        output='screen',
        parameters=[{
            'task_map': map_file.name,
            'websocket_port': WEBSOCKET_PORT,
            'task_timeout': float(LaunchConfiguration('task_timeout').perform(context)),
            'channel_prefix': CHANNEL_PREFIX,
        }])

    # One pair of ears per agent, up before anything is said. A speech act is
    # published once, and although the channels are latched so a late listener
    # still hears, a radio that starts first is the honest arrangement: the
    # transcript then records what was heard rather than what was replayed.
    radios = [
        Node(
            package='eplansys_rmf_demo',
            executable='radio',
            name=f'radio_{agent}',
            output='screen',
            parameters=[{
                'agent': agent,
                'channel_prefix': CHANNEL_PREFIX,
            }])
        for agent in AGENTS
    ]

    mission = Node(
        package='eplansys_demo',
        executable='survey_sites_mission',
        name='survey_sites_mission',
        output='screen')

    # office_fleet.launch.xml, not rmf_demos_gz_classic/office.launch.xml:
    # that one forwards neither server_uri nor use_rmf_panel down to
    # common.launch.xml, so passing them to it would quietly do nothing.
    fleet = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            here, 'launch', 'office_fleet.launch.xml')),
        condition=IfCondition(LaunchConfiguration('rmf')),
        launch_arguments={
            'use_rmf_panel': 'false',
            'headless': LaunchConfiguration('headless'),
            'server_uri': f'ws://localhost:{WEBSOCKET_PORT}',
        }.items())

    # The third conjunct of the goal is a negative one, and `observer` is bound
    # to no robot and never moves, so it is satisfied by everything that fails
    # to happen. Asking the epistemic state afterwards is what tells a run that
    # honoured the private channel apart from a run that simply did nothing.
    #
    # The window for asking is narrow: the executor has to be finished and the
    # state node has to still be alive, which is between the mission exiting
    # and the shutdown it used to emit directly.
    # Six conjuncts, three per site, and the transcripts that say who was
    # actually spoken to. Both scouts speak, to the relay and to nobody else.
    check = Node(
        package='eplansys_rmf_demo',
        executable='mission_check',
        name='mission_check',
        output='screen',
        parameters=[{
            'channel_prefix': CHANNEL_PREFIX,
            'must_hold': [
                '(Kw north contaminated-north)',
                '(Kw relay contaminated-north)',
                '(Kw south contaminated-south)',
                '(Kw relay contaminated-south)',
            ],
            'must_not_hold': [
                '(Kw observer contaminated-north)',
                '(Kw observer contaminated-south)',
            ],
            'heard_something': ['relay'],
            'heard_nothing': ['observer'],
        }])

    checking = LaunchConfiguration('check').perform(context).lower() in ('true', '1')

    staged = []
    if checking:
        staged.append(
            RegisterEventHandler(OnProcessExit(target_action=mission, on_exit=[check])))

    staged.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=check if checking else mission,
                on_exit=[EmitEvent(event=Shutdown())])))

    return [fleet, plansys2, bridge, *radios, mission, *staged]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'north', default_value='dirty',
            description='What the north site turns out to hold: dirty or clean.'),
        DeclareLaunchArgument(
            'south', default_value='clean',
            description='What the south site turns out to hold: dirty or clean.'),
        DeclareLaunchArgument(
            'parallel', default_value='true',
            description="Dispatch the policy's independent runs together, so "
                        'that both robots drive at once.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run gazebo headless and leave rviz out.'),
        DeclareLaunchArgument(
            'check', default_value='true',
            description='Ask the epistemic state, once the robots have '
                        'stopped, whether the goal actually came out.'),
        DeclareLaunchArgument(
            'task_timeout', default_value='180.0',
            description='How long the bridge waits for a task it submitted '
                        'before giving up on it. Shorten it to watch what a '
                        'fleet that stops answering does to the mission.'),
        DeclareLaunchArgument(
            'rmf', default_value='true',
            description='Launch the rmf_demos office fleet too. false when it '
                        'is already running in another terminal.'),
        OpaqueFunction(function=launch_setup),
    ])
