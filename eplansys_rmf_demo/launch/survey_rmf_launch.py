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
The site survey over an Open-RMF fleet.

    ros2 launch eplansys_rmf_demo survey_rmf_launch.py
    ros2 launch eplansys_rmf_demo survey_rmf_launch.py site:=clean
    ros2 launch eplansys_rmf_demo survey_rmf_launch.py rmf:=false

The mission, the EPDDL and the policy are `eplansys_demo`'s. What this file
changes is the performers: instead of waiting out a duration, they submit RMF
tasks and wait for the fleet to report them done.

`rmf:=false` leaves the fleet to a separate terminal, which is what to use when
the office demo is already running. Launched here, the adapter is pointed at
the bridge's websocket and the rmf_demos panel is switched off, because an
adapter has one server_uri and the panel would otherwise own the port.
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
    'dirty': 'e-scan-dirty',
    'clean': 'e-scan-clean',
}

WEBSOCKET_PORT = 7879


def launch_setup(context, *args, **kwargs):
    demo = get_package_share_directory('eplansys_demo')
    here = get_package_share_directory('eplansys_rmf_demo')

    site = LaunchConfiguration('site').perform(context)
    if site not in OUTCOMES:
        raise RuntimeError(
            f'site:={site} is not one of {sorted(OUTCOMES)}. It is what the '
            'scout turns out to find.')

    domain = os.path.join(demo, 'epddl', 'survey-team.epddl')
    problem = os.path.join(demo, 'epddl', 'survey-team-problem.epddl')
    mapping = os.path.join(demo, 'pddl', 'survey-mapping.json')
    model = os.path.join(demo, 'pddl', 'survey.pddl')

    with open(os.path.join(demo, 'params', 'survey.yaml')) as handle:
        params = handle.read()
    params = (params
              .replace('EPDDL_DOMAIN', domain)
              .replace('EPDDL_PROBLEM', problem)
              .replace('MAPPING_FILE', mapping))

    filled = tempfile.NamedTemporaryFile(
        mode='w', suffix='_survey_rmf.yaml', delete=False)
    filled.write(params)
    filled.close()

    # The task map is read at start up, and site: selects what the scan falls
    # back to when the fleet reports nothing. Writing it out here keeps the
    # checked-in map free of a value that is really a launch argument.
    with open(os.path.join(here, 'config', 'office_survey.json')) as handle:
        task_map = handle.read()
    task_map = task_map.replace('"e-scan-dirty"', f'"{OUTCOMES[site]}"')

    map_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='_office_survey.json', delete=False)
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
            'task_timeout': 180.0,
        }])

    mission = Node(
        package='eplansys_demo',
        executable='survey_mission',
        name='survey_mission',
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

    finish = RegisterEventHandler(
        OnProcessExit(target_action=mission, on_exit=[EmitEvent(event=Shutdown())]))

    return [fleet, plansys2, bridge, mission, finish]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'site', default_value='dirty',
            description='What the scout turns out to find: dirty or clean.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run gazebo headless and leave rviz out.'),
        DeclareLaunchArgument(
            'rmf', default_value='true',
            description='Launch the rmf_demos office fleet too. false when it '
                        'is already running in another terminal.'),
        OpaqueFunction(function=launch_setup),
    ])
