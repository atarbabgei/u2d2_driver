#!/usr/bin/env python3
"""Launch the u2d2_driver_ros node.

Topics are published/subscribed under the `namespace` (default `u2d2`), e.g.
/u2d2/cmd_turn, /u2d2/cmd_velocity, /u2d2/cmd_position, /u2d2/joint_states.
"""

import ast

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARAMS = [
    {'name': 'namespace',           'default': 'u2d2',  'description': 'namespace / topic prefix'},
    {'name': 'port',                'default': "''",    'description': "serial port ('' = autodetect)"},
    {'name': 'baud',                'default': '57600', 'description': 'baud rate (0 = auto-scan common bauds)'},
    {'name': 'ids',                 'default': '[1]',   'description': 'motor IDs, e.g. [1,2]'},
    {'name': 'profile_vel',         'default': '0.0',   'description': 'position travel speed rpm (0 = max)'},
    {'name': 'publish_rate',        'default': '20.0',  'description': 'joint_states rate Hz (0 = off)'},
    {'name': 'release_on_shutdown', 'default': 'true',  'description': 'release torque on exit'},
    {'name': 'log_level',           'default': 'info',  'description': 'rclpy log level'},
]

_TYPES = {'baud': int, 'profile_vel': float, 'publish_rate': float,
          'release_on_shutdown': bool}
_LAUNCH_ONLY = {'namespace', 'log_level', 'ids'}


def _parse(name: str, val: str):
    t = _TYPES.get(name)
    if t is bool:
        return val.lower() in ('true', '1', 'yes')
    if t is int:
        return int(val)
    if t is float:
        return float(val)
    return val


def _setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)

    ids_str = LaunchConfiguration('ids').perform(context)
    try:
        ids = [int(x) for x in ast.literal_eval(ids_str)]
    except (ValueError, SyntaxError):
        ids = [1]

    params = {'ids': ids}
    for p in PARAMS:
        name = p['name']
        if name in _LAUNCH_ONLY:
            continue
        val = LaunchConfiguration(name).perform(context)
        if val in ('', "''"):
            continue  # let the node's declared default apply
        params[name] = _parse(name, val)

    return [Node(
        package='u2d2_driver_ros',
        executable='u2d2_node',
        name='u2d2_node',
        namespace=namespace,
        parameters=[params],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(p['name'], default_value=p['default'],
                               description=p['description']) for p in PARAMS]
        + [OpaqueFunction(function=_setup)]
    )
