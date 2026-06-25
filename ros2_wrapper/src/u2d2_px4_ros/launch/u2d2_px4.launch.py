#!/usr/bin/env python3
"""Launch the u2d2_px4_ros relay (PX4 RC -> /u2d2/cmd_abs).

This node owns no hardware; launch u2d2_driver_ros separately to drive the motor.
Topics are under the `namespace` (default `u2d2`), so it publishes /u2d2/cmd_abs,
which the u2d2_driver_ros node subscribes to.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARAMS = [
    {'name': 'namespace',        'default': 'u2d2',               'description': 'namespace / topic prefix'},
    {'name': 'rc_topic',         'default': '/fmu/out/input_rc',  'description': 'px4_msgs/InputRc topic'},
    {'name': 'cmd_topic',        'default': 'cmd_abs',            'description': 'absolute-position topic to publish (deg)'},
    {'name': 'rc_channel',       'default': '10',                 'description': '0-based RC channel index (10 = ch 11)'},
    {'name': 'rc_min',           'default': '982',                'description': 'RC pulse mapped to pos_min'},
    {'name': 'rc_max',           'default': '2006',               'description': 'RC pulse mapped to pos_max'},
    {'name': 'pos_min',          'default': '0.0',                'description': 'logical deg at rc_min'},
    {'name': 'pos_max',          'default': '700.0',              'description': 'logical deg at rc_max'},
    {'name': 'update_rate',      'default': '20.0',               'description': 'control loop Hz'},
    {'name': 'pos_deadband_deg', 'default': '2.0',                'description': 'min target change before a move'},
    {'name': 'failsafe_hold',    'default': 'true',               'description': 'hold position on rc_lost/failsafe'},
    {'name': 'log_level',        'default': 'info',               'description': 'rclpy log level'},
]

_TYPES = {'rc_channel': int, 'rc_min': int, 'rc_max': int, 'pos_min': float,
          'pos_max': float, 'update_rate': float, 'pos_deadband_deg': float,
          'failsafe_hold': bool}
_LAUNCH_ONLY = {'namespace', 'log_level'}


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

    params = {}
    for p in PARAMS:
        name = p['name']
        if name in _LAUNCH_ONLY:
            continue
        val = LaunchConfiguration(name).perform(context)
        if val in ('', "''"):
            continue  # let the node's declared default apply
        params[name] = _parse(name, val)

    return [Node(
        package='u2d2_px4_ros',
        executable='u2d2_px4_node',
        name='u2d2_px4_node',
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
