# ROS launch files conventionally use the .launch.py suffix.
# ruff: noqa: N999
"""Start one installed Sawyer bridge with the packaged routes; no UI helpers."""

from __future__ import annotations

import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from irobot.src.robots.sawyer.config import SawyerConfig
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def start(context):
    config = LaunchConfiguration('config').perform(context)
    actions = []
    if not config:
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='irobot-sawyer-', delete=False) as stream:
            yaml.safe_dump(SawyerConfig().bridge_configuration(), stream, sort_keys=False)
            config = stream.name

        def cleanup(_event, _context):
            if Path(config).exists():
                Path(config).unlink()
            return []

        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=cleanup)))
    installed = Path(get_package_share_directory('sawyer_bridge')) / 'launch/bridge.launch.py'
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(installed)),
            launch_arguments={
                'config': config,
                'commands': LaunchConfiguration('commands'),
                'head_commands': 'false',
                'all_interfaces': 'false',
            }.items(),
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config', default_value='', description='Optional override; default generated from SawyerConfig'
            ),
            DeclareLaunchArgument('commands', default_value='false'),
            OpaqueFunction(function=start),
        ]
    )
