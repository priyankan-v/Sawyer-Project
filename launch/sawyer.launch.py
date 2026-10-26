"""Start the installed Sawyer bridge and enable cuff-button bindings by default."""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def helper_exited(event, context):
    if context.is_shutdown:
        return []
    return [EmitEvent(event=Shutdown(reason=f'Cuff-button controller exited ({event.returncode})'))]


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]
    installed_launch = Path(get_package_share_directory('sawyer_bridge')) / 'launch' / 'bridge.launch.py'
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(installed_launch)),
        launch_arguments={name: LaunchConfiguration(name) for name in
                          ('config', 'all_interfaces', 'commands', 'head_commands')}.items(),
    )
    buttons = ExecuteProcess(
        cmd=[sys.executable, str(project / 'scripts' / 'gripper_control.py'),
             'buttons', '--execute', '--wait-for-ready'],
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        condition=IfCondition(LaunchConfiguration('cuff_buttons')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(project / 'bridge.yaml')),
        DeclareLaunchArgument('all_interfaces', default_value='true'),
        DeclareLaunchArgument('commands', default_value='true'),
        DeclareLaunchArgument('head_commands', default_value='true'),
        DeclareLaunchArgument('cuff_buttons', default_value='true',
                              description='Automatically enable gripper cuff-button bindings'),
        RegisterEventHandler(OnProcessExit(target_action=buttons, on_exit=helper_exited)),
        bridge,
        buttons,
    ])
