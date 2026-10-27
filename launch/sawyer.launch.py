"""Start the Sawyer bridge, cuff-button bindings, and display lifecycle."""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def helper_exited(event, context):
    if context.is_shutdown:
        return []
    return [EmitEvent(event=Shutdown(reason=f'Cuff-button controller exited ({event.returncode})'))]


def display_exited(event, context):
    if context.is_shutdown:
        return []
    return [LogInfo(msg=f'Display manager exited ({event.returncode}); arm bridge remains running. Check display logs.')]


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
    display = ExecuteProcess(
        cmd=[sys.executable, str(project / 'scripts/display_manager.py'),
             '--image', LaunchConfiguration('display_image'),
             '--default-image', LaunchConfiguration('default_display_image')],
        output='screen', additional_env={'PYTHONUNBUFFERED': '1'},
        condition=IfCondition(LaunchConfiguration('display_manager')),
        sigterm_timeout='10', sigkill_timeout='5',
    )
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(project / 'bridge.yaml')),
        DeclareLaunchArgument('all_interfaces', default_value='true'),
        DeclareLaunchArgument('commands', default_value='true'),
        DeclareLaunchArgument('head_commands', default_value='true'),
        DeclareLaunchArgument('cuff_buttons', default_value='true',
                              description='Automatically enable gripper cuff-button bindings'),
        DeclareLaunchArgument('display_manager', default_value='true',
                              description='Show project image, clock overlay, and restore SDK image on exit'),
        DeclareLaunchArgument('display_image', default_value=str(project / 'display.png')),
        DeclareLaunchArgument('default_display_image', default_value=str(project / 'assets/default_display.png')),
        RegisterEventHandler(OnProcessExit(target_action=buttons, on_exit=helper_exited)),
        RegisterEventHandler(OnProcessExit(target_action=display, on_exit=display_exited)),
        bridge,
        buttons,
        display,
    ])
