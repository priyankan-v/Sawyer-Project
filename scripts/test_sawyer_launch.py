"""Inspect launch defaults and lifecycle without starting ROS or child processes."""

import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription
from launch.utilities import perform_substitutions


PROJECT = Path(__file__).resolve().parents[1]


class LaunchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.logs = tempfile.TemporaryDirectory(prefix='sawyer-launch-tests-')
        cls.log_environment = patch.dict('os.environ', {'ROS_LOG_DIR': cls.logs.name})
        cls.log_environment.start()
        spec = importlib.util.spec_from_file_location('project_sawyer_launch', PROJECT / 'launch/sawyer.launch.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    @classmethod
    def tearDownClass(cls):
        cls.log_environment.stop()
        cls.logs.cleanup()

    def description(self):
        description = self.module.generate_launch_description()
        context = LaunchContext()
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
        return description, context

    def test_project_defaults_enable_bridge_commands_and_buttons(self):
        _, context = self.description()
        self.assertEqual(context.launch_configurations['config'], str(PROJECT / 'bridge.yaml'))
        for name in ('commands', 'head_commands', 'all_interfaces', 'cuff_buttons'):
            self.assertEqual(context.launch_configurations[name], 'true')

    def test_helper_is_active_persistent_and_can_be_disabled(self):
        description, context = self.description()
        helpers = [action for action in description.entities if isinstance(action, ExecuteProcess)]
        self.assertEqual(len(helpers), 1)
        helper = helpers[0]
        command = [perform_substitutions(context, part) for part in helper.cmd]
        self.assertEqual(command[1], str(PROJECT / 'scripts/gripper_control.py'))
        self.assertEqual(command[2:], ['buttons', '--execute', '--wait-for-ready'])
        self.assertTrue(helper.condition.evaluate(context))
        context.launch_configurations['cuff_buttons'] = 'false'
        self.assertFalse(helper.condition.evaluate(context))

    def test_installed_bridge_receives_selected_arguments(self):
        description, context = self.description()
        bridge = next(action for action in description.entities if isinstance(action, IncludeLaunchDescription))
        arguments = {name: value.perform(context) for name, value in bridge.launch_arguments}
        self.assertEqual(arguments['config'], str(PROJECT / 'bridge.yaml'))
        self.assertEqual(arguments['commands'], 'true')
        self.assertEqual(arguments['head_commands'], 'true')

    def test_helper_exit_requests_shutdown_unless_already_shutting_down(self):
        event = SimpleNamespace(returncode=1)
        actions = self.module.helper_exited(event, SimpleNamespace(is_shutdown=False))
        self.assertEqual(len(actions), 1)
        self.assertIsInstance(actions[0], EmitEvent)
        self.assertEqual(self.module.helper_exited(event, SimpleNamespace(is_shutdown=True)), [])
