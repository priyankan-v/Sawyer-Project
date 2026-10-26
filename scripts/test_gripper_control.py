"""Offline gripper/cuff encoding and execution-path tests; never initialize ROS."""

import json
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from builtin_interfaces.msg import Time
from intera_core_msgs.msg import IOComponentStatus, IODataStatus, IODeviceStatus, IOStatus

import gripper_control as control
from gripper_io import CuffEdges, Signal, button_value, command_args, read_signals, signal_value


def signal(name, value, role='input', kind=None):
    kind = kind or ('bool' if isinstance(value, bool) else 'float')
    return IODataStatus(name=name, format=json.dumps({'role': role, 'type': kind}),
                        data=json.dumps([value]), status=IOStatus(tag='ready'))


def gripper(calibrated=True, status='ready'):
    return IODeviceStatus(
        device=IOComponentStatus(name='right_gripper', status=IOStatus(tag=status)),
        signals=[signal('has_error', False), signal('is_calibrated', calibrated),
                 signal('is_moving', False), signal('is_gripping', False),
                 signal('position_response_m', 0.0), signal('position_m', 0.0, 'output'),
                 signal('go', True, 'output'), signal('calibrate', False, 'output')],
    )


def cuff(upper=False, lower=False):
    return IODeviceStatus(
        device=IOComponentStatus(name='cuff', status=IOStatus(tag='ready')),
        signals=[signal('right_button_upper', upper), signal('right_button_lower', lower)],
    )


def node(action='open', execute=False, calibrated=True):
    item = control.GripperControl.__new__(control.GripperControl)
    item.options = SimpleNamespace(action=action, execute=execute, open_position=.041667, timeout=.01)
    item.gripper = item.cuff = None
    item.signals, item.cuff_signals = {}, {}
    item.cuff_seen, item.last_cuff_error = False, None
    item.edges, item.pending, item.in_progress = CuffEdges(), None, False
    item.last_input_error = None
    item.publisher = MagicMock()
    item.publisher.get_subscription_count.return_value = 1
    item.get_clock = MagicMock()
    item.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=123, nanosec=456)
    item.get_logger = MagicMock()
    item.on_gripper(gripper(calibrated, 'ready' if calibrated else 'unready'))
    return item


class IOTests(unittest.TestCase):
    def test_sdk_set_command_shape(self):
        signals = read_signals(gripper())
        payload = json.loads(command_args(signals, {'position_m': .041667, 'go': True}))
        self.assertEqual(payload, {'signals': {
            'position_m': {'format': {'type': 'float'}, 'data': [.041667]},
            'go': {'format': {'type': 'bool'}, 'data': [True]},
        }})

    def test_rejects_read_only_wrong_type_unready_and_missing_signals(self):
        signals = read_signals(gripper())
        for values in ({'has_error': True}, {'position_m': True}, {'unknown': 1.0},
                       {'position_m': float('nan')}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                command_args(signals, values)
        signals['go'] = Signal(True, 'bool', 'output', 'unready')
        with self.assertRaises(ValueError):
            command_args(signals, {'go': False})
        with self.assertRaises(ValueError):
            signal_value({'go': Signal('false', 'bool', 'input', 'ready')}, 'go', 'bool')

    def test_button_mapping_edges_release_and_held_start(self):
        edges = CuffEdges()
        self.assertEqual(edges.update(read_signals(cuff(True, False)), 1), [])
        self.assertEqual(edges.update(read_signals(cuff()), 1.1), [])
        self.assertEqual(edges.update(read_signals(cuff(True, False)), 1.2), ['close'])
        self.assertEqual(edges.update(read_signals(cuff(True, False)), 1.3), [])
        edges.update(read_signals(cuff()), 1.4)
        self.assertEqual(edges.update(read_signals(cuff(False, True)), 1.5), ['open'])

    def test_button_debounce_and_simultaneous_press(self):
        edges = CuffEdges()
        edges.update(read_signals(cuff()), 1)
        self.assertEqual(edges.update(read_signals(cuff(True, True)), 1.1), [])
        edges.update(read_signals(cuff()), 1.2)
        self.assertEqual(edges.update(read_signals(cuff(True)), 1.3), ['close'])
        edges.update(read_signals(cuff()), 1.31)
        self.assertEqual(edges.update(read_signals(cuff(True)), 1.32), [])
        edges.update(read_signals(cuff()), 1.5)
        self.assertEqual(edges.update(read_signals(cuff(True)), 1.6), ['close'])

    def test_integer_cuff_buttons_match_boolean_edges(self):
        item = node('buttons')
        def integer_cuff(upper, lower):
            message = cuff()
            message.signals = [signal('right_button_upper', upper, kind='int'),
                               signal('right_button_lower', lower, kind='int')]
            return message
        item.on_cuff(integer_cuff(0, 0))
        self.assertIsNotNone(item.cuff)
        self.assertIsNone(item.pending)
        item.on_cuff(integer_cuff(1, 0))
        self.assertEqual(item.pending[0], 'close')
        item.pending = None
        item.on_cuff(integer_cuff(1, 0))
        self.assertIsNone(item.pending)
        item.on_cuff(integer_cuff(0, 0))
        item.on_cuff(integer_cuff(0, 1))
        self.assertEqual(item.pending[0], 'open')
        self.assertIsNone(item.last_cuff_error)

    def test_button_decoder_rejects_non_binary_and_malformed_types(self):
        for kind, value in (('int', 2), ('int', -1), ('int', 1.0), ('int', True),
                            ('string', 'false'), ('bool', 'false'), ('float', 1.0)):
            with self.subTest(kind=kind, value=value), self.assertRaises(ValueError):
                button_value({'button': Signal(value, kind, 'input', 'ready')}, 'button')
        for kind, value, expected in (('int', 0, False), ('int', 1, True),
                                       ('bool', False, False), ('bool', True, True)):
            self.assertIs(button_value({'button': Signal(value, kind, 'input', 'ready')}, 'button'), expected)

    def test_boolean_command_encoding_remains_strict(self):
        signals = read_signals(gripper())
        signals['go'] = Signal(1, 'int', 'output', 'ready')
        with self.assertRaises(ValueError):
            command_args(signals, {'go': True})

    def test_unusable_cuff_state_is_not_reported_as_missing_transport(self):
        item = node('buttons', execute=True)
        invalid = cuff()
        invalid.signals[0] = signal('right_button_upper', 2, kind='int')
        item.on_cuff(invalid)
        self.assertTrue(item.cuff_seen)
        self.assertIn('int 0/1', item.last_cuff_error)
        now = [10.0]
        def spin(*args, **kwargs):
            now[0] += 1.0
        with patch.object(control.time, 'monotonic', side_effect=lambda: now[0]), patch.object(control.rclpy, 'spin_once', side_effect=spin):
            with self.assertRaisesRegex(TimeoutError, 'Cuff state is arriving but cannot be used'):
                item.run()
        item.publisher.publish.assert_not_called()

    def test_preview_and_status_never_publish(self):
        for action in ('open', 'close', 'calibrate', 'stop', 'status'):
            item = node(action, execute=False, calibrated=False)
            with patch('builtins.print'):
                item.run()
            item.publisher.publish.assert_not_called()

    def test_uncalibrated_gripper_cannot_open_or_enable_button_control(self):
        for action in ('open', 'close', 'buttons'):
            item = node(action, execute=True, calibrated=False)
            with self.subTest(action=action), self.assertRaisesRegex(RuntimeError, 'not calibrated'):
                item.run()
            item.publisher.publish.assert_not_called()

    def test_success_checks_command_ack_and_measured_completion(self):
        for action in ('open', 'close', 'calibrate', 'stop'):
            item = node(action, execute=True, calibrated=action != 'calibrate')
            sent = []
            def acknowledge(message):
                sent.append(message)
                state = gripper()
                state.commands = [message.time]
                values = json.loads(message.args)['signals']
                if 'position_m' in values:
                    state.signals.append(signal('unused', 0.0))
                    for entry in state.signals:
                        if entry.name == 'position_response_m':
                            entry.data = json.dumps(values['position_m']['data'])
                item.on_gripper(state)
            item.publisher.publish.side_effect = acknowledge
            with patch('builtins.print'):
                item.perform(action)
            self.assertEqual(len(sent), 1)
            self.assertEqual(sent[0].op, 'set')
            self.assertNotEqual(control.time_key(sent[0].time), (0, 0))
            self.assertFalse(item.in_progress)

    def test_ack_retry_keeps_identical_timestamp(self):
        item = node(execute=True)
        sent = []
        def acknowledge_third(message):
            sent.append(control.time_key(message.time))
            if len(sent) == 3:
                item.gripper.commands = [message.time]
        def wait(predicate, *args):
            if not predicate():
                raise TimeoutError()
        item.publisher.publish.side_effect = acknowledge_third
        item.wait_for = wait
        item.send({'go': True})
        self.assertEqual(len(sent), 3)
        self.assertEqual(len(set(sent)), 1)

    def test_command_timeout_requests_stop(self):
        item = node(execute=True)
        now = [10.0]
        def spin(*args, **kwargs):
            now[0] += .02
        with patch.object(control.time, 'monotonic', side_effect=lambda: now[0]), patch.object(control.rclpy, 'spin_once', side_effect=spin):
            with self.assertRaisesRegex(TimeoutError, 'acknowledge'):
                item.perform('open')
        payloads = [json.loads(call.args[0].args) for call in item.publisher.publish.call_args_list]
        self.assertEqual(payloads[-1], {'signals': {'go': {'format': {'type': 'bool'}, 'data': [False]}}})
        self.assertFalse(item.in_progress)

    def test_close_can_complete_on_detected_object(self):
        item = node('close')
        item.signals['position_response_m'] = Signal(.01, 'float', 'input', 'ready')
        item.signals['is_gripping'] = Signal(True, 'bool', 'input', 'ready')
        self.assertTrue(item.completed('close', 0))
        self.assertFalse(item.completed('open', .041667))

    def test_partial_gripper_and_cuff_updates_are_merged(self):
        item = node('buttons')
        partial = IODeviceStatus(device=gripper().device, signals=[signal('position_response_m', .02)])
        item.on_gripper(partial)
        self.assertTrue(item.signals['is_calibrated'].value)
        self.assertEqual(item.signals['position_response_m'].value, .02)
        item.on_cuff(cuff())
        press = cuff(True)
        press.signals = press.signals[:1]
        item.on_cuff(press)
        self.assertEqual(item.pending[0], 'close')

    def test_presses_while_command_in_progress_are_not_queued(self):
        item = node('buttons')
        item.on_cuff(cuff())
        item.in_progress = True
        item.on_cuff(cuff(True))
        self.assertIsNone(item.pending)
        item.in_progress = False
        item.on_cuff(cuff(True))
        self.assertIsNone(item.pending)

    def test_faulted_and_moving_grippers_reject_commands(self):
        for name in ('has_error', 'is_moving'):
            item = node(execute=True)
            item.signals[name] = Signal(True, 'bool', 'input', 'ready')
            with self.subTest(name=name), self.assertRaises(RuntimeError):
                item.perform('open')
            item.publisher.publish.assert_not_called()

    def test_busy_calibration_feedback_waits_instead_of_aborting(self):
        item = node('calibrate', calibrated=False)
        item.signals['is_calibrated'] = Signal(False, 'bool', 'input', 'busy')
        self.assertFalse(item.completed('calibrate', 0))
        item.signals['is_calibrated'] = Signal(True, 'bool', 'input', 'ready')
        self.assertTrue(item.completed('calibrate', 0))

    def test_stop_is_allowed_even_when_gripper_reports_error(self):
        item = node('stop', execute=True)
        item.signals['has_error'] = Signal(True, 'bool', 'input', 'ready')
        item.require_ready('stop')

    def test_persistent_startup_waits_and_discards_presses_before_readiness(self):
        item = node('buttons', execute=True, calibrated=False)
        item.options.wait_for_ready = True
        count = [0]
        def startup(*args, **kwargs):
            count[0] += 1
            if count[0] == 1:
                item.on_cuff(cuff())
            elif count[0] == 2:
                item.on_cuff(cuff(True))  # Pressed while still waiting for calibration.
                item.on_gripper(gripper(calibrated=True))
            else:
                raise KeyboardInterrupt()
        with patch.object(control.rclpy, 'spin_once', side_effect=startup), patch('builtins.print'):
            with self.assertRaises(KeyboardInterrupt):
                item.run()
        self.assertEqual(count[0], 3)
        self.assertIsNone(item.pending)
        item.publisher.publish.assert_not_called()

    def test_persistent_wait_does_not_automatically_calibrate(self):
        item = node('buttons', execute=True, calibrated=False)
        item.options.wait_for_ready = True
        item.on_cuff(cuff())
        with patch.object(control.rclpy, 'spin_once', side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                item.run()
        self.assertIn('not calibrated', item.get_logger.return_value.info.call_args.args[0])
        item.publisher.publish.assert_not_called()

    def test_persistent_wait_recovers_missing_command_route(self):
        item = node('buttons', execute=True)
        item.on_cuff(cuff())
        item.publisher.get_subscription_count.return_value = 0
        def connect(*args, **kwargs):
            item.publisher.get_subscription_count.return_value = 1
        with patch.object(control.rclpy, 'spin_once', side_effect=connect):
            item.wait_for_button_readiness()
        self.assertIsNone(item.button_readiness_problem())
        item.publisher.publish.assert_not_called()

    def test_persistent_buttons_resume_after_gripper_readiness_returns(self):
        item = node('buttons', execute=True)
        item.options.wait_for_ready = True
        item.on_cuff(cuff())
        count = [0]
        def reconnect(*args, **kwargs):
            count[0] += 1
            if count[0] == 1:
                item.on_gripper(gripper(calibrated=False))
            elif count[0] == 2:
                item.on_cuff(cuff(True))
                item.on_gripper(gripper(calibrated=True))
            else:
                raise KeyboardInterrupt()
        with patch.object(control.rclpy, 'spin_once', side_effect=reconnect), patch('builtins.print'):
            with self.assertRaises(KeyboardInterrupt):
                item.run()
        self.assertIsNone(item.pending)
        item.publisher.publish.assert_not_called()

    def test_wait_for_ready_flag_is_only_for_buttons(self):
        with patch('sys.argv', ['gripper_control.py', 'open', '--wait-for-ready']), patch('sys.stderr'):
            with patch.object(control.rclpy, 'init') as initialize, self.assertRaises(SystemExit):
                control.main()
        initialize.assert_not_called()


if __name__ == '__main__':
    unittest.main()
