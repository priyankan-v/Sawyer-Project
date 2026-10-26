#!/usr/bin/env python3
"""Control the standard Intera electric gripper or map its cuff buttons in ROS 2."""

import argparse
import json
import sys
import time

import rclpy
from intera_core_msgs.msg import IOComponentCommand, IODeviceStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

if __package__:
    from .gripper_io import CuffEdges, command_args, read_signals, signal_value
else:
    from gripper_io import CuffEdges, command_args, read_signals, signal_value


GRIPPER = '/io/end_effector/right_gripper'
CUFF = '/io/robot/cuff/state'
OPEN_POSITION = 0.041667  # SDK default for the standard Intera electric gripper.


def time_key(stamp):
    return stamp.sec, stamp.nanosec


class GripperControl(Node):
    def __init__(self, options):
        super().__init__('gripper_control')
        self.options = options
        self.gripper = self.cuff = None
        self.signals = {}
        self.cuff_signals = {}
        self.cuff_seen = False
        self.last_cuff_error = None
        self.edges = CuffEdges()
        self.pending = None
        self.in_progress = False
        self.last_input_error = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(IODeviceStatus, GRIPPER + '/state', self.on_gripper, qos)
        if options.action == 'buttons':
            self.create_subscription(IODeviceStatus, CUFF, self.on_cuff, qos)
        self.publisher = self.create_publisher(IOComponentCommand, GRIPPER + '/command', 1)

    def on_gripper(self, message):
        try:
            signals = read_signals(message)
            if message.device.name != 'right_gripper':
                raise ValueError(f'Unexpected gripper device: {message.device.name}')
        except (ValueError, KeyError, TypeError) as exc:
            self.gripper = None
            self.last_input_error = str(exc)
            return
        # IODeviceStatus may update only some signals, as supported by the SDK.
        self.signals.update(signals)
        self.gripper = message

    def on_cuff(self, message):
        self.cuff_seen = True
        try:
            if message.device.status.tag != 'ready':
                raise ValueError(f'Cuff device is {message.device.status.tag}')
            self.cuff_signals.update(read_signals(message))
            if not all(name in self.cuff_signals for name in ('right_button_upper', 'right_button_lower')):
                self.last_cuff_error = 'Waiting for both right_button_upper and right_button_lower signals'
                return
            actions = self.edges.update(self.cuff_signals, time.monotonic())
        except (ValueError, KeyError, TypeError) as exc:
            self.cuff = None
            self.pending = None
            self.edges.previous = None
            self.last_input_error = str(exc)
            self.last_cuff_error = str(exc)
            return
        self.cuff = message
        self.last_cuff_error = None
        if self.options.action == 'buttons' and not self.in_progress:
            for action in actions:
                # Only the latest fresh press can be acted on, never a backlog.
                self.pending = action, time.monotonic()

    def wait_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                if callable(description):
                    raise TimeoutError(description())
                detail = f' Last IO error: {self.last_input_error}' if self.last_input_error else ''
                raise TimeoutError(f'{description}.{detail}')
            rclpy.spin_once(self, timeout_sec=0.02)

    def require_ready(self, action):
        if self.gripper is None:
            raise RuntimeError('No valid gripper state')
        if action != 'stop' and signal_value(self.signals, 'has_error', 'bool'):
            raise RuntimeError('Gripper reports an error; inspect hardware/state before commanding it')
        if action in ('open', 'close', 'buttons'):
            if not signal_value(self.signals, 'is_calibrated', 'bool'):
                raise RuntimeError('Gripper is not calibrated. With fingers clear, run: '
                                   'python scripts/gripper_control.py calibrate --execute')
            if self.gripper.device.status.tag != 'ready':
                raise RuntimeError(f'Gripper device is {self.gripper.device.status.tag}')
            if signal_value(self.signals, 'is_moving', 'bool'):
                raise RuntimeError('Gripper is moving; press again after it stops')
        elif self.gripper.device.status.tag not in ('ready', 'unready') and action != 'stop':
            raise RuntimeError(f'Gripper device is {self.gripper.device.status.tag}')

    def make_message(self, values):
        return IOComponentCommand(time=self.get_clock().now().to_msg(), op='set',
                                  args=command_args(self.signals, values))

    def send(self, values):
        message = self.make_message(values)
        key = time_key(message.time)
        deadline = time.monotonic() + self.options.timeout
        while True:
            if self.publisher.get_subscription_count() == 0:
                raise RuntimeError('No gripper command bridge subscription')
            self.publisher.publish(message)
            try:
                self.wait_for(lambda: self.gripper is not None and
                              key in [time_key(stamp) for stamp in self.gripper.commands],
                              min(0.1, max(0.001, deadline - time.monotonic())), 'Waiting for IO acknowledgement')
                index = [time_key(stamp) for stamp in self.gripper.commands].index(key)
                if index < len(self.gripper.responses) and self.gripper.responses[index]:
                    print('IO response:', self.gripper.responses[index])
                if (values != {'go': False} and self.signals.get('has_error') is not None
                        and self.signals['has_error'].status == 'ready'
                        and signal_value(self.signals, 'has_error', 'bool')):
                    raise RuntimeError('Gripper reported an error after command acknowledgement')
                return
            except TimeoutError:
                if time.monotonic() >= deadline:
                    raise TimeoutError('Gripper did not acknowledge the command; check bridge and device')

    def request_stop(self):
        """Best-effort stop after an interrupted/failed command, preserving grasp."""
        try:
            message = self.make_message({'go': False})
            for _ in range(3):
                self.publisher.publish(message)
                until = time.monotonic() + 0.05
                while time.monotonic() < until:
                    rclpy.spin_once(self, timeout_sec=0.01)
        except Exception as exc:
            self.get_logger().warning(f'Gripper stop delivery unconfirmed: {exc}')

    def perform(self, action):
        target = self.options.open_position if action == 'open' else 0.0
        values = ({'position_m': target, 'go': True} if action in ('open', 'close') else
                  {'calibrate': True} if action == 'calibrate' else {'go': False})
        # Validate encoding even for a preview; never publish without --execute.
        encoded = command_args(self.signals, values)
        if not self.options.execute:
            print(f'Preview {action}: {encoded}; add --execute to send.')
            return
        self.require_ready(action)
        self.wait_for(lambda: self.publisher.get_subscription_count() > 0, 5,
                      'No gripper command route; restart bridge with project bridge.yaml')
        self.require_ready(action)
        self.in_progress = True
        try:
            self.send(values)
            if action == 'calibrate':
                self.wait_for(lambda: self.completed(action, target), self.options.timeout,
                              'Gripper calibration did not complete')
            elif action in ('open', 'close'):
                self.wait_for(lambda: self.completed(action, target), self.options.timeout,
                              f'Gripper {action} did not complete')
            print(f'Gripper {action} confirmed.' if action != 'stop' else 'Gripper stop acknowledged.')
        except BaseException:
            self.request_stop()
            raise
        finally:
            self.in_progress = False
            self.pending = None

    def completed(self, action, target):
        if self.gripper is None:
            return False
        if self.gripper.device.status.tag in ('down', 'error'):
            raise RuntimeError(f'Gripper device is {self.gripper.device.status.tag}')
        required = (('has_error', 'is_calibrated') if action == 'calibrate' else
                    ('has_error', 'is_moving', 'position_response_m', 'is_gripping'))
        if any(name in self.signals and self.signals[name].status != 'ready' for name in required):
            return False  # A calibration/motion can temporarily mark input data busy.
        if signal_value(self.signals, 'has_error', 'bool'):
            raise RuntimeError('Gripper reports an error')
        if action == 'calibrate':
            return signal_value(self.signals, 'is_calibrated', 'bool')
        if signal_value(self.signals, 'is_moving', 'bool'):
            return False
        at_target = abs(signal_value(self.signals, 'position_response_m', 'float') - target) <= .002
        return at_target or (action == 'close' and signal_value(self.signals, 'is_gripping', 'bool'))

    def button_readiness_problem(self):
        if self.gripper is None:
            return 'waiting for gripper state from the bridge'
        if self.cuff is None:
            return ('waiting for valid cuff state: ' + self.last_cuff_error
                    if self.last_cuff_error else 'waiting for cuff state from the bridge')
        if self.options.execute:
            if self.publisher.get_subscription_count() == 0:
                return 'waiting for the gripper command bridge route'
            try:
                self.require_ready('buttons')
            except (RuntimeError, ValueError) as exc:
                return str(exc)
        return None

    def wait_for_button_readiness(self):
        """Persistent startup/recovery for launch; no automatic motion or calibration."""
        previous = None
        while True:
            problem = self.button_readiness_problem()
            if problem is None:
                self.pending = None  # Discard presses made before readiness.
                return
            self.pending = None
            if problem != previous:
                self.get_logger().info('Cuff buttons: ' + problem)
                previous = problem
            rclpy.spin_once(self, timeout_sec=.1)

    def run(self):
        persistent_buttons = (self.options.action == 'buttons' and
                              getattr(self.options, 'wait_for_ready', False))
        if persistent_buttons:
            self.wait_for_button_readiness()
        self.wait_for(lambda: self.gripper is not None, 10,
                      'No gripper state; check /io/end_effector/right_gripper/state')
        if self.options.action == 'status':
            print('Gripper device:', self.gripper.device.name, self.gripper.device.status.tag)
            print(json.dumps({name: signal.value for name, signal in self.signals.items()}, indent=2))
            return
        if self.options.action != 'buttons':
            self.perform(self.options.action)
            return
        if self.options.execute:
            self.require_ready('buttons')
        self.wait_for(lambda: self.cuff is not None, 10,
                      lambda: ('Cuff state is arriving but cannot be used: ' +
                               (self.last_cuff_error or 'no valid button state') if self.cuff_seen else
                               'No cuff state received. Check /io/robot/cuff/state and restart bridge '
                               'with project bridge.yaml (reliable + transient_local cuff state)'))
        self.pending = None  # Never act on a press received during startup.
        print('Upper/dash: close. Lower/circle: open. Ctrl+C exits.')
        print('Button control active.' if self.options.execute else 'Monitor only; no commands sent.')
        while True:
            rclpy.spin_once(self, timeout_sec=.05)
            if persistent_buttons and self.button_readiness_problem() is not None:
                self.wait_for_button_readiness()
                print('Button control active.' if self.options.execute else 'Button monitor ready.')
                continue
            if self.pending is not None:
                (action, received), self.pending = self.pending, None
                if time.monotonic() - received > .25:
                    continue
                try:
                    self.perform(action)
                except (RuntimeError, ValueError, TimeoutError) as exc:
                    self.get_logger().warning(f'Button request ignored/failed: {exc}')


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('status', 'open', 'close', 'stop', 'calibrate', 'buttons'))
    parser.add_argument('--execute', action='store_true', help='Send commands; otherwise preview/monitor only')
    parser.add_argument('--wait-for-ready', action='store_true',
                        help='For buttons: stay running while bridge/calibration/readiness is unavailable')
    parser.add_argument('--open-position', type=float, default=OPEN_POSITION,
                        help='SDK position_m open target in meters (default: 0.041667)')
    parser.add_argument('--timeout', type=float, default=10.0, help='Acknowledgement/completion timeout (s)')
    return parser


def main():
    import math
    parser = build_parser()
    options = parser.parse_args()
    if options.wait_for_ready and options.action != 'buttons':
        parser.error('--wait-for-ready is only supported for buttons')
    if not math.isfinite(options.open_position) or not 0 <= options.open_position <= OPEN_POSITION:
        parser.error('--open-position must be within [0, 0.041667] meters')
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    node = None
    try:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = GripperControl(options)
        node.run()
        return 0
    except KeyboardInterrupt:
        print('Gripper control exited. No automatic opening/release requested.')
        return 130
    except Exception as exc:
        print(f'Gripper control error: {exc}', file=sys.stderr)
        return 1
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
