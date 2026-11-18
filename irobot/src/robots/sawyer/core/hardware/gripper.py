"""Electric gripper commands, acknowledgements and completion."""

from __future__ import annotations

import time

from ...config import vector
from .io import command_args, read_signals, signal_value, time_key


class SawyerGripper:
    def __init__(self, node, config):
        from ..communication.wire import IOComponentCommand, IODeviceStatus

        self.node, self.config = node, config
        self.message = None
        self.signals = {}
        self.error = None
        self.cancel_event = None
        node.create_subscription(
            IODeviceStatus, config.gripper_namespace + '/state', self.on_state
        )
        self.publisher = node.create_publisher(
            IOComponentCommand, config.gripper_namespace + '/command'
        )

    def on_state(self, message):
        try:
            if message.device.name != 'right_gripper':
                msg = f'Unexpected gripper device: {message.device.name}'
                raise ValueError(msg)  # noqa: TRY301
            self.signals.update(read_signals(message))
            self.message, self.error = message, None
        except (ValueError, KeyError, TypeError) as exc:
            self.message, self.error = None, str(exc)

    def wait_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not predicate():
            if self.cancel_event is not None and self.cancel_event.is_set():
                msg = 'Gripper operation cancelled'
                raise RuntimeError(msg)
            if time.monotonic() >= deadline:
                msg = f'{description}. {self.error or ""}'
                raise TimeoutError(msg)
            self.node.spin_once(timeout_sec=self.config.callback_timeout)

    def wait_for_state(self, timeout):
        self.wait_for(
            lambda: self.message is not None,
            timeout,
            'No valid gripper state from Sawyer',
        )

    def require_ready(self, action):
        if self.message is None:
            msg = 'No valid gripper state'
            raise RuntimeError(msg)
        if action == 'reboot':
            if signal_value(self.signals, 'is_moving', 'bool'):
                msg = 'Stop the gripper explicitly before rebooting'
                raise RuntimeError(msg)
            return
        if action != 'stop' and signal_value(
            self.signals, 'has_error', 'bool'
        ):
            msg = 'Gripper reports an error'
            raise RuntimeError(msg)
        if action in ('open', 'close', 'move', 'start'):
            if not signal_value(self.signals, 'is_calibrated', 'bool'):
                msg = (
                    'Gripper is not calibrated; explicitly call '
                    'calibrate_gripper(execute=True)'
                )
                raise RuntimeError(msg)
            if self.message.device.status.tag != 'ready':
                msg = f'Gripper is {self.message.device.status.tag}'
                raise RuntimeError(msg)
            if signal_value(self.signals, 'is_moving', 'bool'):
                msg = 'Gripper is already moving'
                raise RuntimeError(msg)
        elif action != 'stop' and self.message.device.status.tag not in (
            'ready',
            'unready',
        ):
            msg = f'Gripper is {self.message.device.status.tag}'
            raise RuntimeError(msg)

    def make_message(self, values):
        from ..communication.wire import IOComponentCommand

        return IOComponentCommand(
            time=self.node.get_clock().now().to_msg(),
            op='set',
            args=command_args(self.signals, values),
        )

    def send(self, values):
        message = self.make_message(values)
        key = time_key(message.time)
        deadline = time.monotonic() + self.config.gripper_timeout
        while True:
            if not self.publisher.get_subscription_count():
                msg = 'No gripper command subscriber'
                raise RuntimeError(msg)
            self.publisher.publish(message)
            try:
                self.wait_for(
                    lambda: (
                        self.message is not None
                        and key
                        in [time_key(stamp) for stamp in self.message.commands]
                    ),
                    min(
                        self.config.gripper_ack_retry_interval,
                        max(0.001, deadline - time.monotonic()),
                    ),
                    'Waiting for IO acknowledgement',
                )
                if values not in (
                    {'go': False},
                    {'reboot': True},
                ) and signal_value(self.signals, 'has_error', 'bool'):
                    msg = 'Gripper reports an error after acknowledgement'
                    raise RuntimeError(msg)
                return  # noqa: TRY300
            except TimeoutError:
                if time.monotonic() >= deadline:
                    msg = 'Gripper did not acknowledge the command'
                    raise TimeoutError(msg) from None

    def completed(self, action, target):  # noqa: PLR0911
        if self.message is None:
            return False
        if action == 'reboot':
            return (
                self.message.device.status.tag in ('ready', 'unready')
                and not signal_value(self.signals, 'is_calibrated', 'bool')
                and not signal_value(self.signals, 'is_moving', 'bool')
                and not signal_value(self.signals, 'has_error', 'bool')
            )
        if self.message.device.status.tag in ('down', 'error'):
            msg = f'Gripper is {self.message.device.status.tag}'
            raise RuntimeError(msg)
        # Calibration can report is_calibrated before the device leaves busy.
        # Wait for ready before accepting completion or the next command.
        if self.message.device.status.tag != 'ready':
            return False
        required = (
            ('has_error', 'is_calibrated', 'is_moving')
            if action == 'calibrate'
            else (
                'has_error',
                'is_calibrated',
                'is_moving',
                'position_response_m',
                'is_gripping',
            )
        )
        if any(
            name in self.signals and self.signals[name].status != 'ready'
            for name in required
        ):
            return False
        if signal_value(self.signals, 'has_error', 'bool'):
            msg = 'Gripper reports an error'
            raise RuntimeError(msg)
        if not signal_value(
            self.signals, 'is_calibrated', 'bool'
        ) or signal_value(self.signals, 'is_moving', 'bool'):
            return False
        if action == 'calibrate':
            return True
        return abs(
            signal_value(self.signals, 'position_response_m', 'float') - target
        ) <= self.config.gripper_position_tolerance or (
            action in ('close', 'move')
            and signal_value(self.signals, 'is_gripping', 'bool')
        )

    def wait_completion(self, action, target):
        stable_since = None

        def settled():
            nonlocal stable_since
            if not self.completed(action, target):
                stable_since = None
                return False
            now = time.monotonic()
            if stable_since is None:
                stable_since = now
            return (
                now - stable_since >= self.config.gripper_settle_confirmation
            )

        self.wait_for(
            settled,
            self.config.gripper_timeout,
            f'Gripper {action} did not complete',
        )

    def request_stop(self):
        try:
            message = self.make_message({'go': False})
            for _ in range(self.config.stop_hold_samples):
                self.publisher.publish(message)
                self.node.spin_until(time.monotonic() + 0.05)
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warning(
                f'Gripper stop delivery unconfirmed: {exc}'
            )

    def perform(
        self, action, *, execute=False, cancel_event=None, position=None
    ):
        if type(execute) is not bool:
            msg = 'execute must be a boolean'
            raise ValueError(msg)
        if action not in (
            'open',
            'close',
            'move',
            'calibrate',
            'stop',
            'start',
            'reboot',
        ):
            msg = 'Unknown gripper action'
            raise ValueError(msg)
        if execute and cancel_event is not None and cancel_event.is_set():
            msg = 'Gripper operation cancelled'
            raise RuntimeError(msg)
        self.wait_for_state(self.config.connect_timeout)
        target = self.config.gripper_open_position if action == 'open' else 0.0
        if action == 'move':
            target = vector((position,), 1, 'Gripper position')[0]
            if not 0 <= target <= self.config.gripper_open_position:
                msg = 'Gripper position exceeds the configured travel'
                raise ValueError(msg)
        if action == 'start':
            target = signal_value(self.signals, 'position_m', 'float')
        values = (
            {'position_m': target, 'go': True}
            if action in ('open', 'close', 'move', 'start')
            else {'calibrate': True}
            if action == 'calibrate'
            else {'reboot': True}
            if action == 'reboot'
            else {'go': False}
        )
        encoded = command_args(self.signals, values)
        if not execute:
            return {'action': action, 'executed': False, 'command': encoded}
        self.require_ready(action)
        self.wait_for(
            lambda: self.publisher.get_subscription_count() > 0,
            self.config.connect_timeout,
            'No gripper command route',
        )
        self.require_ready(action)
        try:
            self.cancel_event = cancel_event
            if cancel_event is not None and cancel_event.is_set():
                msg = 'Gripper operation cancelled'
                raise RuntimeError(msg)  # noqa: TRY301
            self.send(values)
            if action != 'stop':
                completion = action
                if action in ('move', 'start'):
                    current = signal_value(
                        self.signals, 'position_response_m', 'float'
                    )
                    completion = 'close' if target < current else 'open'
                self.wait_completion(completion, target)
        except BaseException:
            self.request_stop()
            raise
        finally:
            self.cancel_event = None
        return {'action': action, 'executed': True}
