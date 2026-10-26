"""Intera electric-gripper signal encoding and cuff edges; no ROS dependency."""

from dataclasses import dataclass
import json
import math


@dataclass(frozen=True)
class Signal:
    value: object
    kind: str
    role: str
    status: str


def read_signals(message):
    result = {}
    for signal in message.signals:
        if signal.name in result:
            raise ValueError(f"Duplicate IO signal: {signal.name}")
        formatting = json.loads(signal.format)
        data = json.loads(signal.data)
        if not isinstance(data, list) or len(data) != 1:
            # This utility handles scalar gripper/cuff signals only.
            continue
        result[signal.name] = Signal(data[0], formatting['type'], formatting.get('role', ''), signal.status.tag)
    return result


def signal_value(signals, name, kind=None):
    if name not in signals:
        raise ValueError(f"Missing signal {name}; this utility requires the standard Intera electric gripper/cuff")
    signal = signals[name]
    if signal.status != 'ready':
        raise ValueError(f"Signal {name} is {signal.status}, not ready")
    if kind is not None and signal.kind != kind:
        raise ValueError(f"Signal {name} has type {signal.kind}, expected {kind}")
    if kind == 'bool' and not isinstance(signal.value, bool):
        raise ValueError(f"Signal {name} is not a JSON boolean")
    if kind == 'float' and (isinstance(signal.value, bool) or
                            not isinstance(signal.value, (int, float)) or not math.isfinite(signal.value)):
        raise ValueError(f"Signal {name} is not a finite number")
    return signal.value


def command_args(signals, values):
    payload = {}
    for name, value in values.items():
        kind = 'bool' if isinstance(value, bool) else 'float'
        signal_value(signals, name, kind)
        if signals[name].role != 'output':
            raise ValueError(f"Signal {name} is not writable")
        if kind == 'float' and not math.isfinite(value):
            raise ValueError(f"Non-finite command for {name}")
        payload[name] = {'format': {'type': kind}, 'data': [value]}
    # Matches the SDK's SetCommand.set_signal() schema.
    return json.dumps({'signals': payload}, allow_nan=False)


def button_value(signals, name):
    """Cuff firmware can report a digital input as JSON bool or integer 0/1.

    Keep this compatibility limited to button inputs; writable gripper signals
    still use their exact advertised type. Strings and other integers are invalid.
    """
    value = signal_value(signals, name)
    kind = signals[name].kind
    if kind == 'bool' and isinstance(value, bool):
        return value
    if kind == 'int' and type(value) is int and value in (0, 1):
        return value == 1
    raise ValueError(f'Button {name} must be bool or int 0/1; received type={kind}, value={value!r}')


class CuffEdges:
    """Rising edges only; no action for an initially held button or both pressed."""

    def __init__(self, debounce=0.15):
        self.debounce = debounce
        self.previous = None
        self.last_press = {'open': -math.inf, 'close': -math.inf}

    def update(self, signals, now):
        current = {
            'close': button_value(signals, 'right_button_upper'),
            'open': button_value(signals, 'right_button_lower'),
        }
        previous, self.previous = self.previous, current
        if previous is None or all(current.values()):
            return []
        actions = []
        for action, pressed in current.items():
            if pressed and not previous[action] and now - self.last_press[action] >= self.debounce:
                self.last_press[action] = now
                actions.append(action)
        return actions
