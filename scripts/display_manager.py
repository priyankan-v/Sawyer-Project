#!/usr/bin/env python3
"""Own Sawyer's display for one bridge session, with a leased clock overlay."""

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import String

if __package__:
    from .display_images import image_message, screen_image
    from .display_state import (CLOCK_TOPIC, DISPLAY_TOPIC, STATUS_TOPIC, RELEASE_TOPIC,
                                DisplayPolicy, parse_clock_tag)
    from .ros1_display import BridgeProbe, NativeImagePublisher
else:
    from display_images import image_message, screen_image
    from display_state import (CLOCK_TOPIC, DISPLAY_TOPIC, STATUS_TOPIC, RELEASE_TOPIC,
                               DisplayPolicy, parse_clock_tag)
    from ros1_display import BridgeProbe, NativeImagePublisher


PROJECT = Path(__file__).resolve().parents[1]


def status_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class DisplayManager(Node):
    def __init__(self, image, default_image, lease=2.5, bridge_node='/sawyer_bridge'):
        super().__init__('sawyer_display_manager')
        self.image, self.default_image = image, default_image
        self.policy = DisplayPolicy(lease)
        self.publisher = self.create_publisher(Image, DISPLAY_TOPIC, 1)
        self.status_publisher = self.create_publisher(String, STATUS_TOPIC, status_qos())
        self.create_subscription(Image, CLOCK_TOPIC, self.on_clock, 1)
        self.create_subscription(String, RELEASE_TOPIC, self.on_release, 10)
        self.ever_active, self.finished = False, False
        self.last_key, self.last_status, self.last_subscribers = None, -math.inf, 0
        self.last_publish = -math.inf
        master = os.environ.get('ROS_MASTER_URI')
        if not master:
            raise ValueError('ROS_MASTER_URI is unset; source scripts/env.sh')
        self.probe = BridgeProbe(master, bridge_node)
        self.timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self.tick, clock=self.timer_clock)
        self.get_logger().info('Waiting for bridge; display.png will be shown when its display route is ready')

    def on_clock(self, message):
        try:
            session, owner = parse_clock_tag(message.header.frame_id)
            if ((message.height, message.width, message.step) != (600, 1024, 3072)
                    or message.encoding != 'bgr8' or len(message.data) != 600*3072):
                return
            self.policy.accept_clock(session, owner, message, time.monotonic())
        except ValueError:
            return

    def on_release(self, message):
        try:
            self.policy.release_clock(*parse_clock_tag(message.data))
        except ValueError:
            return

    def publish_status(self, now):
        self.status_publisher.publish(String(data=json.dumps({
            'session': self.policy.session, 'active': self.policy.bridge_active})))
        self.last_status = now

    def tick(self):
        now = time.monotonic()
        seen, alive, detail = self.probe.status
        if seen and not alive:
            self.get_logger().info(f'Bridge stopped: {detail}; restoring original SDK screen')
            self.policy.set_bridge(False)
            self.publish_status(now)
            self.finished = True
            return
        subscribers = self.publisher.get_subscription_count()
        active = seen and alive and subscribers > 0
        changed = active != self.policy.bridge_active
        self.policy.set_bridge(active)
        if changed or now-self.last_status >= 1:
            self.publish_status(now)
        if active:
            if not self.ever_active:
                self.get_logger().info('Display ready: project image; run head_clock.py to show the clock')
            self.ever_active = True
            mode, frame = self.policy.selection(now)
            key = (mode, id(frame))
            # ROS 1 subscribers can reconnect without changing the ROS 2 graph.
            # Refresh static images so late SDK connections also get a frame.
            refresh = mode == 'image' and now-self.last_publish >= 2
            if key != self.last_key or subscribers != self.last_subscribers or refresh:
                message = frame if mode == 'clock' else self.image
                # Private ownership metadata must not leak into the robot frame.
                original = message.header.frame_id
                message.header.frame_id = ''
                self.publisher.publish(message)
                message.header.frame_id = original
                self.last_key = key
                self.last_publish = now
        else:
            self.last_key = None
        self.last_subscribers = subscribers

    def restore_default(self):
        """Remove the ROS 2 writer first, then restore through ROS 1 directly."""
        self.policy.set_bridge(False)
        self.publish_status(time.monotonic())
        self.destroy_publisher(self.publisher)
        self.probe.close()
        if not self.ever_active:
            return
        # Let any already queued bridge frame drain before the final stock image.
        time.sleep(.2)
        publisher = NativeImagePublisher()
        try:
            count = publisher.publish(self.default_image, wait=3)
            time.sleep(.15)
            count = max(count, publisher.publish(self.default_image, wait=.5))
            if not count:
                raise RuntimeError('No ROS 1 head-display subscriber connected')
            self.get_logger().info('Original Sawyer SDK screen restored')
        finally:
            publisher.close()


def acquire_lock():
    identity = f"{os.getuid()}:{os.environ.get('ROS_MASTER_URI')}:{os.environ.get('ROS_DOMAIN_ID')}"
    key = hashlib.sha256(identity.encode()).hexdigest()[:16]
    handle = open(f'/tmp/sawyer-display-{key}.lock', 'a')
    deadline = time.monotonic()+5
    while True:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return handle
        except BlockingIOError:
            if time.monotonic() >= deadline:
                handle.close()
                raise RuntimeError('A display manager is already running for this bridge')
            time.sleep(.1)


def interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, default=PROJECT / 'display.png')
    parser.add_argument('--default-image', type=Path, default=PROJECT / 'assets/default_display.png')
    parser.add_argument('--bridge-node', default='/sawyer_bridge')
    parser.add_argument('--clock-lease', type=float, default=2.5)
    parser.add_argument('--duration', type=float, help='Optional display-manager running time in seconds')
    args = parser.parse_args()
    for name, value in [('clock-lease', args.clock_lease), ('duration', args.duration)]:
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error(f'--{name} must be positive and finite')
    node, lock = None, None
    old_term = signal.signal(signal.SIGTERM, interrupt)
    result = 0
    try:
        image = image_message(screen_image(args.image))
        default_image = image_message(screen_image(args.default_image, (0, 0, 0)))
        lock = acquire_lock()
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = DisplayManager(image, default_image, args.clock_lease, args.bridge_node)
        deadline = math.inf if args.duration is None else time.monotonic()+args.duration
        while not node.finished and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Display manager error: {exc}', file=sys.stderr)
        result = 1
    finally:
        # A second termination signal must not interrupt the bounded restoration.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node:
            try:
                node.restore_default()
            except Exception as exc:
                print(f'Could not restore original SDK screen: {exc}', file=sys.stderr)
                result = 1
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if lock:
            lock.close()
        signal.signal(signal.SIGTERM, old_term)
        signal.signal(signal.SIGINT, signal.default_int_handler)
    return result


if __name__ == '__main__':
    sys.exit(main())
