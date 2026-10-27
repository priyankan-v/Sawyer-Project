#!/usr/bin/env python3
"""Show a live local-time clock on Sawyer's head display through the ROS bridge."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
import signal
import sys
import time
import uuid
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import String

if __package__:
    from .clock_display import HEIGHT, WIDTH, render_clock, write_png
    from .display_images import image_message
    from .display_state import CLOCK_TOPIC, STATUS_TOPIC, RELEASE_TOPIC, clock_tag
else:
    from clock_display import HEIGHT, WIDTH, render_clock, write_png
    from display_images import image_message
    from display_state import CLOCK_TOPIC, STATUS_TOPIC, RELEASE_TOPIC, clock_tag


class HeadClock(Node):
    def __init__(self, zone):
        super().__init__('sawyer_head_clock')
        self.zone = zone
        self.publisher = self.create_publisher(
            Image, CLOCK_TOPIC, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.release_publisher = self.create_publisher(String, RELEASE_TOPIC, 10)
        self.create_subscription(String, STATUS_TOPIC, self.on_status,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.owner, self.session = uuid.uuid4().hex, None
        self.active, self.finished, self.last_status = False, False, 0.0
        self.last_second = None
        self.waiting = False
        self.frames = 0
        # Wall time supplies the displayed clock; steady time schedules updates
        # even if an application elsewhere uses /clock or changes ROS sim time.
        self.timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.1, self.tick, clock=self.timer_clock)
        self.get_logger().info(f'Head clock: {zone.key}; PID {os.getpid()}; Ctrl+C stops updates')

    def on_status(self, message):
        try:
            status = json.loads(message.data)
            session, active = status['session'], status['active']
            if not isinstance(session, str) or not session or not isinstance(active, bool):
                return
        except (ValueError, KeyError, TypeError):
            return
        if self.session is not None and (session != self.session or not active):
            self.finished = True
            self.active = False
            return
        if active:
            self.session = session
        self.active, self.last_status = active, time.monotonic()

    def release(self):
        if self.session:
            self.release_publisher.publish(String(data=clock_tag(self.session, self.owner)))
            # Give the small ownership message a chance to reach the manager.
            self.release_publisher.wait_for_all_acked(Duration(seconds=.3))

    def tick(self):
        if self.session and time.monotonic()-self.last_status > 4:
            self.active, self.finished = False, True
        if not self.active or self.publisher.get_subscription_count() == 0:
            if not self.waiting:
                self.get_logger().info('Waiting for the display manager; start launch/sawyer.launch.py')
                self.waiting = True
            self.last_second = None
            return
        self.waiting = False
        instant = datetime.now(timezone.utc)
        second = int(instant.timestamp())
        if second == self.last_second:
            return
        message = image_message(render_clock(instant, self.zone))
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = clock_tag(self.session, self.owner)
        self.publisher.publish(message)
        self.last_second = second
        self.frames += 1
        if self.frames == 1:
            self.get_logger().info('Live clock frames are publishing to Sawyer head display')


def interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timezone', default='America/Chicago', help='IANA timezone (default: America/Chicago)')
    parser.add_argument('--preview', metavar='FILE.png', help='Write one PNG preview and exit without connecting to ROS')
    parser.add_argument('--duration', type=float, help='Optional running time in seconds; default: until Ctrl+C')
    args = parser.parse_args()
    try:
        zone = ZoneInfo(args.timezone)
    except (ZoneInfoNotFoundError, ValueError):
        parser.error(f'Unknown timezone: {args.timezone}')
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration must be positive and finite')
    if args.preview:
        write_png(args.preview, render_clock(zone=zone))
        print(f'Clock preview saved: {args.preview}')
        return 0
    node = None
    old_term = signal.signal(signal.SIGTERM, interrupt)
    try:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = HeadClock(zone)
        deadline = None if args.duration is None else time.monotonic() + args.duration
        while not node.finished and (deadline is None or time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=.1)
        print(f'Head clock stopped after publishing {node.frames} frames.')
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if node:
            try:
                node.release()
            except Exception as exc:
                print(f'Clock release unavailable; display lease will expire: {exc}', file=sys.stderr)
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, old_term)


if __name__ == '__main__':
    sys.exit(main())
