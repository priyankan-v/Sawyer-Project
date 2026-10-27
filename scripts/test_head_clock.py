"""Clock timezone/rendering/publishing checks without ROS initialization."""

from datetime import datetime, timedelta, timezone
from pathlib import Path
import json
import struct
import tempfile
import unittest
import time
from unittest.mock import MagicMock, patch
from zoneinfo import ZoneInfo

from builtin_interfaces.msg import Time
from std_msgs.msg import String
import numpy as np

from clock_display import HEIGHT, WIDTH, clock_fields, render_clock, write_png
import head_clock as clock


class ClockTests(unittest.TestCase):
    def test_chicago_local_time_date_and_offset(self):
        fields = clock_fields(datetime(2026, 10, 26, 18, 4, 5, tzinfo=timezone.utc), ZoneInfo('America/Chicago'))
        self.assertEqual(fields.time, '13:04:05')
        self.assertEqual(fields.date, 'MONDAY 26 OCTOBER 2026')
        self.assertIn('CDT  UTC-05:00', fields.zone)

    def test_daylight_saving_fallback_updates_zone_as_well_as_hour(self):
        zone = ZoneInfo('America/Chicago')
        before = clock_fields(datetime(2026, 11, 1, 6, 59, 59, tzinfo=timezone.utc), zone)
        after = clock_fields(datetime(2026, 11, 1, 7, 0, 0, tzinfo=timezone.utc), zone)
        self.assertEqual(before.time, '01:59:59')
        self.assertEqual(after.time, '01:00:00')
        self.assertIn('CDT', before.zone)
        self.assertIn('CST  UTC-06:00', after.zone)

    def test_frame_resolution_encoding_and_png_preview(self):
        image = render_clock(datetime(2026, 10, 26, 18, tzinfo=timezone.utc))
        self.assertEqual(image.shape, (600, 1024, 3))
        self.assertEqual(image.dtype, np.uint8)
        self.assertGreater(len(np.unique(image.reshape(-1,3), axis=0)), 3)
        message = clock.image_message(image)
        self.assertEqual((message.width, message.height, message.step), (1024, 600, 3072))
        self.assertEqual(message.encoding, 'bgr8')
        self.assertEqual(len(message.data), 1024*600*3)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'clock.png'
            write_png(path, image)
            contents = path.read_bytes()
            self.assertTrue(contents.startswith(b'\x89PNG\r\n\x1a\n'))
            self.assertEqual(struct.unpack('!II', contents[16:24]), (WIDTH, HEIGHT))

    def test_only_one_image_per_wall_second_and_reconnect_republishes(self):
        node = clock.HeadClock.__new__(clock.HeadClock)
        node.publisher, node.get_logger, node.get_clock = MagicMock(), MagicMock(), MagicMock()
        node.publisher.get_subscription_count.return_value = 0
        node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
        node.zone = ZoneInfo('America/Chicago')
        node.session, node.owner = 'bridge-session', 'clock-owner'
        node.active, node.finished, node.last_status = True, False, time.monotonic()
        node.last_second, node.frames, node.waiting = None, 0, False
        instant = datetime(2026, 10, 26, 18, tzinfo=timezone.utc)
        with patch.object(clock, 'datetime') as wall:
            wall.now.return_value = instant
            node.tick()
            node.publisher.publish.assert_not_called()
            node.publisher.get_subscription_count.return_value = 1
            node.tick()
            node.tick()
            self.assertEqual(node.frames, 1)
            wall.now.return_value = instant + timedelta(seconds=1)
            node.tick()
            self.assertEqual(node.frames, 2)
            node.publisher.get_subscription_count.return_value = 0
            node.tick()
            node.publisher.get_subscription_count.return_value = 1
            node.tick()
            self.assertEqual(node.frames, 3)
            message = node.publisher.publish.call_args.args[0]
            self.assertEqual(message.header.frame_id, 'clock:bridge-session:clock-owner')

    def test_clock_stops_on_bridge_shutdown_or_replacement(self):
        for session, active in [('same', False), ('new', True)]:
            node = clock.HeadClock.__new__(clock.HeadClock)
            node.session, node.active, node.finished = 'same', True, False
            node.on_status(String(data=json.dumps({'session': session, 'active': active})))
            self.assertTrue(node.finished)
            self.assertFalse(node.active)

    def test_clock_waits_for_manager_and_releases_own_lease(self):
        node = clock.HeadClock.__new__(clock.HeadClock)
        node.session, node.owner, node.active, node.finished = None, 'owner', False, False
        node.release_publisher = MagicMock()
        node.on_status(String(data='invalid json'))
        node.on_status(String(data='[]'))
        self.assertIsNone(node.session)
        node.on_status(String(data='{"session":"manager","active":true}'))
        self.assertEqual(node.session, 'manager')
        self.assertTrue(node.active)
        node.release()
        self.assertEqual(node.release_publisher.publish.call_args.args[0].data, 'clock:manager:owner')

    def test_lost_manager_heartbeat_stops_clock(self):
        node = clock.HeadClock.__new__(clock.HeadClock)
        node.session, node.last_status, node.active, node.finished = 'manager', time.monotonic()-5, True, False
        node.waiting, node.get_logger = False, MagicMock()
        node.tick()
        self.assertTrue(node.finished)
        self.assertFalse(node.active)

    def test_invalid_naive_timestamp_and_image_rejected(self):
        with self.assertRaises(ValueError):
            clock_fields(datetime(2026, 10, 26), ZoneInfo('America/Chicago'))
        with self.assertRaises(ValueError):
            clock.image_message(np.zeros((10,10,3), dtype=np.uint8))

    def test_preview_does_not_initialize_ros(self):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder) / 'clock.png'
            with patch('sys.argv', ['head_clock.py', '--preview', str(target)]), patch.object(clock.rclpy, 'init') as initialize, patch('builtins.print'):
                self.assertEqual(clock.main(), 0)
            initialize.assert_not_called()
            self.assertTrue(target.is_file())


if __name__ == '__main__':
    unittest.main()
