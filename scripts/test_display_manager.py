"""Display ownership, PNG decoding, shutdown ordering, and ROS 1 wire checks."""

import io
import json
from pathlib import Path
import struct
import tempfile
import threading
import unittest
from unittest.mock import MagicMock, patch
import zlib

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String

import display_manager as manager
from display_images import image_message, read_png, screen_image
from display_state import DisplayPolicy, clock_tag
import ros1_display as ros1


PROJECT = Path(__file__).resolve().parents[1]


def png_chunk(kind, data):
    return struct.pack('!I', len(data))+kind+data+struct.pack('!I', zlib.crc32(kind+data))


class ImageTests(unittest.TestCase):
    def test_project_and_original_sdk_images_load(self):
        for name in ('display.png', 'assets/default_display.png'):
            frame = screen_image(PROJECT / name)
            message = image_message(frame)
            self.assertEqual(frame.shape, (600, 1024, 3))
            self.assertEqual(message.encoding, 'bgr8')
            self.assertEqual(len(message.data), 600*1024*3)
            self.assertGreater(int(frame.max()), int(frame.min()))

    def test_png_filters_and_rgba_compositing(self):
        # Two identical rows, with transparency; encode using each PNG filter.
        pixels = bytes([200, 10, 30, 255, 0, 100, 200, 128])
        expected = np.array([[[30, 10, 200], [227, 177, 127]]], dtype=np.uint8)
        for mode in range(5):
            encoded = bytearray()
            for row in range(2):
                encoded.append(mode)
                for x, value in enumerate(pixels):
                    a = pixels[x-4] if x >= 4 else 0
                    b = pixels[x] if row else 0
                    c = pixels[x-4] if row and x >= 4 else 0
                    p = a+b-c
                    distances = abs(p-a), abs(p-b), abs(p-c)
                    paeth = (a, b, c)[distances.index(min(distances))]
                    prediction = (0, a, b, (a+b)//2, paeth)[mode]
                    encoded.append((value-prediction) & 255)
            contents = (b'\x89PNG\r\n\x1a\n'
                        + png_chunk(b'IHDR', struct.pack('!IIBBBBB', 2, 2, 8, 6, 0, 0, 0))
                        + png_chunk(b'IDAT', zlib.compress(encoded)) + png_chunk(b'IEND', b''))
            with tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / 'image.png'
                path.write_bytes(contents)
                frame = read_png(path)
                np.testing.assert_array_equal(frame, np.repeat(expected, 2, axis=0))
                self.assertEqual(screen_image(path).shape, (600, 1024, 3))
                path.write_bytes(contents[:-8])
                with self.assertRaises(ValueError):
                    read_png(path)


class PolicyTests(unittest.TestCase):
    def test_start_clock_release_and_bridge_stop(self):
        policy = DisplayPolicy(session='bridge')
        self.assertEqual(policy.selection(0)[0], 'native')
        self.assertFalse(policy.accept_clock('bridge', 'clock', 'frame', 0))
        policy.set_bridge(True)
        self.assertEqual(policy.selection(0)[0], 'image')
        self.assertTrue(policy.accept_clock('bridge', 'clock', 'frame', 0))
        self.assertEqual(policy.selection(1), ('clock', 'frame'))
        policy.release_clock('bridge', 'clock')
        self.assertEqual(policy.selection(1)[0], 'image')
        policy.accept_clock('bridge', 'clock', 'frame', 1)
        policy.set_bridge(False)
        self.assertEqual(policy.selection(2)[0], 'native')

    def test_sigkill_expires_lease_and_old_clock_cannot_release_new_owner(self):
        policy = DisplayPolicy(session='bridge')
        policy.set_bridge(True)
        policy.accept_clock('bridge', 'old', 'old frame', 0)
        self.assertFalse(policy.accept_clock('old-bridge', 'old', 'stale', 1))
        policy.accept_clock('bridge', 'new', 'new frame', 1)
        policy.release_clock('bridge', 'old')
        self.assertEqual(policy.selection(2), ('clock', 'new frame'))
        self.assertEqual(policy.selection(3.5)[0], 'image')


def fake_manager():
    node = manager.DisplayManager.__new__(manager.DisplayManager)
    node.image, node.default_image = Image(), Image()
    node.policy = DisplayPolicy(session='bridge')
    node.publisher, node.status_publisher, node.probe = MagicMock(), MagicMock(), MagicMock()
    node.publisher.get_subscription_count.return_value = 1
    node.probe.status = True, True, ''
    node.ever_active, node.finished = False, False
    node.last_key, node.last_status, node.last_subscribers = None, -100, 0
    node.last_publish = -100
    node.get_logger, node.destroy_publisher = MagicMock(), MagicMock()
    return node


class ManagerTests(unittest.TestCase):
    def test_manager_switches_project_clock_and_back_after_timeout(self):
        node = fake_manager()
        with patch.object(manager.time, 'monotonic', return_value=0) as now:
            node.tick()
            self.assertIs(node.publisher.publish.call_args.args[0], node.image)
            node.tick()
            self.assertEqual(node.publisher.publish.call_count, 1)
            image = image_message(np.zeros((600,1024,3), dtype=np.uint8))
            image.header.frame_id = clock_tag('bridge', 'clock')
            node.on_clock(image)
            node.tick()
            self.assertIs(node.publisher.publish.call_args.args[0], image)
            now.return_value = 2.5
            node.tick()
            self.assertIs(node.publisher.publish.call_args.args[0], node.image)
            node.on_clock(image)
            node.tick()
            node.on_release(String(data=clock_tag('bridge', 'clock')))
            node.tick()
            self.assertIs(node.publisher.publish.call_args.args[0], node.image)

    def test_bridge_loss_finishes_and_stops_clock(self):
        node = fake_manager()
        node.tick()
        node.probe.status = True, False, 'Exited'
        node.tick()
        self.assertTrue(node.finished)
        self.assertFalse(json.loads(node.status_publisher.publish.call_args.args[0].data)['active'])

    def test_static_refresh_handles_ros1_reconnect_without_ros2_graph_change(self):
        node = fake_manager()
        with patch.object(manager.time, 'monotonic', return_value=0) as now:
            node.tick()
            now.return_value = 2
            node.tick()
            self.assertEqual(node.publisher.publish.call_count, 2)

    def test_shutdown_removes_bridge_writer_before_independent_ros1_restore(self):
        node = fake_manager()
        node.ever_active = True
        events = []
        node.destroy_publisher.side_effect = lambda _: events.append('destroy')
        with patch.object(manager, 'NativeImagePublisher') as publisher, patch.object(manager.time, 'sleep'):
            native = MagicMock()
            native.publish.return_value = 1
            publisher.side_effect = lambda: events.append('restore') or native
            node.restore_default()
        self.assertEqual(events, ['destroy', 'restore'])
        node.probe.close.assert_called_once()
        native.close.assert_called_once()

    def test_startup_without_bridge_does_not_change_native_image(self):
        node = fake_manager()
        with patch.object(manager, 'NativeImagePublisher') as publisher:
            node.restore_default()
        publisher.assert_not_called()

    def test_invalid_or_old_session_clock_is_ignored(self):
        node = fake_manager()
        node.tick()
        node.on_clock(Image())
        node.on_release(String(data='bad tag'))
        self.assertEqual(node.policy.selection(0)[0], 'image')


class WireTests(unittest.TestCase):
    def test_ros1_image_serialization(self):
        image = image_message(np.zeros((600,1024,3), dtype=np.uint8))
        wire = ros1.serialize_image(image, sequence=9)
        self.assertEqual(struct.unpack_from('<I', wire)[0], len(wire)-4)
        self.assertEqual(struct.unpack_from('<I', wire, 4)[0], 9)
        # Header: seq/sec/nsec then empty frame_id, height/width, encoding.
        self.assertEqual(struct.unpack_from('<II', wire, 20), (600,1024))
        self.assertEqual(wire[32:36], b'bgr8')
        self.assertEqual(wire[-len(image.data):], bytes(image.data))

    def test_tcpros_header_round_trip_and_truncation(self):
        fields = {'topic': ros1.TOPIC, 'md5sum': ros1.IMAGE_MD5}
        connection = MagicMock()
        data = io.BytesIO(ros1.encode_header(fields))
        connection.recv.side_effect = data.read
        self.assertEqual(ros1.receive_header(connection), fields)
        connection.recv.return_value = b''
        connection.recv.side_effect = None
        with self.assertRaises(ConnectionError):
            ros1.receive_header(connection)

    def test_bridge_generation_change_is_detected(self):
        probe = ros1.BridgeProbe.__new__(ros1.BridgeProbe)
        probe.master_uri, probe.name, probe.generation = 'http://master/', '/bridge', None
        master, node = MagicMock(), MagicMock()
        master.lookupNode.return_value = [1, '', 'http://bridge/']
        node.getPid.return_value = [1, '', 123]
        def connect(uri):
            context = MagicMock()
            context.__enter__.return_value = master if uri == probe.master_uri else node
            return context
        with patch.object(ros1, 'proxy', side_effect=connect):
            probe.check()
            self.assertEqual(probe.generation, ('http://bridge/', 123))
            node.getPid.return_value = [1, '', 456]
            with self.assertRaises(ConnectionError):
                probe.check()

    def test_bridge_probe_requires_three_failures_after_seen(self):
        probe = ros1.BridgeProbe.__new__(ros1.BridgeProbe)
        probe.generation = ('http://bridge/', 123)
        probe.stopping = threading.Event()
        probe.status = True, True, ''
        probe.check = MagicMock(side_effect=ConnectionError('bridge gone'))
        with patch.object(probe.stopping, 'wait'):
            probe.run()
        self.assertEqual(probe.check.call_count, 3)
        self.assertEqual(probe.status, (True, False, 'bridge gone'))


if __name__ == '__main__':
    unittest.main()
