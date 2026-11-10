"""Nonblocking visualization delivery and saved/live trace integrity."""

import io
import json
from pathlib import Path
import queue
import tempfile
import threading
import unittest
from unittest.mock import MagicMock, patch

from pick_place_plot import PlotView, receive_stream
from pick_place_visualization import LivePlot, plot_environment


class PlotDeliveryTests(unittest.TestCase):
    def test_slow_viewer_never_blocks_measurement_delivery(self):
        viewer = LivePlot('/unused/python', '/tmp/unused.json')
        row = dict(phase='to_middle_1', actual=dict(position=(.1,.2,.3)))
        for _ in range(1000):
            viewer.publish(row)
        self.assertEqual(viewer.pending.qsize(), 256)
        viewer.closed.set()
        viewer.publish(row)
        self.assertEqual(viewer.pending.qsize(), 256)

    def test_closed_viewer_pipe_does_not_raise_into_controller(self):
        viewer = LivePlot('/unused/python', '/tmp/unused.json')
        viewer.process = MagicMock()
        viewer.process.stdin.write.side_effect = BrokenPipeError()
        viewer.publish(dict(phase='lift', actual=dict(position=(0,0,.2))))
        viewer._write()
        self.assertTrue(viewer.closed.is_set())
        viewer.process.stdin.close.assert_called_once()

    def test_finish_uses_complete_saved_snapshot_and_closes_writer(self):
        viewer = LivePlot('/unused/python', '/tmp/unused.json')
        viewer.process = MagicMock()
        viewer.writer = threading.Thread(target=viewer._write, daemon=True)
        viewer.writer.start()
        self.assertTrue(viewer.finish('/tmp/complete.plot.json', keep_open=True))
        event = json.loads(viewer.process.stdin.write.call_args.args[0])
        self.assertEqual(event, dict(type='finish', file='/tmp/complete.plot.json', keep_open=True))
        self.assertFalse(viewer.writer.is_alive())

    def test_gui_start_failure_is_optional_and_does_not_abort_replay(self):
        with tempfile.TemporaryDirectory() as directory:
            viewer = LivePlot('/unused/python', Path(directory)/'plot.json')
            with patch('pick_place_visualization.has_display', return_value=True), \
                    patch('pick_place_visualization.subprocess.Popen', side_effect=OSError('no display')), \
                    patch('builtins.print'):
                self.assertFalse(viewer.start())

    def test_finish_event_survives_full_gui_queue(self):
        incoming = queue.Queue(maxsize=1)
        incoming.put(dict(type='point'))
        finish = dict(type='finish', file='/tmp/final.json', keep_open=True)
        with patch('sys.stdin', io.StringIO(json.dumps(finish)+'\n')):
            receive_stream(incoming)
        self.assertEqual(incoming.get_nowait(), finish)

    def test_updates_keep_camera_and_replace_live_trace_with_full_measurements(self):
        figure, axes, line, marker = MagicMock(), MagicMock(), MagicMock(), MagicMock()
        view = PlotView(figure, axes, {'lift': line}, marker, [])
        view.add_point('lift', (1,2,3))
        view.redraw()
        self.assertEqual(view.positions['lift'], [(1,2,3)])
        rows = [dict(phase='lift', actual=dict(position=p)) for p in ((0,0,0), (.1,.2,.3))]
        view.replace_measurements(rows)
        self.assertEqual(view.positions['lift'], [(0,0,0), (.1,.2,.3)])
        axes.clear.assert_not_called()
        axes.set_xlim.assert_not_called()
        axes.view_init.assert_not_called()

    def test_plot_process_does_not_inherit_ros_python_paths(self):
        with patch.dict('os.environ', {'PYTHONPATH':'/ros/python3.12', 'MPLBACKEND':'Agg'}):
            environment = plot_environment()
        self.assertNotIn('PYTHONPATH', environment)
        self.assertNotIn('MPLBACKEND', environment)
        self.assertEqual(environment['SAWYER_PLOT_WORKER'], '1')


if __name__ == '__main__':
    unittest.main()
