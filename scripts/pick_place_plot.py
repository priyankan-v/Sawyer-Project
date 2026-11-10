"""Save or interactively view pick/place measurements, including a live stream."""

import argparse
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import threading


def create_plot(data, interactive=False):
    os.environ.setdefault('MPLCONFIGDIR', os.path.join(tempfile.gettempdir(), 'sawyer-matplotlib'))
    import matplotlib
    matplotlib.use('TkAgg' if interactive else 'Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    figure = plt.figure(figsize=(11, 8))
    axes = figure.add_subplot(111, projection='3d')
    all_positions = []
    lines = {}
    for index, segment in enumerate(data['segments']):
        color = plt.get_cmap('tab10')(index % 10)
        positions = np.array([p['position'] for p in segment['poses']])
        if len(positions):
            axes.plot(*positions.T, '--', color=color, linewidth=1.5, label='Planned '+segment['name'])
            all_positions.extend(positions)
        lines[segment['name']] = axes.plot([], [], [], color=color, linewidth=2,
                                          label='Measured '+segment['name'])[0]
    all_positions.extend(row['actual']['position'] for row in data['measurements'])
    for waypoint in data['waypoints']:
        position = waypoint['pose']['position']
        axes.scatter(*position, color='black', s=32)
        axes.text(*position, ' '+waypoint['name'])
        all_positions.append(position)
    positions = np.array(all_positions or [(0,0,0)])
    low, high = positions.min(axis=0), positions.max(axis=0)
    center = (low+high)/2
    radius = max(.05, float(max(high-low))/2)*1.1
    axes.set_xlim(center[0]-radius, center[0]+radius)
    axes.set_ylim(center[1]-radius, center[1]+radius)
    axes.set_zlim(center[2]-radius, center[2]+radius)
    axes.set_box_aspect((1, 1, 1))
    axes.set(xlabel='Base X (m)', ylabel='Base Y (m)', zlabel='Base Z (m)',
             title='Sawyer pick and place: recorded, planned and measured paths')
    axes.legend(fontsize=6.5, loc='upper left')
    figure.tight_layout()
    marker = axes.plot([], [], [], 'o', color='red', markersize=5)[0]
    return PlotView(figure, axes, lines, marker, data['measurements']), plt


class PlotView:
    def __init__(self, figure, axes, lines, marker, rows):
        self.figure, self.axes, self.lines, self.marker = figure, axes, lines, marker
        self.positions = {}
        self.last = None
        self.replace_measurements(rows)

    def add_point(self, phase, position):
        self.positions.setdefault(phase, []).append(position)
        self.last = position

    def replace_measurements(self, rows):
        self.positions.clear()
        self.last = None
        for row in rows:
            self.add_point(row['phase'], row['actual']['position'])
        self.redraw()

    def redraw(self):
        # Updating existing lines preserves the user's current rotation/zoom.
        for phase, positions in self.positions.items():
            if phase not in self.lines:
                self.lines[phase] = self.axes.plot([], [], [], linewidth=2, label='Measured '+phase)[0]
            self.lines[phase].set_data_3d(*zip(*positions))
        for phase, line in self.lines.items():
            if phase not in self.positions:
                line.set_data_3d([], [], [])
        if self.last is not None:
            self.marker.set_data_3d(*[(value,) for value in self.last])
        self.figure.canvas.draw_idle()


def plot_path_3d(data, destination):
    view, plt = create_plot(data)
    figure = view.figure
    figure.savefig(destination, dpi=180)
    plt.close(figure)


def receive_stream(incoming):
    for line in sys.stdin:
        try:
            message = json.loads(line)
            if message['type'] == 'finish':
                # Final snapshot contains every point; old live updates can go.
                while True:
                    try:
                        incoming.get_nowait()
                    except queue.Empty:
                        break
                incoming.put_nowait(message)
                return
            incoming.put_nowait(message)
        except (ValueError, KeyError, queue.Full):
            continue
    incoming.put(dict(type='disconnected'))


def show_plot(data, live=False):
    view, plt = create_plot(data, interactive=True)
    if live:
        incoming = queue.Queue(maxsize=2048)
        def update():
            changed = False
            for _ in range(2048):
                try:
                    message = incoming.get_nowait()
                except queue.Empty:
                    break
                if message['type'] == 'point':
                    view.add_point(message['phase'], message['position'])
                    changed = True
                elif message['type'] == 'finish':
                    with open(message['file']) as stream:
                        final = json.load(stream)
                    view.replace_measurements(final['measurements'])
                    view.axes.set_title('Replay finished — drag to rotate; toolbar to save/zoom')
                    view.figure.canvas.draw_idle()
                    timer.stop()
                    if not message['keep_open']:
                        plt.close(view.figure)
                    return
                elif message['type'] == 'disconnected':
                    view.axes.set_title('Live stream ended — drag to rotate')
                    timer.stop()
                    changed = True
            if changed:
                view.redraw()
        threading.Thread(target=receive_stream, args=(incoming,), daemon=True).start()
        timer = view.figure.canvas.new_timer(interval=100)
        timer.add_callback(update)
        timer.start()
    view.figure.canvas.draw()
    print('PLOT_READY', flush=True)
    plt.show(block=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('data', type=Path, help='Saved .plot.json file')
    parser.add_argument('destination', nargs='?', help='PNG destination for headless rendering')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--show', action='store_true', help='Open a rotatable 3D window for saved measurements')
    modes.add_argument('--live', action='store_true', help='Read live JSONL telemetry from stdin')
    args = parser.parse_args(argv)
    if not (args.show or args.live or args.destination):
        parser.error('Supply a PNG destination or --show')
    if os.environ.get('SAWYER_PLOT_WORKER') != '1':
        if __package__:
            from .pick_place_visualization import plot_python, plot_environment
        else:
            from pick_place_visualization import plot_python, plot_environment
        # Reopening a saved plot works from the ROS shell as well as system Python.
        return subprocess.call([plot_python(), str(Path(__file__).resolve()),
                                *(sys.argv[1:] if argv is None else argv)], env=plot_environment())
    with args.data.open() as stream:
        data = json.load(stream)
    if args.show or args.live:
        show_plot(data, live=args.live)
    else:
        plot_path_3d(data, args.destination)
    return 0


if __name__ == '__main__':
    sys.exit(main())
