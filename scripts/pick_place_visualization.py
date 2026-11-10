"""Isolated Matplotlib processes and nonblocking live measurement delivery.

No ROS or plotting imports here. The controller only appends to a bounded queue;
serialization, pipe writes, GUI events and rendering run outside its loop.
"""

import json
import os
from pathlib import Path
import queue
import select
import subprocess
import sys
import tempfile
import threading
import time


def plot_environment():
    environment = os.environ.copy()
    for name in ('PYTHONPATH', 'PYTHONHOME', 'FONTCONFIG_FILE', 'FONTCONFIG_PATH', 'MPLBACKEND'):
        environment.pop(name, None)
    environment['PYTHONNOUSERSITE'] = '1'
    environment['XDG_CACHE_HOME'] = os.path.join(tempfile.gettempdir(), 'sawyer-plot-cache')
    environment['MPLCONFIGDIR'] = os.path.join(tempfile.gettempdir(), 'sawyer-matplotlib')
    environment['SAWYER_PLOT_WORKER'] = '1'
    return environment


def plot_python():
    for candidate in dict.fromkeys([sys.executable, '/usr/bin/python3']):
        try:
            check = subprocess.run([candidate, '-c',
                'import matplotlib; matplotlib.use("Agg"); '
                'from mpl_toolkits.mplot3d import Axes3D; import matplotlib.pyplot as p; '
                'p.figure().add_subplot(111, projection="3d"); p.close("all")'],
                capture_output=True, timeout=20, env=plot_environment())
        except (OSError, subprocess.TimeoutExpired):
            continue
        if check.returncode == 0:
            return candidate
    raise RuntimeError('Matplotlib is required in ROS Python or /usr/bin/python3 for 3D plotting')


def has_display():
    return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))


def worker_path():
    return str(Path(__file__).with_name('pick_place_plot.py'))


def show_saved_plot(renderer, path):
    """Open saved measurements in a rotatable desktop window, without blocking."""
    if not has_display():
        print(f'No desktop display. Reopen {path} with pick_place_plot.py --show on a desktop.')
        return False
    log_path = Path(path).with_suffix('.viewer.log')
    with log_path.open('a') as log:
        subprocess.Popen([renderer, worker_path(), str(path), '--show'],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=log,
            env=plot_environment(), start_new_session=True)
    print(f'Interactive 3D window: left-drag to rotate, right-drag to zoom. Data: {path}')
    return True


class LivePlot:
    def __init__(self, renderer, data_path):
        self.renderer, self.data_path = renderer, Path(data_path)
        self.pending = queue.Queue(maxsize=256)
        self.closed = threading.Event()
        self.process = self.writer = None
        self.finish_sent = False

    def start(self, pump=None, timeout=10):
        if not has_display():
            print('No desktop display; saving measurements for later interactive viewing.')
            return False
        log_path = self.data_path.with_suffix('.viewer.log')
        try:
            with log_path.open('a') as log:
                self.process = subprocess.Popen([self.renderer, worker_path(), str(self.data_path), '--live'],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, text=True,
                    env=plot_environment(), start_new_session=True, bufsize=1)
        except OSError as exc:
            print(f'Live 3D window unavailable: {exc}; measurements will still be saved.')
            self.closed.set()
            return False
        deadline = time.monotonic()+timeout
        try:
            while time.monotonic() < deadline:
                if self.process.poll() is not None:
                    break
                readable, _, _ = select.select([self.process.stdout], [], [], .02)
                if readable and self.process.stdout.readline().strip() == 'PLOT_READY':
                    self.writer = threading.Thread(target=self._write, daemon=True, name='pick-place-plot')
                    self.writer.start()
                    print('Live 3D plot opened; the window remains available after replay.')
                    return True
                if pump is not None:
                    pump()
            print(f'Live 3D window unavailable; measurements will still be saved. Viewer log: {log_path}')
            self._stop_startup()
            return False
        except BaseException:
            self._stop_startup()
            raise

    def _stop_startup(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
            for stream in (self.process.stdin, self.process.stdout):
                stream.close()
        self.closed.set()

    def publish(self, row):
        """Best-effort UI delivery; all full-rate rows stay in the session log."""
        if self.closed.is_set():
            return
        try:
            self.pending.put_nowait(dict(type='point', phase=row['phase'], position=row['actual']['position']))
        except queue.Full:
            pass  # A slow/closed viewer must never delay joint commands.

    def _write(self):
        try:
            while True:
                message = self.pending.get()
                self.process.stdin.write(json.dumps(message, allow_nan=False)+'\n')
                self.process.stdin.flush()
                if message['type'] == 'finish':
                    self.finish_sent = True
                    return
        except (OSError, ValueError):
            pass  # Includes a viewer closed while the arm is moving.
        finally:
            self.closed.set()
            self.process.stdin.close()
            self.process.stdout.close()

    def finish(self, path, keep_open=True):
        """After control stops, replace the downsampled live trace with all rows."""
        if self.closed.is_set() or self.process is None:
            return False
        # Final data is on disk; stale queued UI samples can be discarded.
        while True:
            try:
                self.pending.get_nowait()
            except queue.Empty:
                break
        self.pending.put_nowait(dict(type='finish', file=str(Path(path).resolve()), keep_open=keep_open))
        self.writer.join(timeout=2)
        return self.finish_sent
