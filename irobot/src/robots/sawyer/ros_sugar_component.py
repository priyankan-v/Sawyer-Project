"""Optional ros_sugar application component using the direct Python driver."""

from __future__ import annotations

import threading
from concurrent.futures import ThreadPoolExecutor

from ros_sugar.core import BaseComponent

from .core.controller import SawyerController


class SawyerComponent(BaseComponent):
    """
    Lifecycle-owned example application; activation starts no robot movement.

    Choose this direct application or SawyerNode as the hardware owner. The
    separate SawyerNode provides the complete typed ROS 2 server interface.
    """

    def __init__(
        self,
        *,
        component_name='sawyer_application',
        sawyer_config=None,
        **kwargs,
    ):
        self.sawyer = SawyerController(sawyer_config)
        self._sawyer_cancel = threading.Event()
        self._sawyer_lock = threading.Lock()
        self._sawyer_worker = self._sawyer_task = None
        self._sawyer_active = False
        self.last_result = None
        super().__init__(component_name=component_name, **kwargs)

    def custom_on_configure(self):
        self.sawyer.connect()
        self._sawyer_worker = ThreadPoolExecutor(max_workers=1)

    def custom_on_activate(self):
        with self._sawyer_lock:
            self._sawyer_active = True

    def _submit(self, operation, **kwargs):
        with self._sawyer_lock:
            if not self._sawyer_active or self._sawyer_worker is None:
                msg = 'Activate the component before requesting a task'
                raise RuntimeError(msg)
            if self._sawyer_task is not None and not self._sawyer_task.done():
                msg = 'A Sawyer task is already running'
                raise RuntimeError(msg)
            self._sawyer_cancel.clear()
            self._sawyer_task = self._sawyer_worker.submit(
                operation, cancel_event=self._sawyer_cancel, **kwargs
            )

    def move_to_neutral(self, *, execute=False, duration=None):
        self._submit(
            self.sawyer.move_to_neutral, execute=execute, duration=duration
        )

    def open_gripper(self, *, execute=False):
        self._submit(self.sawyer.open_gripper, execute=execute)

    def close_gripper(self, *, execute=False):
        self._submit(self.sawyer.close_gripper, execute=execute)

    def cancel_motion(self):
        self._sawyer_cancel.set()
        self.sawyer.stop()

    def _execution_step(self):
        with self._sawyer_lock:
            if self._sawyer_task is None or not self._sawyer_task.done():
                return
            task, self._sawyer_task = self._sawyer_task, None
        try:
            self.last_result = task.result()
        except (RuntimeError, ValueError, OSError) as exc:
            self.get_logger().error(str(exc))
            self.last_result = {'error': str(exc)}

    def custom_on_deactivate(self):
        with self._sawyer_lock:
            self._sawyer_active = False
        self.cancel_motion()
        if self._sawyer_task is not None:
            timeout = (
                self.sawyer.config.gripper_timeout
                + self.sawyer.config.connect_timeout
                + 3
            )
            try:
                self._sawyer_task.result(timeout=timeout)
            except TimeoutError:
                raise
            except (RuntimeError, ValueError, OSError) as exc:
                self.get_logger().warning(str(exc))

    def custom_on_cleanup(self):
        self.custom_on_deactivate()
        if self._sawyer_worker is not None:
            self._sawyer_worker.shutdown(wait=True)
            self._sawyer_worker = None
        self.sawyer.disconnect()

    def custom_on_shutdown(self):
        self.custom_on_cleanup()

    def custom_on_error(self):
        self.cancel_motion()
