"""
Optional integration with the installed real ros_sugar lifecycle framework.
"""

# Inspect worker ownership while running real lifecycle transitions.
# ruff: noqa: SLF001
from __future__ import annotations

import os
import threading
import time
from unittest.mock import MagicMock

import pytest

if os.environ.get('SAWYER_SUGAR_TEST') != '1':
    pytest.skip(
        'Set SAWYER_SUGAR_TEST=1 for real framework checks',
        allow_module_level=True,
    )

pytest.importorskip('ros_sugar.core')
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn

from irobot.src.robots.sawyer import ros_sugar_component as adapter


def test_real_framework_timer_cancellation_and_cleanup(monkeypatch):
    context = Context()
    context.init(domain_id=105)
    driver = MagicMock()
    driver.config.gripper_timeout = driver.config.connect_timeout = 0.1
    driver.move_to_neutral.return_value = {'preview': True}
    monkeypatch.setattr(adapter, 'SawyerController', lambda _config: driver)
    node = adapter.SawyerComponent()
    node.rclpy_init_node(context=context)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS
        driver.move_to_neutral.assert_not_called()
        node.move_to_neutral()
        deadline = time.monotonic() + 3
        while node.last_result is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert node.last_result == {'preview': True}
        assert not driver.move_to_neutral.call_args.kwargs['execute']
        started = threading.Event()

        def wait_cancel(**kwargs):
            started.set()
            assert kwargs['cancel_event'].wait(2)

        driver.open_gripper.side_effect = wait_cancel
        node.open_gripper()
        assert started.wait(1)
        assert node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS
        assert node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS
        assert node._sawyer_worker is None
        driver.disconnect.assert_called_once()
        driver.devices.set_enabled.assert_not_called()
    finally:
        executor.shutdown(timeout_sec=3)
        worker.join(timeout=3)
        node.custom_on_cleanup()
        node.destroy_node()
        context.shutdown()
