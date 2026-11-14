"""Exercise the optional lifecycle adapter with a minimal framework fixture."""

# Fixture inspects ownership; this is not a real ros_sugar runtime test.
# ruff: noqa: SLF001
from __future__ import annotations

import importlib.util
import sys
import threading
from pathlib import Path
from types import ModuleType
from unittest.mock import MagicMock

import pytest


@pytest.fixture
def component(monkeypatch):
    framework = ModuleType('ros_sugar.core')

    class BaseComponent:
        def __init__(self, **kwargs):
            self.component_arguments = kwargs

        def get_logger(self):
            return MagicMock()

    framework.BaseComponent = BaseComponent
    monkeypatch.setitem(sys.modules, 'ros_sugar.core', framework)
    path = Path(__file__).parents[2] / (
        'irobot/src/robots/sawyer/ros_sugar_component.py'
    )
    spec = importlib.util.spec_from_file_location(
        'irobot.src.robots.sawyer._component_fixture', path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    driver = MagicMock()
    driver.config.gripper_timeout = driver.config.connect_timeout = 0.1
    monkeypatch.setattr(module, 'SawyerController', lambda _config: driver)
    return module.SawyerComponent(), driver


def test_lifecycle_connects_without_enabling_or_starting_a_task(component):
    application, driver = component
    application.custom_on_configure()
    try:
        application.custom_on_activate()
        driver.connect.assert_called_once()
        driver.move_to_neutral.assert_not_called()
        driver.devices.set_enabled.assert_not_called()
        application.move_to_neutral()
        application._sawyer_task.result(timeout=1)
        application._execution_step()
        assert driver.move_to_neutral.call_args.kwargs['execute'] is False
    finally:
        application.custom_on_cleanup()
    driver.disconnect.assert_called_once()


def test_lifecycle_rejects_queued_tasks_and_cancels_on_deactivate(component):
    application, driver = component
    started = threading.Event()

    def move(**kwargs):
        started.set()
        assert kwargs['cancel_event'].wait(1)

    driver.move_to_neutral.side_effect = move
    with pytest.raises(RuntimeError, match='Activate'):
        application.move_to_neutral()
    application.custom_on_configure()
    application.custom_on_activate()
    try:
        application.move_to_neutral(execute=True)
        assert started.wait(1)
        with pytest.raises(RuntimeError, match='already running'):
            application.open_gripper()
        application.custom_on_deactivate()
        driver.stop.assert_called_once()
    finally:
        application.custom_on_cleanup()


def test_completed_task_collection_preserves_concurrent_new_task(component):
    application, _driver = component
    inspected, release, submitted = (
        threading.Event(),
        threading.Event(),
        threading.Event(),
    )
    old_task, new_task = MagicMock(), MagicMock()
    application._sawyer_active = True
    application._sawyer_worker = MagicMock()
    application._sawyer_worker.submit.return_value = new_task
    application._sawyer_task = old_task
    old_task.result.return_value = 'first result'
    checks = []

    def completed():
        checks.append(True)
        if len(checks) == 1:
            inspected.set()
            assert release.wait(1)
        return True

    old_task.done.side_effect = completed
    collector = threading.Thread(target=application._execution_step)

    def submit():
        application.open_gripper()
        submitted.set()

    requester = threading.Thread(target=submit)
    collector.start()
    assert inspected.wait(1)
    requester.start()
    try:
        assert not submitted.wait(0.05)
    finally:
        release.set()
        collector.join(timeout=1)
        requester.join(timeout=1)
    assert submitted.is_set()
    assert application._sawyer_task is new_task
    assert application.last_result == 'first result'
