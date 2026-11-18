"""Controller-native ROS 1 actions without actionlib or a ROS runtime."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from uuid import uuid4

from ...config import positive
from . import wire

TERMINAL_STATES = frozenset((2, 3, 4, 5, 8, 9))


@dataclass(frozen=True)
class ActionResult:
    goal_id: str
    status: int
    text: str
    result: object

    @property
    def succeeded(self):
        return self.status == wire.GoalStatus.SUCCEEDED


class NativeActionClient:
    """
    One in-flight goal, bounded waits, explicit cancellation, no replay.

    Callbacks are dispatched by the owning direct transport. A late result
    from another goal cannot finish the current one. Controller-owned actions
    may outlive a workstation process; cancellation needs a live connection.
    """

    def __init__(self, node, namespace, action_type, *, server_timeout=2.0):
        self.node, self.namespace = node, namespace
        self.server_timeout = server_timeout
        self.goal_type = wire.message_type(action_type + 'ActionGoal')
        self.result_type = wire.message_type(action_type + 'ActionResult')
        self.feedback_type = wire.message_type(action_type + 'ActionFeedback')
        self.goal_pub = node.create_publisher(
            self.goal_type, namespace + '/goal'
        )
        self.cancel_pub = node.create_publisher(
            wire.GoalID, namespace + '/cancel'
        )
        node.create_subscription(
            wire.GoalStatusArray, namespace + '/status', self.on_status
        )
        node.create_subscription(
            self.result_type, namespace + '/result', self.on_result
        )
        node.create_subscription(
            self.feedback_type, namespace + '/feedback', self.on_feedback
        )
        self.busy = threading.Lock()
        self.active_id = None
        self.result = self.feedback = None
        self.status = wire.GoalStatus.PENDING
        self.status_at = 0.0

    def on_status(self, message):
        self.status_at = message.received_at or time.monotonic()
        if self.result is not None:
            return
        for status in message.status_list:
            if status.goal_id.id == self.active_id:
                self.status = status.status

    def on_result(self, message):
        if message.status.goal_id.id != self.active_id:
            return
        if message.status.status not in TERMINAL_STATES:
            msg = 'Action result has a nonterminal status'
            raise ValueError(msg)
        self.result = ActionResult(
            self.active_id,
            message.status.status,
            message.status.text,
            message.result,
        )

    def on_feedback(self, message):
        if message.status.goal_id.id == self.active_id:
            self.feedback = message.feedback

    def wait_ready(self, timeout):
        deadline = time.monotonic() + timeout
        while True:
            ready = (
                self.goal_pub.get_subscription_count() > 0
                and self.cancel_pub.get_subscription_count() > 0
                and time.monotonic() - self.status_at < self.server_timeout
            )
            if ready:
                return
            if time.monotonic() >= deadline:
                msg = f'Action server unavailable: {self.namespace}'
                raise TimeoutError(msg)
            self.node.spin_once(timeout_sec=0.01)

    def cancel(self):
        """
        Cancel this goal only; an empty ID must never cancel other
        clients.
        """
        if self.active_id is not None:
            self.cancel_pub.publish(wire.GoalID(id=self.active_id))

    def _wait_cancel(self, timeout):
        deadline = time.monotonic() + timeout
        while self.result is None and time.monotonic() < deadline:
            self.node.spin_once(timeout_sec=0.01)
        return self.result is not None

    def _start(self, goal, cancel_event):
        self.wait_ready(self.server_timeout)
        self.active_id = uuid4().hex
        self.result = self.feedback = None
        self.status = wire.GoalStatus.PENDING
        if cancel_event is not None and cancel_event.is_set():
            msg = 'Action cancelled before execution'
            raise RuntimeError(msg)
        message = self.goal_type(goal=goal)
        # Leave scheduling stamps zero; host/controller clocks may differ.
        # The unique ID identifies the goal and its cancellation.
        message.goal_id.id = self.active_id
        self.goal_pub.publish(message)

    def _await_result(
        self, timeout, cancel_event, on_feedback, health_check, cancel_timeout
    ):
        deadline = time.monotonic() + timeout
        generation = self.node.generation
        last_feedback = None
        while self.result is None:
            if cancel_event is not None and cancel_event.is_set():
                self.cancel()
                if not self._wait_cancel(cancel_timeout):
                    msg = 'Action cancellation could not be confirmed'
                    raise TimeoutError(msg)
                break
            if time.monotonic() >= deadline:
                msg = 'Controller action timed out'
                raise TimeoutError(msg)
            if self.node.generation != generation:
                msg = 'Action transport changed; explicit restart required'
                raise ConnectionError(msg)
            if time.monotonic() - self.status_at > self.server_timeout:
                msg = 'Controller action server heartbeat disappeared'
                raise ConnectionError(msg)
            if health_check is not None:
                health_check()
            self.node.spin_once(timeout_sec=0.01)
            if (
                self.feedback is not None
                and self.feedback is not last_feedback
            ):
                if on_feedback is not None:
                    on_feedback(self.feedback)
                last_feedback = self.feedback
        return self.result

    def run(  # noqa: PLR0913
        self,
        goal,
        *,
        timeout,
        cancel_event=None,
        on_feedback=None,
        health_check=None,
        cancel_timeout=2.0,
    ):
        positive('timeout', timeout)
        positive('cancel_timeout', cancel_timeout)
        if not self.busy.acquire(blocking=False):
            msg = 'An action goal is already active'
            raise RuntimeError(msg)
        sent = False
        try:
            self._start(goal, cancel_event)
            sent = True
            result = self._await_result(
                timeout,
                cancel_event,
                on_feedback,
                health_check,
                cancel_timeout,
            )
        except BaseException:
            if sent and self.result is None:
                try:
                    self.cancel()
                    if not self._wait_cancel(cancel_timeout):
                        self.node.get_logger().warning(
                            'Native action stop was not confirmed'
                        )
                except (OSError, RuntimeError) as exc:
                    self.node.get_logger().warning(
                        f'Native action cancel delivery unconfirmed: {exc}'
                    )
            raise
        finally:
            self.active_id = None
            self.busy.release()
        return result
