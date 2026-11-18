"""Native Intera motion and calibration action coordination."""

from __future__ import annotations

import time
from copy import deepcopy
from dataclasses import asdict

from ..communication import wire
from ..communication.actions import NativeActionClient
from ..planning.native import joint_trajectory, validate_trajectory


def run_trajectory(  # noqa: PLR0913
    node,
    actions,
    trajectory,
    *,
    execute=False,
    plan_only=False,
    timeout=60.0,
    cancel_event=None,
    on_feedback=None,
):
    if type(execute) is not bool or type(plan_only) is not bool:
        msg = 'execute and plan_only must be booleans'
        raise TypeError(msg)
    trajectory = deepcopy(trajectory)
    validate_trajectory(
        trajectory,
        limits=node.limits,
        joints=node.joints,
        frame=node.config.frame,
        current_pose=node.wait_pose,
        position_tolerance=node.config.planning_position_tolerance,
        orientation_tolerance=node.config.planning_orientation_tolerance,
        joint_tolerance=node.config.planning_joint_tolerance,
    )
    goal = wire.MotionCommandGoal(
        command='generate' if plan_only else 'start', trajectory=trajectory
    )
    if not execute:
        return {'executed': False, 'goal': asdict(goal)}
    if (
        trajectory.trajectory_options.interaction_control
        and not plan_only
        and not node.config.allow_unvalidated_modes
    ):
        msg = (
            'Native interaction trajectories require explicit '
            'allow_unvalidated_modes'
        )
        raise RuntimeError(msg)
    node.cancel_requested.clear()
    node.check_ready()
    return actions.run(
        goal,
        timeout=timeout,
        cancel_event=cancel_event,
        on_feedback=on_feedback,
    )


class MotionActions:
    """Lazy clients tied to one connected driver session."""

    def __init__(self, node, config):
        self.node, self.config = node, config
        self.clients = {}

    def client(self, name):
        if name not in self.clients:
            namespace, action_type = {
                'motion': (
                    self.config.native_motion_action,
                    'intera_motion_msgs/MotionCommand',
                ),
                'calibration': (
                    self.config.calibration_action,
                    'intera_core_msgs/CalibrationCommand',
                ),
            }[name]
            self.clients[name] = NativeActionClient(
                self.node,
                namespace,
                action_type,
                server_timeout=self.config.action_server_timeout,
            )
        return self.clients[name]

    def check_health(self):
        self.node.check_ready()
        if self.node.cancel_requested.is_set():
            msg = 'Controller action cancelled'
            raise RuntimeError(msg)

    def run(
        self,
        goal,
        *,
        name='motion',
        timeout=60.0,
        cancel_event=None,
        on_feedback=None,
    ):
        result = self.client(name).run(
            goal,
            timeout=timeout,
            cancel_event=cancel_event,
            on_feedback=on_feedback,
            health_check=self.check_health,
        )
        return asdict(result) | {
            'succeeded': result.succeeded and bool(result.result.result)
        }


def run_joint_path(  # noqa: PLR0913
    node,
    actions,
    positions,
    *,
    execute=False,
    plan_only=False,
    speed=0.05,
    max_acceleration=0.1,
    timeout=60.0,
    cancel_event=None,
    on_feedback=None,
):
    """
    Preview, generate on the controller, or explicitly execute a joint
    path.
    """
    if type(execute) is not bool or type(plan_only) is not bool:
        msg = 'execute and plan_only must be booleans'
        raise TypeError(msg)
    trajectory = joint_trajectory(
        positions,
        start=node.joints,
        limits=node.limits,
        tip=node.config.tip,
        speed=speed,
        max_acceleration=max_acceleration,
    )
    goal = wire.MotionCommandGoal(
        command='generate' if plan_only else 'start', trajectory=trajectory
    )
    if not execute:
        return {'executed': False, 'goal': asdict(goal)}
    node.cancel_requested.clear()
    node.check_ready()
    report = actions.run(
        goal,
        timeout=timeout,
        cancel_event=cancel_event,
        on_feedback=on_feedback,
    )
    report.update(executed=not plan_only, planned_only=plan_only)
    if report['succeeded'] and not plan_only:
        node.spin_until(time.monotonic() + node.config.callback_timeout)
        node.check_ready()
        report['max_final_error_rad'] = max(
            abs(a - b)
            for a, b in zip(
                trajectory.waypoints[-1].joint_positions,
                node.joints,
                strict=True,
            )
        )
        report['arrival_confirmed'] = (
            report['max_final_error_rad'] <= node.config.position_tolerance
        )
    return report
