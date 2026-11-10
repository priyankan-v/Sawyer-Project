"""Mixed-publisher JointState regression checks without ROS."""

# Numeric regression bounds and controlled private fixtures are intentional.
# ruff: noqa: PLR2004, SLF001, PT009, SIM117
from __future__ import annotations

import unittest

import pytest

from irobot.src.robots.sawyer.core.base import JointFeedback


class FeedbackTests(unittest.TestCase):
    def test_unrelated_message_does_not_clear_or_refresh_arm(self):
        feedback = JointFeedback(['j0', 'j1'])
        feedback.update(['j0', 'j1'], [1, 2], [0, 0], 1)
        before = feedback.snapshot()
        assert not feedback.update(['gripper'], [0], [0], 1.1)
        assert feedback.snapshot() == before
        assert not feedback.issues(1.1, 0.2)
        assert feedback.issues(1.3, 0.2)

    def test_partial_arm_messages_merge_by_name(self):
        feedback = JointFeedback(['j0', 'j1'])
        feedback.update(['j1', 'head'], [2, 10], [0.2, 0], 1)
        assert feedback.snapshot()[0] is None
        feedback.update(['j0'], [1], [0.1], 1.01)
        assert feedback.snapshot() == ((1, 2), (0.1, 0.2), 1)

    def test_positions_and_velocities_have_independent_freshness(self):
        feedback = JointFeedback(['j0'])
        feedback.update(['j0'], [1], [], 1)
        assert 'missing velocity: j0' in feedback.issues(1, 0.2)
        feedback.update(['j0'], [], [0.1], 1.01)
        assert feedback.snapshot() == ((1,), (0.1,), 1)
        feedback.update(['j0'], [2], [], 1.3)
        issues = '; '.join(feedback.issues(1.3, 0.2))
        assert 'stale velocity' in issues
        assert 'stale position' not in issues

    def test_other_joint_updates_cannot_hide_a_stale_joint(self):
        feedback = JointFeedback(['j0', 'j1'])
        feedback.update(['j0', 'j1'], [1, 2], [0, 0], 1)
        feedback.update(['j0'], [1], [0], 1.3)
        assert feedback.snapshot()[2] == 1
        issues = '; '.join(feedback.issues(1.3, 0.2))
        assert 'j1 (0.300s old)' in issues
        assert 'j0 (' not in issues

    def test_invalid_relevant_message_does_not_commit_or_refresh(self):
        feedback = JointFeedback(['j0'])
        feedback.update(['j0'], [1], [0], 1)
        before = feedback.snapshot()
        for names, positions, velocities in (
            (['j0'], [2], [0, 0]),
            (['j0'], [float('nan')], [0]),
            (['j0', 'j0'], [2, 2], [0, 0]),
        ):
            with self.subTest(names=names, positions=positions), pytest.raises(ValueError, match='.'):
                feedback.update(names, positions, velocities, 1.1)
            assert feedback.snapshot() == before

    def test_empty_or_nonfinite_unrelated_fields_do_not_invalidate_arm(self):
        feedback = JointFeedback(['j0'])
        feedback.update(['j0'], [1], [0], 1)
        assert not feedback.update(['gripper'], [], [], 1.1)
        feedback.update(['j0', 'head'], [2, float('nan')], [0, float('nan')], 1.1)
        assert feedback.snapshot() == ((2,), (0,), 1.1)

    def test_position_hold_requires_only_fresh_positions(self):
        feedback = JointFeedback(['j0'])
        feedback.update(['j0'], [1], [0], 1)
        feedback.update(['j0'], [2], [], 1.3)
        assert feedback.issues(1.3, 0.2)
        assert feedback.fresh_positions(1.3, 0.2) == (2,)
        assert feedback.fresh_positions(1.6, 0.2) is None


if __name__ == '__main__':
    unittest.main()
