"""Mixed-publisher JointState regression checks without ROS."""

import unittest

from joint_feedback import JointFeedback


class FeedbackTests(unittest.TestCase):
    def test_unrelated_message_does_not_clear_or_refresh_arm(self):
        feedback = JointFeedback(['j0', 'j1'])
        feedback.update(['j0', 'j1'], [1, 2], [0, 0], 1)
        before = feedback.snapshot()
        self.assertFalse(feedback.update(['gripper'], [0], [0], 1.1))
        self.assertEqual(feedback.snapshot(), before)
        self.assertFalse(feedback.issues(1.1, .2))
        self.assertTrue(feedback.issues(1.3, .2))

    def test_partial_arm_messages_merge_by_name(self):
        feedback = JointFeedback(['j0', 'j1'])
        feedback.update(['j1', 'head'], [2, 10], [.2, 0], 1)
        self.assertIsNone(feedback.snapshot()[0])
        feedback.update(['j0'], [1], [.1], 1.01)
        self.assertEqual(feedback.snapshot(), ((1, 2), (.1, .2), 1))

    def test_positions_and_velocities_have_independent_freshness(self):
        feedback = JointFeedback(['j0'])
        feedback.update(['j0'], [1], [], 1)
        self.assertIn('missing velocity: j0', feedback.issues(1, .2))
        feedback.update(['j0'], [], [.1], 1.01)
        self.assertEqual(feedback.snapshot(), ((1,), (.1,), 1))
        feedback.update(['j0'], [2], [], 1.3)
        issues = '; '.join(feedback.issues(1.3, .2))
        self.assertIn('stale velocity', issues)
        self.assertNotIn('stale position', issues)

    def test_other_joint_updates_cannot_hide_a_stale_joint(self):
        feedback = JointFeedback(['j0', 'j1'])
        feedback.update(['j0', 'j1'], [1, 2], [0, 0], 1)
        feedback.update(['j0'], [1], [0], 1.3)
        self.assertEqual(feedback.snapshot()[2], 1)
        issues = '; '.join(feedback.issues(1.3, .2))
        self.assertIn('j1 (0.300s old)', issues)
        self.assertNotIn('j0 (', issues)

    def test_invalid_relevant_message_does_not_commit_or_refresh(self):
        feedback = JointFeedback(['j0'])
        feedback.update(['j0'], [1], [0], 1)
        before = feedback.snapshot()
        for names, positions, velocities in ((['j0'], [2], [0, 0]),
                                             (['j0'], [float('nan')], [0]),
                                             (['j0', 'j0'], [2, 2], [0, 0])):
            with self.subTest(names=names, positions=positions), self.assertRaises(ValueError):
                feedback.update(names, positions, velocities, 1.1)
            self.assertEqual(feedback.snapshot(), before)

    def test_empty_or_nonfinite_unrelated_fields_do_not_invalidate_arm(self):
        feedback = JointFeedback(['j0'])
        feedback.update(['j0'], [1], [0], 1)
        self.assertFalse(feedback.update(['gripper'], [], [], 1.1))
        feedback.update(['j0', 'head'], [2, float('nan')], [0, float('nan')], 1.1)
        self.assertEqual(feedback.snapshot(), ((2,), (0,), 1.1))

    def test_position_hold_requires_only_fresh_positions(self):
        feedback = JointFeedback(['j0'])
        feedback.update(['j0'], [1], [0], 1)
        feedback.update(['j0'], [2], [], 1.3)
        self.assertTrue(feedback.issues(1.3, .2))
        self.assertEqual(feedback.fresh_positions(1.3, .2), (2,))
        self.assertIsNone(feedback.fresh_positions(1.6, .2))


if __name__ == '__main__':
    unittest.main()
