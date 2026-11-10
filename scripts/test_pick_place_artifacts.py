"""Scoped cleanup and ownership; all deletion tests use disposable directories."""

import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from pick_place_artifacts import (ARTIFACT_SCHEMA, SUFFIXES, cleanup_recording,
    identity, manifest_path, register_outputs)
from pick_place_plan import SCHEMA


def recording(path, x=0):
    data = dict(schema=SCHEMA, waypoints=[dict(name='pick', pose=dict(position=[x,0,0],
        orientation=[0,0,0,1], frame='base'))])
    path.write_text(json.dumps(data))
    return data


def outputs(directory, stem, points=None):
    directory.mkdir(parents=True, exist_ok=True)
    result = []
    for suffix in SUFFIXES:
        path = directory/(stem+suffix)
        path.write_text(json.dumps(dict(waypoints=points)) if suffix == '.plot.json' else 'generated')
        result.append(path)
    return result


class ArtifactTests(unittest.TestCase):
    def test_cleanup_deletes_only_selected_task_and_registered_outputs_in_other_directories(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            task = root/'task.json'
            recording(task)
            other = root/'other.json'
            recording(other, x=1)
            protected = root/'replay_pick_place.py'
            protected.write_text('source code')
            unrelated = root/'notes.json'
            unrelated.write_text('{}')
            planned = outputs(root, 'task_planned')
            stem = 'task_replay_20261102T010203_000000Z'
            generated = outputs(root/'old-custom-output', stem)
            register_outputs(task, root/'old-custom-output', stem)
            register_outputs(task, root/'old-custom-output', stem)  # Idempotent.
            foreign = outputs(root/'old-custom-output', 'other_replay_20261102T010203_000000Z')
            removed = cleanup_recording(task, root/'new-custom-output')
            self.assertIn(str(task), removed)
            self.assertTrue(all(not p.exists() for p in planned+generated))
            self.assertFalse(manifest_path(task).exists())
            self.assertTrue(all(p.exists() for p in [other,protected,unrelated]+foreign))
            self.assertTrue((root/'old-custom-output').is_dir())

    def test_legacy_replay_files_are_matched_by_waypoints(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            task = root/'task.json'
            data = recording(task)
            owned = outputs(root, 'replay_20261102T010203_000000Z', data['waypoints'])
            foreign_points = json.loads(json.dumps(data['waypoints']))
            foreign_points[0]['pose']['position'][0] = 1
            foreign = outputs(root, 'replay_20261102T010204_000000Z', foreign_points)
            cleanup_recording(task)
            self.assertTrue(all(not p.exists() for p in owned))
            self.assertTrue(all(p.exists() for p in foreign))

    def test_legacy_ownership_ambiguity_preserves_outputs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            task = root/'task.json'
            data = recording(task)
            recording(root/'other.json', x=1)
            legacy = outputs(root, 'replay_20261102T010203_000000Z', data['waypoints'])
            cleanup_recording(task)
            self.assertTrue(all(p.exists() for p in legacy))
            self.assertTrue((root/'other.json').exists())

    def test_unexpected_manifest_entry_aborts_before_deleting_anything(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            task = root/'task.json'
            recording(task)
            important = root/'important.txt'
            important.write_text('keep')
            manifest_path(task).write_text(json.dumps(dict(schema=ARTIFACT_SCHEMA,
                recording_file=identity(task), outputs=[str(important)])))
            with self.assertRaises(ValueError):
                cleanup_recording(task)
            self.assertTrue(task.exists())
            self.assertEqual(important.read_text(), 'keep')

    def test_invalid_selected_recording_can_be_replaced_without_adopting_legacy_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            task = root/'task.json'
            task.write_text('interrupted old recording')
            legacy = outputs(root, 'replay_20261102T010203_000000Z')
            cleanup_recording(task)
            self.assertFalse(task.exists())
            self.assertTrue(all(p.exists() for p in legacy))

    def test_recording_cleanup_runs_after_gripper_ready_before_first_capture(self):
        from record_pick_place import record_sequence
        session, cleanup = MagicMock(), MagicMock()
        session.capture_waypoint.side_effect = RuntimeError('first prompt reached')
        def capture(_):
            cleanup.assert_called_once()
            raise RuntimeError('first prompt reached')
        session.capture_waypoint.side_effect = capture
        with patch('builtins.print'), self.assertRaisesRegex(RuntimeError, 'first prompt reached'):
            record_sequence(session, 0, on_start=cleanup)
        session.command_gripper.assert_called_once_with('open')
        session.command_gripper.side_effect = RuntimeError('gripper unavailable')
        cleanup.reset_mock()
        with patch('builtins.print'), self.assertRaisesRegex(RuntimeError, 'gripper unavailable'):
            record_sequence(session, 0, on_start=cleanup)
        cleanup.assert_not_called()


if __name__ == '__main__':
    unittest.main()
