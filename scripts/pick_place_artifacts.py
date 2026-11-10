"""Task-scoped output ownership and cleanup. Never recursively delete folders."""

import json
import os
from pathlib import Path
import re

if __package__:
    from .pick_place_plan import SCHEMA, atomic_json, read_pose
    from .cartesian_path import pose_error
else:
    from pick_place_plan import SCHEMA, atomic_json, read_pose
    from cartesian_path import pose_error


ARTIFACT_SCHEMA = 'sawyer.pick_place.artifacts.v1'
SUFFIXES = ('.png', '.csv', '.plot.json', '.plot.viewer.log')
STAMP = r'\d{8}T\d{6}_\d{6}Z'


def identity(path):
    return os.path.abspath(path)


def manifest_path(recording):
    path = Path(recording)
    return path.with_name(path.name+'.artifacts.json')


def valid_output(recording, path):
    base = re.escape(Path(recording).stem)
    return any(re.fullmatch(rf'(?:{base}_planned|{base}_replay_{STAMP}|replay_{STAMP}){re.escape(suffix)}',
                           path.name) for suffix in SUFFIXES)


def load_manifest(recording):
    path = manifest_path(recording)
    if not path.exists():
        return []
    data = json.loads(path.read_text())
    if data.get('schema') != ARTIFACT_SCHEMA or data.get('recording_file') != identity(recording):
        raise ValueError(f'Unrecognized artifact ownership manifest: {path}')
    outputs = [Path(value) for value in data['outputs']]
    if any(not p.is_absolute() or not valid_output(recording, p) for p in outputs):
        raise ValueError(f'Unexpected file in artifact ownership manifest: {path}')
    return outputs


def register_outputs(recording, directory, stem):
    outputs = set(load_manifest(recording))
    new = [Path(identity(Path(directory)/(stem+suffix))) for suffix in SUFFIXES]
    if any(not valid_output(recording, path) for path in new):
        raise ValueError('Output filenames must belong to the selected task')
    outputs.update(new)
    atomic_json(manifest_path(recording), dict(schema=ARTIFACT_SCHEMA,
        recording_file=identity(recording), outputs=sorted(str(p) for p in outputs)))


def legacy_outputs(recording, directories):
    """Adopt old generic replay names only when their ownership is unambiguous.

    Older versions saved no owner path. Require this task to be the only recording
    in that directory and require every recorded waypoint to match the plot data.
    """
    recording = Path(recording)
    if not recording.is_file() or recording.is_symlink():
        return []
    try:
        task = json.loads(recording.read_text())
    except (ValueError, OSError):
        return []
    if not isinstance(task, dict) or task.get('schema') != SCHEMA:
        return []
    expected = task.get('waypoints')
    if not isinstance(expected, list) or not expected:
        return []
    result = []
    for directory in directories:
        directory = Path(directory)
        if not directory.is_dir():
            continue
        owners = []
        for candidate in directory.glob('*.json'):
            if candidate.name.endswith(('.plot.json', '.artifacts.json')) or candidate.is_symlink():
                continue
            try:
                if json.loads(candidate.read_text()).get('schema') == SCHEMA:
                    owners.append(identity(candidate))
            except (ValueError, OSError, AttributeError):
                continue
        # An explicitly selected output directory may contain no task file; only
        # the recording's own directory has enough evidence to adopt legacy runs.
        if owners != [identity(recording)]:
            continue
        for plot in directory.glob('replay_*.plot.json'):
            if plot.is_symlink() or not re.fullmatch(rf'replay_{STAMP}\.plot\.json', plot.name):
                continue
            try:
                points = json.loads(plot.read_text())['waypoints']
                matches = len(points) == len(expected) and all(
                    a['name'] == b['name'] and max(pose_error(read_pose(a['pose']), read_pose(b['pose']))) < 1e-7
                    for a,b in zip(points, expected))
                if matches:
                    stem = plot.name[:-len('.plot.json')]
                    result.extend(directory/(stem+suffix) for suffix in SUFFIXES)
            except (ValueError, KeyError, TypeError, OSError):
                continue
    return result


def cleanup_recording(recording, output_dir=None):
    """Called only when a new teaching run starts, after the gripper is ready."""
    recording = Path(recording)
    directories = {recording.parent, Path(output_dir) if output_dir else recording.parent}
    # Validate ownership and build the complete deletion list before unlinking.
    outputs = set(load_manifest(recording))
    outputs.update(legacy_outputs(recording, directories))
    for directory in directories:
        outputs.update(directory/(recording.stem+'_planned'+suffix) for suffix in SUFFIXES)
    outputs.update((recording, manifest_path(recording)))
    removed = []
    for path in sorted(outputs):
        if path.is_dir():
            raise ValueError(f'Refusing to delete a directory: {path}')
    for path in sorted(outputs):
        if path.is_file() or path.is_symlink():
            path.unlink()
            removed.append(str(path))
    return removed
