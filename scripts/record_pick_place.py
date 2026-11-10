#!/usr/bin/env python3
"""Teach pick/middle/place poses interactively, then immediately plan and save.

Opens the gripper at startup. This script commands the gripper but never moves
the arm. Hand-guide each pose, release the cuff, then press Enter to record.
"""

import argparse
from dataclasses import asdict, replace
import sys

if __package__:
    from .cartesian_path import pose_error
    from .pick_place_plan import atomic_json, recording_data, plan_task, segment_data
    from .pick_place_runtime import (Session, add_arguments, motion_options, limits_for,
        neutral_pose, initialize, shutdown, plot_python, write_outputs)
    from .pick_place_artifacts import cleanup_recording, register_outputs
else:
    from cartesian_path import pose_error
    from pick_place_plan import atomic_json, recording_data, plan_task, segment_data
    from pick_place_runtime import (Session, add_arguments, motion_options, limits_for,
        neutral_pose, initialize, shutdown, plot_python, write_outputs)
    from pick_place_artifacts import cleanup_recording, register_outputs


def build_parser():
    parser = add_arguments(argparse.ArgumentParser(description=__doc__), recording=True)
    parser.add_argument('--middle-points', type=int, required=True, help='Number of middle points, excluding pick/place (>=0)')
    return parser


def record_sequence(session, middle_points, on_start=None):
    print('Opening the gripper for teaching.')
    session.command_gripper('open')
    if on_start is not None:
        on_start()
    points = [session.capture_waypoint('pick')]
    session.wait_for_enter('Position the object; press Enter to close the gripper')
    # Detect accidental arm movement before accepting the teaching grasp.
    pose, _ = session.current_state()
    if any(error > limit for error, limit in zip(pose_error(pose, points[0].pose), (.003, .02))):
        raise RuntimeError('Pick pose changed before grasp; restart recording')
    session.command_gripper('close')
    for i in range(1, middle_points+1):
        points.append(session.capture_waypoint(f'middle_{i}'))
    points.append(session.capture_waypoint('place'))
    session.wait_for_enter('Press Enter to open the gripper at place and finish recording')
    pose, _ = session.current_state()
    if any(error > limit for error, limit in zip(pose_error(pose, points[-1].pose), (.003, .02))):
        raise RuntimeError('Place pose changed before release; restart recording')
    session.command_gripper('open')
    return tuple(points)


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.middle_points < 0:
        parser.error('--middle-points must be >=0')
    try:
        options = motion_options(args)
        renderer = plot_python()
    except (ValueError, RuntimeError) as exc:
        parser.error(str(exc))
    session, result = None, 0
    try:
        initialize()
        session = Session(options, args.gripper_timeout)
        def clear_previous():
            removed = cleanup_recording(args.file, args.output_dir)
            print(f'New recording: removed {len(removed)} previous task/plot/replay files for {args.file}.')
        points = record_sequence(session, args.middle_points, on_start=clear_previous)
        settings = asdict(replace(options, tip=session.tip))
        data = recording_data(points, session.tip, settings, asdict(session.arm.limits))
        # Preserve teaching even if IK/FK planning subsequently fails. Replay
        # accepts only an artifact whose status has become "planned".
        atomic_json(args.file, data)
        print(f'Recording complete. Planning transfer, 20 cm lift and neutral return; raw poses: {args.file}')
        neutral = neutral_pose()
        with session.planner() as planner:
            velocity, acceleration = limits_for(session.arm, options)
            segments = plan_task(points, planner, velocity, acceleration, neutral)
        data.update(status='planned', segments=[segment_data(s) for s in segments])
        atomic_json(args.file, data)
        for segment in segments:
            print(f'{segment.name}: {segment.trajectory.duration:.3f} s minimum quintic reference')
        print(f'Replay plan saved: {args.file}')
        register_outputs(args.file, args.output_dir or args.file.parent, args.file.stem+'_planned')
        write_outputs(args.output_dir or args.file.parent, args.file.stem+'_planned',
                      points, segments, renderer=renderer)
    except (KeyboardInterrupt, EOFError):
        print('\nRecording cancelled.')
        result = 130
    except Exception as exc:
        print(f'Recording/planning failed: {exc}', file=sys.stderr)
        result = 1
    finally:
        shutdown(session)
    return result


if __name__ == '__main__':
    sys.exit(main())
