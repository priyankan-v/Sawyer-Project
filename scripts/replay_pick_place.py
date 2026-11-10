#!/usr/bin/env python3
"""Replay a saved pick/place plan; approach pick, then wait for Enter to start.

Default is read-only planning/preview. --execute opens the gripper, approaches
pick, and enables the interactive pick/place execution.
"""

import argparse
from datetime import datetime, timezone
import sys

if __package__:
    from .pick_place_plan import (RELEASE_DELAY, read_task, retime_segment, segment_from_plan,
                                  Segment, SegmentSequence, validate_segment, replan_from_pick, atomic_json)
    from .pick_place_runtime import (Session, add_arguments, motion_options, limits_for,
        neutral_pose, initialize, shutdown, plot_python, write_outputs, plot_payload)
    from .pick_place_visualization import LivePlot, show_saved_plot
    from .pick_place_artifacts import register_outputs
else:
    from pick_place_plan import (RELEASE_DELAY, read_task, retime_segment, segment_from_plan,
                                 Segment, SegmentSequence, validate_segment, replan_from_pick, atomic_json)
    from pick_place_runtime import (Session, add_arguments, motion_options, limits_for,
        neutral_pose, initialize, shutdown, plot_python, write_outputs, plot_payload)
    from pick_place_visualization import LivePlot, show_saved_plot
    from pick_place_artifacts import register_outputs


def build_parser():
    parser = add_arguments(argparse.ArgumentParser(description=__doc__))
    parser.add_argument('--no-live-plot', dest='live_plot', action='store_false',
                        help='Disable the live 3D window during execution')
    parser.add_argument('--no-show-plot', dest='show_plot', action='store_false',
                        help='Close the live window afterward and skip the final interactive viewer')
    return parser


def prepare_replay(session, points, segments):
    """Validate saved paths, approach pick continuously, and connect the joint chain."""
    neutral = neutral_pose()
    if max(abs(a-b) for a, b in zip(neutral, segments[-1].trajectory.end)) > 1e-8:
        raise ValueError('Configured neutral pose changed; record and plan again')
    with session.planner() as planner:
        velocity, acceleration = limits_for(session.arm, session.options)
        # A reduced live limit slows the reference rather than violating it.
        segments = tuple(retime_segment(s, velocity, acceleration) for s in segments)
        checked = []
        for segment in segments:
            try:
                references = validate_segment(segment, planner)
            except ValueError as exc:
                raise ValueError(f'Saved segment {segment.name}: {exc}') from exc
            checked.append(Segment(segment.name, segment.trajectory, segment.line, references))
        pose, joints = session.current_state()
        approach_limits = checked[0].trajectory
        print('Planning continuous Cartesian approach to pick (position and orientation).')
        try:
            # Preserve IK continuation from the actual arm posture. Biasing every
            # seed toward recorded joints can force a branch switch even when
            # the EE is already close to pick. Matching pose is sufficient here.
            plan = planner.plan_pose(pose, points[0].pose, joints,
                approach_limits.max_velocity, approach_limits.max_acceleration)
        except ValueError as exc:
            raise ValueError(f'Approach to pick: {exc}') from exc
        approach = segment_from_plan('approach_pick', plan)
        approach = Segment(approach.name, approach.trajectory, approach.line, validate_segment(approach, planner))
        difference = max(abs(a-b) for a, b in zip(approach.trajectory.end, checked[0].trajectory.start))
        # Require reference continuity much more tightly than execution's start
        # tolerance. Even a small branch offset must not become a command step.
        if difference > 1e-8:
            print(f'Pick posture differs from saved joints by up to {difference:.4f} rad; '
                  'connecting the recorded Cartesian path to this posture.')
            checked = replan_from_pick(checked, planner, approach.trajectory.end)
    # Normalize limits/timings for each uninterrupted controller stream before
    # previewing, plotting, opening the gripper, or issuing any motion command.
    settings = session.options
    approach = SegmentSequence((approach,), settings.settle_time, settings.timeout).segments[0]
    checked = (SegmentSequence(checked[:-2], settings.settle_time, settings.timeout).segments
               + SegmentSequence(checked[-2:], settings.settle_time, settings.timeout).segments)
    return approach, checked


def replay_sequence(session, approach, segments):
    session.command_gripper('open')
    session.execute_segment(approach)
    session.wait_for_enter('At pick. Reset/position the object; press Enter to start replay')
    session.assert_start(segments[0])
    session.command_gripper('close')
    session.execute_segments(segments[:-2])
    session.command_gripper('open')
    print('Release confirmed. Waiting 2 seconds before lifting.')
    session.wait_release_delay(RELEASE_DELAY)
    session.execute_segments(segments[-2:])
    print('Pick/place complete; neutral reached.')


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        data, points, segments = read_task(args.file)
        if args.tip and args.tip != data['tip']:
            raise ValueError('--tip must match the tool used for recording')
        options = motion_options(args, data['settings'])
        renderer = plot_python()
    except (OSError, KeyError, TypeError, ValueError, RuntimeError) as exc:
        parser.error(str(exc))
    session, result, plotted, live = None, 0, (), None
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    output_dir, stem = args.output_dir or args.file.parent, args.file.stem+'_replay_'+stamp
    plot_data = output_dir/(stem+'.plot.json')
    try:
        initialize()
        session = Session(options, args.gripper_timeout)
        approach, segments = prepare_replay(session, points, segments)
        plotted = (approach,) + segments
        for segment in plotted:
            print(f'{segment.name}: {segment.trajectory.duration:.3f} s reference')
        register_outputs(args.file, output_dir, stem)
        if options.execute:
            if args.live_plot:
                atomic_json(plot_data, plot_payload(points, plotted))
                candidate = LivePlot(renderer, plot_data)
                if candidate.start(pump=session.pump):
                    live = session.live_plot = candidate
            replay_sequence(session, approach, segments)
        else:
            print('Preview complete. Add --execute to approach pick and start interactive replay.')
    except (KeyboardInterrupt, EOFError):
        print('\nReplay cancelled; stopping arm.')
        result = 130
    except Exception as exc:
        print(f'Replay failed: {exc}', file=sys.stderr)
        result = 1
    finally:
        try:
            shutdown(session)
        finally:
            # Save partial measurements on failure, after stopping arm commands.
            if session is not None and (plotted or session.rows):
                try:
                    write_outputs(output_dir, stem,
                                  points, plotted, session.rows, renderer=renderer)
                except Exception as exc:
                    print(f'Plot/export failed: {exc}', file=sys.stderr)
                    if result == 0:
                        result = 1
                finally:
                    if plot_data.exists():
                        try:
                            displayed = live.finish(plot_data, keep_open=args.show_plot) if live else False
                            if args.show_plot and not displayed:
                                show_saved_plot(renderer, plot_data)
                        except Exception as exc:
                            print(f'Interactive viewer unavailable: {exc}. Saved data: {plot_data}', file=sys.stderr)
    return result


if __name__ == '__main__':
    sys.exit(main())
