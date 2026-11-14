"""Generate portable wire metadata using ROS 1 genmsg at development time."""

# This is a standalone development tool, not a runtime package.
# ruff: noqa: INP001

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import genmsg
from genmsg.gentools import compute_full_text, compute_md5
from genmsg.msg_loader import (
    load_depends,
    load_msg_by_type,
    load_msg_from_file,
    load_msg_from_string,
    load_srv_from_file,
)


def message_record(context, spec):
    """Retain original wire names and constants from the source definition."""
    return {
        'fields': list(zip(spec.names, spec.types, strict=True)),
        'constants': {item.name: item.val for item in spec.constants},
        'md5': compute_md5(context, spec),
        'definition': compute_full_text(context, spec),
    }


def action_messages(context, base, definition):
    parts = definition.split('---')
    action_specs = []
    for suffix, text in zip(
        ('Goal', 'Result', 'Feedback'), parts, strict=True
    ):
        action_specs.append(load_msg_from_string(context, text, base + suffix))
    for suffix, text in (
        (
            'ActionGoal',
            'Header header\nactionlib_msgs/GoalID goal_id\n'
            f'{base}Goal goal',
        ),
        (
            'ActionResult',
            'Header header\nactionlib_msgs/GoalStatus '
            f'status\n{base}Result result',
        ),
        (
            'ActionFeedback',
            'Header header\nactionlib_msgs/GoalStatus '
            f'status\n{base}Feedback feedback',
        ),
        (
            'Action',
            f'{base}ActionGoal action_goal\n'
            f'{base}ActionResult action_result\n'
            f'{base}ActionFeedback action_feedback',
        ),
    ):
        action_specs.append(load_msg_from_string(context, text, base + suffix))
    return action_specs


def generate(intera_common, ros_share):
    packages = ('intera_core_msgs', 'intera_motion_msgs')
    search = {name: [str(intera_common / name / 'msg')] for name in packages}
    for name in (
        'std_msgs',
        'geometry_msgs',
        'sensor_msgs',
        'diagnostic_msgs',
        'actionlib_msgs',
    ):
        search[name] = [str(ros_share / name / 'msg')]
    standard = Path(__file__).parent / 'standard'
    search['tf2_msgs'] = [str(standard / 'tf2_msgs' / 'msg')]
    context = genmsg.MsgContext.create_default()
    context.set_depends('time', [])
    context.set_depends('duration', [])
    services, sources = {}, {}
    names = {'time', 'duration'}
    for name in (
        'std_msgs/Float64',
        'std_msgs/Bool',
        'std_msgs/Empty',
        'sensor_msgs/Image',
        'sensor_msgs/CameraInfo',
        'diagnostic_msgs/DiagnosticArray',
        'actionlib_msgs/GoalID',
        'actionlib_msgs/GoalStatusArray',
        'tf2_msgs/TFMessage',
    ):
        spec = load_msg_by_type(context, name, search)
        load_depends(context, spec, search)
        names.update([name, *context.get_all_depends(name)])
    for package in packages:
        for path in sorted((intera_common / package / 'msg').glob('*.msg')):
            name = f'{package}/{path.stem}'
            spec = load_msg_from_file(context, str(path), name)
            load_depends(context, spec, search)
            names.update([name, *context.get_all_depends(name)])
            sources[f'{package}/msg/{path.name}'] = hashlib.sha256(
                path.read_bytes()
            ).hexdigest()
        for path in sorted((intera_common / package / 'srv').glob('*.srv')):
            name = f'{package}/{path.stem}'
            spec = load_srv_from_file(context, str(path), name)
            load_depends(context, spec, search)
            for part in (spec.request, spec.response):
                names.update(
                    [part.full_name, *context.get_all_depends(part.full_name)]
                )
            services[name] = {
                'md5': compute_md5(context, spec),
                'request': spec.request.full_name,
                'response': spec.response.full_name,
            }
            sources[f'{package}/srv/{path.name}'] = hashlib.sha256(
                path.read_bytes()
            ).hexdigest()
    for package in packages:
        for path in sorted(
            (intera_common / package / 'action').glob('*.action')
        ):
            base = f'{package}/{path.stem}'
            action_specs = action_messages(context, base, path.read_text())
            for spec in action_specs:
                load_depends(context, spec, search)
                names.update(
                    [spec.full_name, *context.get_all_depends(spec.full_name)]
                )
            sources[f'{package}/action/{path.name}'] = hashlib.sha256(
                path.read_bytes()
            ).hexdigest()
    messages = {
        name: message_record(context, context.get_registered(name))
        for name in sorted(names)
    }
    for name in names:
        if '/' not in name:
            continue
        package, message = name.split('/', 1)
        if package in packages:
            continue
        root = standard if package == 'tf2_msgs' else ros_share
        path = root / package / 'msg' / (message + '.msg')
        sources[f'{package}/msg/{path.name}'] = hashlib.sha256(
            path.read_bytes()
        ).hexdigest()
    return {'messages': messages, 'services': services, 'sources': sources}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--intera-common', required=True, type=Path)
    parser.add_argument('--ros-share', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    data = generate(args.intera_common, args.ros_share)
    args.output.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n')


if __name__ == '__main__':
    main()
