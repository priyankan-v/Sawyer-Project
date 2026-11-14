"""
Generate the optional Humble interface package from pinned public schemas.
"""

# Development tool; the plain Python driver never imports generated ROS 2 code.
# ruff: noqa: INP001
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

PACKAGE = 'irobot_sawyer_interfaces'
ROOTS = (
    'intera_core_msgs/RobotAssemblyState',
    'intera_core_msgs/DigitalIOState',
    'intera_core_msgs/EndpointStates',
    'intera_core_msgs/IODeviceStatus',
    'intera_core_msgs/IODeviceConfiguration',
    'intera_core_msgs/JointLimits',
    'intera_core_msgs/InteractionControlCommand',
    'intera_core_msgs/InteractionControlState',
    'intera_motion_msgs/MotionCommandGoal',
    'intera_motion_msgs/MotionCommandResult',
    'intera_motion_msgs/MotionCommandFeedback',
    'intera_core_msgs/CalibrationCommandGoal',
    'intera_core_msgs/CalibrationCommandResult',
    'intera_core_msgs/CalibrationCommandFeedback',
)


def field_name(name):
    return re.sub(r'(?<!^)(?=[A-Z])', '_', name).lower()


def ros2_type(kind):
    base, bracket, suffix = kind.partition('[')
    if base in ('time', 'duration'):
        base = 'builtin_interfaces/' + base.title()
    elif base.startswith(('intera_core_msgs/', 'intera_motion_msgs/')):
        base = PACKAGE + '/' + base.split('/')[1]
    return base + (bracket + suffix if bracket else '')


def declaration(record):
    lines = ['# Generated from the pinned public Intera schema.']
    original = record['definition'].split('===', 1)[0]
    lines.extend(
        line.split('#', 1)[0].strip()
        for line in original.splitlines()
        if re.match(r'^\s*\S+\s+\w+\s*=', line)
    )
    lines.extend(
        f'{ros2_type(kind)} {field_name(name)}'
        for name, kind in record['fields']
    )
    return '\n'.join(lines) + '\n'


def generate(schema, output):
    messages = schema['messages']
    names, pending = set(), list(ROOTS)
    while pending:
        name = pending.pop()
        if name in names:
            continue
        names.add(name)
        pending.extend(
            kind.split('[')[0]
            for _, kind in messages[name]['fields']
            if kind.startswith(('intera_core_msgs/', 'intera_motion_msgs/'))
        )
    for folder in ('msg', 'srv', 'action'):
        (output / folder).mkdir(parents=True, exist_ok=True)
    for name in sorted(names):
        (output / 'msg' / (name.split('/')[1] + '.msg')).write_text(
            declaration(messages[name])
        )
    for package, name in (
        ('intera_motion_msgs', 'MotionCommand'),
        ('intera_core_msgs', 'CalibrationCommand'),
    ):
        text = '---\n'.join(
            declaration(messages[f'{package}/{name}{suffix}'])
            for suffix in ('Goal', 'Result', 'Feedback')
        )
        (output / 'action' / (name + '.action')).write_text(text)
    custom = {
        'srv/CheckJointCollision.srv': """float64[] positions
string tip
---
bool success
string detail
bool[] in_collision
""",
        'msg/StreamCommand.msg': """std_msgs/Header header
string session_id
uint64 sequence
uint8 mode
string[] names
float64[] position
float64[] velocity
float64[] acceleration
float64[] effort
""",
        'action/JointStream.action': """uint8 POSITION=1
uint8 VELOCITY=2
uint8 TORQUE=3
uint8 TRAJECTORY=4
uint8 mode
float64 duration
float64 rate
float64 max_velocity
float64 max_acceleration
float64 max_displacement
float64 effort_fraction
float64 position_speed
float64 lease_timeout
---
bool success
string detail
uint64 samples
float64 max_command_gap
---
string session_id
float64 elapsed
float64[7] position
float64[7] velocity
""",
        'action/InteractionSession.action': """\
irobot_sawyer_interfaces/InteractionControlCommand command
float64 duration
---
bool success
bool activation_confirmed
bool deactivation_confirmed
string detail
---
float64 elapsed
float64[7] position
float64[7] velocity
""",
        'action/MoveCartesian.action': """geometry_msgs/PoseStamped target
float64 duration
float64 speed
float64 max_acceleration
---
bool success
string detail
---
geometry_msgs/PoseStamped actual
""",
        'srv/ForwardKinematics.srv': """float64[] positions
string tip
---
bool success
string detail
geometry_msgs/PoseStamped[] poses
""",
        'srv/InverseKinematics.srv': """geometry_msgs/PoseStamped pose
float64[7] seed
string tip
---
bool success
string detail
float64[7] position
""",
        'srv/ReadParameter.srv': """string name
---
bool success
string detail
string json_value
""",
    }
    for name, text in custom.items():
        (output / name).write_text(text)
    definitions = sorted(
        str(path.relative_to(output))
        for folder in ('msg', 'srv', 'action')
        for path in (output / folder).iterdir()
    )
    paths = '\n'.join(f'  "{name}"' for name in definitions)
    (
        output / 'CMakeLists.txt'
    ).write_text(f"""cmake_minimum_required(VERSION 3.16)
project({PACKAGE})
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${{PROJECT_NAME}}
{paths}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)
install(FILES LICENSE NOTICE DESTINATION share/${{PROJECT_NAME}})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
""")
    dependencies = '\n'.join(
        f'  <depend>{name}</depend>'
        for name in (
            'builtin_interfaces',
            'std_msgs',
            'geometry_msgs',
            'sensor_msgs',
        )
    )
    (output / 'package.xml').write_text(f"""<?xml version="1.0"?>
<package format="3">
  <name>{PACKAGE}</name>
  <version>0.1.0</version>
  <description>Typed Humble interfaces for direct Sawyer.</description>
  <maintainer email="ihuman.research.lab@gmail.com">iHuman Lab</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
{dependencies}
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export><build_type>ament_cmake</build_type></export>
</package>
""")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    schema = json.loads(
        (Path(__file__).parent / 'wire_schema.json').read_text()
    )
    generate(schema, args.output)


if __name__ == '__main__':
    main()
