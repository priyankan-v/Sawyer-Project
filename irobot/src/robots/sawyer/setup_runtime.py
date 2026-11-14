"""
Install/check the standalone driver or the optional pinned Humble runtime.
"""

# Explicit installer CLI output and checked argument-vector subprocesses.
# ruff: noqa: T201, S603
from __future__ import annotations

import argparse
import json
import os
import platform
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

INSTALLATION = Path(__file__).parent / 'installation'
HUMBLE_LOCK = INSTALLATION / 'humble-linux-64.explicit.txt'
PROBE = """import json, sys
import numpy
from irobot import SawyerController
assert sys.version_info >= (3, 11), 'Python >=3.11 required'
assert int(numpy.__version__.split('.')[0]) == 2, 'NumPy 2 required'
SawyerController()
print(json.dumps({
    'python': sys.version.split()[0], 'numpy': numpy.__version__
}))
"""
HUMBLE_PROBE = """
assert sys.version_info[:2] == (3, 12), 'Humble ABI requires Python 3.12'
import rclpy
from irobot.src.robots.sawyer.ros2 import SawyerNode
from irobot.src.robots.sawyer.ros2_telemetry import SawyerTelemetry
from control_msgs.action import FollowJointTrajectory
from rclpy.type_support import check_for_type_support
check_for_type_support(FollowJointTrajectory)
"""


def platform_issues():
    """The pinned Humble artifacts target the tested Linux x86_64 platform."""
    if platform.system() != 'Linux' or platform.machine() != 'x86_64':
        return ['The pinned Humble environment requires Linux x86_64']
    return []


def runtime_environment(prefix, overlay=None):
    environment = dict(os.environ)
    for name in (
        'PYTHONPATH',
        'AMENT_PREFIX_PATH',
        'CMAKE_PREFIX_PATH',
        'COLCON_PREFIX_PATH',
        'LD_LIBRARY_PATH',
    ):
        environment.pop(name, None)
    environment.update(
        ROS_DISTRO='humble',
        AMENT_PREFIX_PATH=str(prefix),
        CMAKE_PREFIX_PATH=str(prefix),
        LD_LIBRARY_PATH=str(prefix / 'lib'),
    )
    if overlay is not None:
        environment['PYTHONPATH'] = str(
            overlay / 'lib/python3.12/site-packages'
        )
        environment['LD_LIBRARY_PATH'] = (
            str(overlay / 'lib') + ':' + environment['LD_LIBRARY_PATH']
        )
        environment['AMENT_PREFIX_PATH'] = str(overlay) + ':' + str(prefix)
    return environment


def inspect_runtime(prefix, *, ros2=False, probe=True, overlay=None):
    """
    Inspect/import only; never download, register nodes, or contact Sawyer.
    """
    prefix = Path(prefix).resolve()
    python = prefix / 'bin/python'
    if not python.is_file():
        return [f'No Python environment at {prefix}']
    issues = platform_issues() if ros2 else []
    metadata = prefix / 'conda-meta'
    if ros2:
        records = {
            record['name']: record
            for path in metadata.glob('*.json')
            if (record := json.loads(path.read_text()))
        }
        if (
            not records.get('python', {})
            .get('version', '')
            .startswith('3.12.')
        ):
            issues.append('The pinned Humble runtime requires Python 3.12')
        if 'sawyer_bridge' in records:
            issues.append(
                'Legacy Sawyer bridge installed; select a fresh prefix'
            )
    if probe and not issues:
        try:
            result = subprocess.run(
                [
                    str(python),
                    '-I',
                    '-c',
                    (
                        (
                            'import sys; sys.path.insert(0, '
                            + repr(
                                str(
                                    Path(overlay)
                                    / 'lib/python3.12/site-packages'
                                )
                            )
                            + ')\n'
                        )
                        if overlay is not None
                        else ''
                    )
                    + PROBE
                    + (HUMBLE_PROBE if ros2 else '')
                    + (
                        'from irobot_sawyer_interfaces.action '
                        'import JointStream\n'
                        'check_for_type_support(JointStream)\n'
                        'from irobot.src.robots.sawyer.ros2_interfaces '
                        'import ExtendedInterfaces\n'
                        if overlay is not None
                        else ''
                    ),
                ],
                env=runtime_environment(prefix, overlay),
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
            if result.returncode:
                issues.append((result.stderr or result.stdout).strip())
        except (OSError, subprocess.TimeoutExpired) as exc:
            issues.append(f'Runtime probe failed: {exc}')
    return issues


def require_current_runtime():
    issues = inspect_runtime(sys.prefix)
    if issues:
        raise RuntimeError('\n'.join(issues))


def installation_commands(args, prefix, runtime):
    project = Path(args.project).resolve()
    if args.ros2:
        conda = args.conda or shutil.which('conda')
        if conda is None:
            msg = (
                'Select a Conda executable with --conda for Humble '
                'installation'
            )
            raise ValueError(msg)
        creation = [
            str(conda),
            'create',
            '--yes',
            '--prefix',
            str(prefix),
            '--file',
            str(HUMBLE_LOCK),
        ]
    else:
        creation = [str(args.python), '-m', 'venv', str(prefix)]
    commands = []
    if not (prefix / 'bin/python').is_file():
        commands.append(creation)
    elif args.ros2 and (
        issues := inspect_runtime(prefix, ros2=True, probe=False)
    ):
        raise RuntimeError('\n'.join(issues))
    commands.append(
        [
            str(prefix / 'bin/python'),
            '-m',
            'pip',
            'install',
            '--no-cache-dir',
            str(project) + '[sawyer]',
        ]
    )
    if args.interfaces:
        if not args.ros2:
            msg = '--interfaces requires --ros2'
            raise ValueError(msg)
        cmake = str(prefix / 'bin/cmake')
        build, install = (
            runtime / 'interfaces/build',
            runtime / 'interfaces/install',
        )
        commands.extend(
            [
                [
                    cmake,
                    '-S',
                    str(project / 'ros2/irobot_sawyer_interfaces'),
                    '-B',
                    str(build),
                    '-DCMAKE_PREFIX_PATH=' + str(prefix),
                    '-DPython3_EXECUTABLE=' + str(prefix / 'bin/python'),
                    '-DPYTHON_EXECUTABLE=' + str(prefix / 'bin/python'),
                    '-DCMAKE_INSTALL_PREFIX=' + str(install),
                    '-DBUILD_TESTING=OFF',
                ],
                [cmake, '--build', str(build), '--parallel', '2'],
                [cmake, '--install', str(build)],
            ]
        )
    return commands


def write_activation(prefix, runtime, *, ros2, interfaces):
    lines = [
        '#!/usr/bin/env bash',
        'export PATH=' + shlex.quote(str(prefix / 'bin')) + ':"$PATH"',
    ]
    if ros2:
        lines.append('source ' + shlex.quote(str(prefix / 'setup.bash')))
    if interfaces:
        lines.append(
            'source '
            + shlex.quote(str(runtime / 'interfaces/install/local_setup.bash'))
        )
    runtime.mkdir(parents=True, exist_ok=True)
    (runtime / 'activate.sh').write_text('\n'.join(lines) + '\n')


def install(args, prefix, runtime):
    """Execute only an explicit install; dry-run performs no subprocesses."""
    prefix, runtime = Path(prefix).resolve(), Path(runtime).resolve()
    if args.ros2 and (issues := platform_issues()):
        raise ValueError('\n'.join(issues))
    commands = installation_commands(args, prefix, runtime)
    for command in commands:
        print(shlex.join(command))
        if not args.dry_run:
            environment = (
                runtime_environment(prefix)
                if command[0].endswith('/cmake')
                else None
            )
            subprocess.run(command, check=True, env=environment)
    if args.dry_run:
        return 0
    issues = inspect_runtime(
        prefix,
        ros2=args.ros2,
        overlay=runtime / 'interfaces/install' if args.interfaces else None,
    )
    if issues:
        raise RuntimeError('\n'.join(issues))
    write_activation(
        prefix, runtime, ros2=args.ros2, interfaces=args.interfaces
    )
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('check', 'install'))
    parser.add_argument(
        '--project', type=Path, default=Path(__file__).resolve().parents[4]
    )
    parser.add_argument('--prefix', type=Path)
    parser.add_argument('--runtime', type=Path)
    parser.add_argument('--python', default=sys.executable)
    parser.add_argument('--conda')
    parser.add_argument('--ros2', action='store_true')
    parser.add_argument('--interfaces', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    runtime = args.runtime or args.project / '.sawyer-runtime'
    prefix = args.prefix or runtime / 'envs' / (
        'humble-direct' if args.ros2 else 'python-direct'
    )
    try:
        if args.command == 'install':
            return install(args, prefix, runtime)
        issues = inspect_runtime(
            prefix,
            ros2=args.ros2,
            overlay=runtime / 'interfaces/install'
            if args.interfaces
            else None,
        )
        for issue in issues:
            print(issue, file=sys.stderr)
        if not issues:
            print(
                'Direct Sawyer runtime imports successfully; no robot '
                'contacted'
            )
        return int(bool(issues))
    except (
        OSError,
        ValueError,
        RuntimeError,
        subprocess.SubprocessError,
    ) as exc:
        print(f'Setup failed: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
