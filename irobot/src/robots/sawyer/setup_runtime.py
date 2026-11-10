"""
Check/install the pinned Sawyer runtime, including Conda when absent.

Runs with system Python (3.10+); imports no ROS, NumPy, or robot SDK. Installs
only when the explicit 'install' command is used. Never contacts the robot.
"""

# CLI output and checked argument-vector subprocesses are intentional.
# ruff: noqa: T201, S603
# Fixed HTTPS installer sources and explicit CLI subprocesses are intentional.
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shlex
import shutil
import subprocess
import sys
import tempfile
import urllib.request
from pathlib import Path

RELEASE = json.loads((Path(__file__).parent / 'installation/bridge-release.json').read_text())
CONDA_EXACT_SPEC_FIELDS = 3
BRIDGE_SPEC = f"sawyer_bridge={RELEASE['version']}={RELEASE['build']}"
MINIFORGE_URL = 'https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh'
PROBE = '''import json, sys
import numpy, rclpy
from ament_index_python.packages import get_package_prefix
from intera_core_msgs.msg import JointCommand, EndpointStates, RobotAssemblyState
from intera_core_msgs.srv import SolvePositionIK, SolvePositionFK
for cls in (JointCommand, EndpointStates, RobotAssemblyState, SolvePositionIK.Request, SolvePositionFK.Request):
    cls.__class__.__import_type_support__()
print(json.dumps({'python': sys.version.split()[0], 'bridge': get_package_prefix('sawyer_bridge'),
                  'numpy': numpy.__version__, 'rclpy': rclpy.__file__}))
'''


def platform_issues():
    issues = []
    if platform.system() != 'Linux' or platform.machine() != 'x86_64':
        issues.append('This bridge release requires Linux x86_64')
    libc, version = platform.libc_ver()
    try:
        supported = libc == 'glibc' and tuple(map(int, version.split('.')[:2])) >= (2, 35)
    except ValueError:
        supported = False
    if not supported:
        issues.append(f'This bridge release requires glibc >= 2.35; found {libc} {version}')
    return issues


def inspect_runtime(prefix, *, probe=True):  # noqa: PLR0912
    """Return installation issues without starting ROS or contacting Sawyer."""
    prefix = Path(prefix).resolve()
    issues = platform_issues()
    metadata = prefix / 'conda-meta'
    if not metadata.is_dir():
        return [*issues, f'No Conda environment at {prefix}']
    records = {}
    try:
        for path in metadata.glob('*.json'):
            record = json.loads(path.read_text())
            records[record['name']] = record
    except (OSError, ValueError, KeyError) as exc:
        return [*issues, f'Cannot read Conda metadata: {exc}']
    expected = {'python': '3.12.', 'numpy': '2.', 'libboost': '1.88.', 'console_bridge': '1.0.'}
    for name, version in expected.items():
        current = records.get(name, {}).get('version', '')
        if not current.startswith(version):
            issues.append(f'{name}: requires {version}x; found {current or "missing"}')
    bridge = records.get('sawyer_bridge', {})
    if bridge.get('version') != RELEASE['version'] or bridge.get('build') != RELEASE['build']:
        issues.append(f'Missing/incompatible bridge; requires {BRIDGE_SPEC}')
    mutex = records.get('ros2-distro-mutex', {})
    if not mutex.get('version', '').startswith('0.9.') or not mutex.get('build', '').startswith('humble_'):
        issues.append('Requires RoboStack Humble mutex 0.9.x humble_*')
    for spec in RELEASE['depends']:
        parts = spec.split()
        if len(parts) == CONDA_EXACT_SPEC_FIELDS and parts[0].startswith('ros-humble-'):
            name, version, build = parts
            current = records.get(name, {})
            if current.get('version') != version or current.get('build') != build:
                issues.append(f'{name}: requires {version} {build}')
    if not (prefix / 'local_setup.bash').is_file():
        issues.append('ROS local_setup.bash is missing')
    for name in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH'):
        if any(p.startswith('/opt/ros/') for p in os.environ.get(name, '').split(':')):
            issues.append(f'{name} contains system ROS; use a fresh terminal with only the Conda runtime')
    if probe and not issues:
        command = [
            'bash',
            '-c',
            'source "$1/local_setup.bash" && exec "$1/bin/python" -I -c "$2"',
            'sawyer-runtime-check',
            str(prefix),
            PROBE,
        ]
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=30, check=False)
            if result.returncode:
                issues.append('ROS/type-support probe failed: ' + (result.stderr or result.stdout).strip())
            else:
                info = json.loads(result.stdout.strip().splitlines()[-1])
                if Path(info['bridge']).resolve() != prefix or not Path(info['rclpy']).resolve().is_relative_to(prefix):
                    issues.append('ROS/bridge resolved outside the selected environment')
        except (OSError, subprocess.TimeoutExpired, ValueError, KeyError, IndexError) as exc:
            issues.append(f'Runtime probe failed: {exc}')
    return issues


def require_current_runtime():
    issues = inspect_runtime(sys.prefix)
    if issues:
        raise RuntimeError(
            'Sawyer runtime is unavailable/incompatible:\n- '
            + '\n- '.join(issues)
            + '\nRun python3 irobot/src/robots/sawyer/setup_runtime.py install --bridge-channel /path/to/release, '
            'then source .sawyer-runtime/activate.sh. See the Sawyer README.'
        )


def validate_channel(channel):
    """Use the release's real local channel, not an invented public package URL."""
    if channel is None:
        msg = 'Provide --bridge-channel /path/to/the/extracted/bridge/release'
        raise ValueError(msg)
    channel = Path(channel).expanduser().resolve()
    artifact = channel / RELEASE['artifact']
    if not artifact.is_file():
        msg = f'Bridge artifact is missing: {artifact}'
        raise ValueError(msg)
    with artifact.open('rb') as stream:
        checksum = hashlib.sha256()
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            checksum.update(chunk)
        digest = checksum.hexdigest()
    if digest != RELEASE['sha256']:
        msg = 'Bridge artifact SHA-256 does not match the pinned release'
        raise ValueError(msg)
    for subdir in ('linux-64', 'noarch'):
        if not (channel / subdir / 'repodata.json').is_file():
            msg = f'Conda channel metadata missing: {subdir}/repodata.json'
            raise ValueError(msg)
    return channel


def find_conda(explicit, runtime_root):
    if explicit and not Path(explicit).expanduser().is_file():
        msg = f'Conda executable not found: {explicit}'
        raise ValueError(msg)
    candidates = [
        explicit,
        os.environ.get('CONDA_EXE'),
        shutil.which('conda'),
        str(runtime_root / 'miniforge/bin/conda'),
    ]
    return next((str(Path(p).expanduser().resolve()) for p in candidates if p and Path(p).expanduser().is_file()), None)


def bootstrap_conda(runtime_root):
    prefix = runtime_root / 'miniforge'
    if prefix.exists():
        msg = f'Incomplete Conda installation at {prefix}; choose another --runtime-root'
        raise ValueError(msg)
    runtime_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='miniforge-', dir=runtime_root) as directory:
        installer = Path(directory) / 'Miniforge3-Linux-x86_64.sh'
        urllib.request.urlretrieve(MINIFORGE_URL, installer)  # noqa: S310
        with urllib.request.urlopen(MINIFORGE_URL + '.sha256', timeout=30) as response:  # noqa: S310
            expected = response.read().decode().split()[0]
        if hashlib.sha256(installer.read_bytes()).hexdigest() != expected:
            msg = 'Miniforge installer SHA-256 mismatch'
            raise ValueError(msg)
        subprocess.run([shutil.which('bash') or '/bin/bash', str(installer), '-b', '-p', str(prefix)], check=True)
    return str(prefix / 'bin/conda')


def installation_command(conda, prefix, channel):
    operation = 'install' if (prefix / 'conda-meta').is_dir() else 'create'
    return [
        conda,
        operation,
        '--yes',
        '--prefix',
        str(prefix),
        '--override-channels',
        '--strict-channel-priority',
        '-c',
        channel.as_uri(),
        '-c',
        'conda-forge',
        '-c',
        'robostack-humble',
        'python=3.12',
        'pip',
        'setuptools>=61',
        'wheel',
        'poetry-core',
        'numpy>=2,<3',
        BRIDGE_SPEC,
    ]


def write_activation(runtime_root, conda, prefix):
    runtime_root.mkdir(parents=True, exist_ok=True)
    hook = Path(conda).parent.parent / 'etc/profile.d/conda.sh'
    script = runtime_root / 'activate.sh'
    script.write_text(
        '# Source in each terminal: source ' + shlex.quote(str(script)) + '\n'
        'source ' + shlex.quote(str(hook)) + '\n'
        'conda activate ' + shlex.quote(str(prefix)) + '\n'
        'source "$CONDA_PREFIX/local_setup.bash"\n'
    )
    return script


def install(args, prefix, runtime_root):
    issues = platform_issues()
    if issues:
        raise ValueError('; '.join(issues))
    project = Path(args.project).expanduser().resolve()
    if not (project / 'pyproject.toml').is_file():
        msg = '--project must point to the source checkout containing pyproject.toml'
        raise ValueError(msg)
    conda = find_conda(args.conda, runtime_root)
    existing_issues = inspect_runtime(prefix)
    channel = validate_channel(args.bridge_channel) if existing_issues else None
    if args.dry_run:
        if conda is None:
            print(f'Would download/check Miniforge from {MINIFORGE_URL} and install to {runtime_root / "miniforge"}')
            conda = str(runtime_root / 'miniforge/bin/conda')
        if channel:
            print(shlex.join(installation_command(conda, prefix, channel)))
        else:
            print('Runtime requirements already satisfied; no Conda transaction needed.')
        print(
            shlex.join(
                [
                    conda,
                    'run',
                    '--prefix',
                    str(prefix),
                    'python',
                    '-m',
                    'pip',
                    'install',
                    '-e',
                    str(project) + '[sawyer]',
                ]
            )
        )
        print('Plan only: no downloads, installation, or robot connection performed.')
        return 0
    conda = conda or bootstrap_conda(runtime_root)
    # Do not install into the Conda base environment by mistake.
    if prefix == Path(conda).parent.parent:
        msg = 'Select a separate --env-prefix; do not install ROS into Conda base'
        raise ValueError(msg)
    if channel:
        subprocess.run(installation_command(conda, prefix, channel), check=True)
    issues = inspect_runtime(prefix)
    if issues:
        raise RuntimeError('Runtime validation failed:\n- ' + '\n- '.join(issues))
    subprocess.run(
        [
            conda,
            'run',
            '--no-capture-output',
            '--prefix',
            str(prefix),
            'python',
            '-m',
            'pip',
            'install',
            '-e',
            str(project) + '[sawyer]',
        ],
        check=True,
    )
    activation = write_activation(runtime_root, conda, prefix)
    print(f'Installed and checked. In each terminal: source {shlex.quote(str(activation))}')
    print('Set ROS_MASTER_URI, ROS_IP, and matching ROS_DOMAIN_ID before launching the bridge.')
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('check', 'install'))
    parser.add_argument('--runtime-root', type=Path, default=Path.cwd() / '.sawyer-runtime')
    parser.add_argument('--env-prefix', type=Path, help='Default: active Conda env, otherwise <runtime-root>/env')
    parser.add_argument('--conda', help='Existing Conda executable; otherwise discover or bootstrap Miniforge')
    parser.add_argument(
        '--bridge-channel', type=Path, help='Extracted local release channel containing linux-64 and noarch'
    )
    parser.add_argument('--project', type=Path, default=Path(__file__).resolve().parents[4])
    parser.add_argument('--dry-run', action='store_true', help='Print installation plan without changing anything')
    args = parser.parse_args(argv)
    runtime_root = args.runtime_root.expanduser().resolve()
    default_prefix = Path(os.environ['CONDA_PREFIX']) if os.environ.get('CONDA_PREFIX') else runtime_root / 'env'
    prefix = (args.env_prefix or default_prefix).expanduser().resolve()
    try:
        if args.command == 'install':
            return install(args, prefix, runtime_root)
        issues = inspect_runtime(prefix)
        if issues:
            print('Runtime requirements missing/incompatible:\n- ' + '\n- '.join(issues))
            print('Run install --bridge-channel /path/to/extracted/release before using Sawyer.')
            return 1
        print(f'PASS: Python 3.12, NumPy 2, RoboStack Humble, and {BRIDGE_SPEC} at {prefix}')
        print('Shared libraries/type support checked; no robot connection performed.')
        return 0  # noqa: TRY300
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f'Setup failed: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
