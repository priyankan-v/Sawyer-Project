"""
Install only explicitly; missing dependencies and ABI mismatches fail early.
"""

# The installer uses controlled subprocess fixtures and inspects its own probe.
# ruff: noqa: SLF001
from __future__ import annotations

import json
from argparse import Namespace
from unittest.mock import patch

import pytest

from irobot.src.robots.sawyer import setup_runtime as setup


def test_missing_environment_runs_no_subprocess(tmp_path):
    with patch.object(setup.subprocess, 'run') as run:
        assert 'No Python environment' in setup.inspect_runtime(tmp_path)[0]
    run.assert_not_called()


def test_plain_dry_run_never_installs_or_contacts_robot(tmp_path, capsys):
    args = Namespace(
        project=tmp_path,
        ros2=False,
        interfaces=False,
        python='python3.12',
        conda=None,
        dry_run=True,
    )
    with patch.object(setup.subprocess, 'run') as run:
        assert setup.install(args, tmp_path / 'env', tmp_path / 'runtime') == 0
    run.assert_not_called()
    assert not (tmp_path / 'runtime').exists()
    assert '[sawyer]' in capsys.readouterr().out


def test_humble_dry_run_uses_only_the_direct_dependency_lock(tmp_path, capsys):
    args = Namespace(
        project=tmp_path,
        ros2=True,
        interfaces=True,
        python='python3.12',
        conda='/conda',
        dry_run=True,
    )
    with (
        patch.object(setup.subprocess, 'run') as run,
        patch.object(setup, 'platform_issues', return_value=[]),
    ):
        assert setup.install(args, tmp_path / 'env', tmp_path / 'runtime') == 0
    run.assert_not_called()
    output = capsys.readouterr().out
    assert 'humble-linux-64.explicit.txt' in output
    assert 'irobot_sawyer_interfaces' in output
    assert 'sawyer_bridge' not in output
    assert 'sawyer_bridge' not in setup.HUMBLE_LOCK.read_text()


def test_humble_rejects_mixed_abi_and_legacy_runtime_before_import(tmp_path):
    (tmp_path / 'bin').mkdir()
    (tmp_path / 'bin/python').touch()
    metadata = tmp_path / 'conda-meta'
    metadata.mkdir()
    for name, version in [('python', '3.10.1'), ('sawyer_bridge', '0.1.0')]:
        (metadata / (name + '.json')).write_text(
            json.dumps({'name': name, 'version': version})
        )
    with (
        patch.object(setup.subprocess, 'run') as run,
        patch.object(setup, 'platform_issues', return_value=[]),
    ):
        issues = setup.inspect_runtime(tmp_path, ros2=True)
    assert any('Python 3.12' in issue for issue in issues)
    assert any('Legacy' in issue for issue in issues)
    run.assert_not_called()


def test_interface_install_requires_humble_before_any_subprocess(tmp_path):
    args = Namespace(
        project=tmp_path,
        ros2=False,
        interfaces=True,
        python='python3.12',
        conda=None,
        dry_run=False,
    )
    with (
        patch.object(setup.subprocess, 'run') as run,
        pytest.raises(ValueError, match='requires --ros2'),
    ):
        setup.install(args, tmp_path / 'env', tmp_path / 'runtime')
    run.assert_not_called()
