"""Dependency installation paths tested without downloads or package changes."""

# Numeric regression bounds and controlled private fixtures are intentional.
# ruff: noqa: PLR2004, SLF001, PT009, SIM117
from __future__ import annotations

import hashlib
import io
import json
import tempfile
import unittest
from argparse import Namespace
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

import pytest

from irobot.src.robots.sawyer import setup_runtime as setup


class SetupTests(unittest.TestCase):
    def test_missing_environment_is_reported_without_subprocess(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(setup.subprocess, 'run') as run:
            issues = setup.inspect_runtime(directory)
        assert any('No Conda environment' in issue for issue in issues)
        run.assert_not_called()

    def test_abi_mismatch_is_rejected_before_import_probe(self):
        with tempfile.TemporaryDirectory() as directory:
            meta = Path(directory) / 'conda-meta'
            meta.mkdir()
            (meta / 'python.json').write_text(json.dumps({'name': 'python', 'version': '3.11.1'}))
            with patch.object(setup.subprocess, 'run') as run:
                issues = setup.inspect_runtime(directory)
            run.assert_not_called()
        assert any('python:' in issue for issue in issues)
        assert any('Missing/incompatible bridge' in issue for issue in issues)

    def test_channel_requires_actual_artifact_checksum_and_repodata(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'linux-64').mkdir()
            (root / 'noarch').mkdir()
            artifact = root / setup.RELEASE['artifact']
            artifact.write_bytes(b'test artifact')
            for subdir in ('linux-64', 'noarch'):
                (root / subdir / 'repodata.json').write_text('{}')
            with pytest.raises(ValueError, match='SHA-256'):
                setup.validate_channel(root)
            release = dict(setup.RELEASE, sha256=hashlib.sha256(b'test artifact').hexdigest())
            with patch.object(setup, 'RELEASE', release):
                assert setup.validate_channel(root) == root
                (root / 'noarch/repodata.json').unlink()
                with pytest.raises(ValueError, match='metadata'):
                    setup.validate_channel(root)

    def test_fresh_machine_dry_run_never_downloads_or_installs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args = Namespace(project=Path.cwd(), conda=None, bridge_channel=root, dry_run=True)
            with (
                patch.object(setup, 'platform_issues', return_value=[]),
                patch.object(setup, 'find_conda', return_value=None),
                patch.object(setup, 'validate_channel', return_value=root),
                patch.object(setup.subprocess, 'run') as run,
                patch.object(setup, 'bootstrap_conda') as bootstrap,
                redirect_stdout(io.StringIO()) as output,
            ):
                assert setup.install(args, root / 'env', root / 'runtime') == 0
            run.assert_not_called()
            bootstrap.assert_not_called()
            assert 'Miniforge' in output.getvalue()
            assert setup.BRIDGE_SPEC in output.getvalue()

    def test_install_bootstraps_then_installs_and_validates_before_pip(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args = Namespace(project=Path.cwd(), conda=None, bridge_channel=root, dry_run=False)
            with (
                patch.object(setup, 'platform_issues', return_value=[]),
                patch.object(setup, 'find_conda', return_value=None),
                patch.object(setup, 'validate_channel', return_value=root),
                patch.object(setup, 'inspect_runtime', side_effect=[['Missing'], []]) as inspect,
                patch.object(setup, 'bootstrap_conda', return_value=str(root / 'miniforge/bin/conda')) as bootstrap,
                patch.object(setup.subprocess, 'run') as run,
                redirect_stdout(io.StringIO()),
            ):
                assert setup.install(args, root / 'env', root / 'runtime') == 0
            bootstrap.assert_called_once()
            assert inspect.call_count == 2
            commands = [call.args[0] for call in run.call_args_list]
            assert commands[0][1] == 'create'
            assert setup.BRIDGE_SPEC in commands[0]
            assert '-e' in commands[1]
            assert (root / 'runtime/activate.sh').is_file()

    def test_healthy_environment_skips_conda_transaction_and_channel(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args = Namespace(project=Path.cwd(), conda=None, bridge_channel=None, dry_run=False)
            with (
                patch.object(setup, 'platform_issues', return_value=[]),
                patch.object(setup, 'find_conda', return_value=str(root / 'conda/bin/conda')),
                patch.object(setup, 'inspect_runtime', return_value=[]),
                patch.object(setup, 'validate_channel') as validate,
                patch.object(setup.subprocess, 'run') as run,
                redirect_stdout(io.StringIO()),
            ):
                setup.install(args, root / 'env', root / 'runtime')
            validate.assert_not_called()
            run.assert_called_once()
            assert 'pip' in run.call_args.args[0]

    def test_platform_failure_stops_before_download(self):
        args = Namespace(project=Path.cwd(), conda=None, bridge_channel=None, dry_run=False)
        with (
            patch.object(setup, 'platform_issues', return_value=['requires Linux x86_64']),
            patch.object(setup, 'bootstrap_conda') as bootstrap,
            pytest.raises(ValueError, match='x86_64'),
        ):
            setup.install(args, Path('/unused'), Path('/unused'))
        bootstrap.assert_not_called()


if __name__ == '__main__':
    unittest.main()
