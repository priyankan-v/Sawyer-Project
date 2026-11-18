"""
Run localhost TCPROS/RPC smoke checks without ROS or a physical robot.

Run from the repository root with pytest installed:
    python -m tests.test_irobot.smoke_sawyer_runtime
"""

from __future__ import annotations

import os
from pathlib import Path


def main():
    os.environ.setdefault('PYTEST_DISABLE_PLUGIN_AUTOLOAD', '1')
    import pytest

    source = Path(__file__).with_name('test_sawyer_transport.py')
    checks = (
        'test_direct_controller_connects_and_previews_neutral_without_ros',
        'test_direct_service_response_and_fragmented_reads',
        'test_publisher_negotiates_and_never_replays_commands',
        'test_master_restart_restores_registration_without_command_replay',
    )
    return pytest.main(['-q', *(f'{source}::{name}' for name in checks)])


if __name__ == '__main__':
    raise SystemExit(main())
