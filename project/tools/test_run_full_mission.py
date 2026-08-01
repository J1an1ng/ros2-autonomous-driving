"""Unit tests for atomic full-mission result publication helpers."""

from argparse import Namespace

from run_full_mission import build_launch_command
from run_full_mission import replace_results_directory
from run_full_mission import write_log_manifest


def test_launch_command_enables_motion_and_uses_requested_visualization():
    """The standard runner always launches the production controller."""
    args = Namespace(
        route_initial_waypoint_index=0,
        visualization=True,
        extra_launch_arg=['enable_octomap:=false'],
    )
    command = build_launch_command(args)
    assert 'enable_motion:=true' in command
    assert 'enable_visualization:=true' in command
    assert 'enable_octomap:=false' in command


def test_result_publication_replaces_stale_files(tmp_path):
    """A new result set removes every artifact from the previous run."""
    output = tmp_path / 'mission_results'
    staging = tmp_path / 'staging'
    output.mkdir()
    staging.mkdir()
    (output / 'stale.svg').write_text('old')
    (staging / 'metrics.json').write_text('{}')
    replace_results_directory(staging, output)
    assert not (output / 'stale.svg').exists()
    assert (output / 'metrics.json').read_text() == '{}'


def test_log_manifest_is_sorted_and_reproducible(tmp_path):
    """Published log evidence has deterministic relative paths and hashes."""
    logs = tmp_path / 'logs'
    logs.mkdir()
    (logs / 'b.log').write_text('second')
    (logs / 'a.log').write_text('first')
    output = tmp_path / 'ros_logs.sha256'
    write_log_manifest(logs, output)
    paths = [
        line.split('  ', 1)[1]
        for line in output.read_text().splitlines()
    ]
    assert paths == ['a.log', 'b.log']
