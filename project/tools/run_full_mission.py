#!/usr/bin/env python3
"""Run, record, audit, and publish one full autonomous-driving mission."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, String


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MISSION_TOPICS = (
    '/OurCar/INS/pose',
    '/OurCar/INS/twist',
    '/car_command',
    '/decision/front_obstacle_distance',
    '/decision/merge_obstacle_distance',
    '/decision/state',
    '/decision/stop_required',
    '/decision/target_speed',
    '/perception/traffic_light_state',
    '/planning/path',
    '/planning/target_speed',
    '/planning/trajectory',
    '/route/current_waypoint_distance',
    '/route/current_waypoint_id',
    '/route/route_state',
)


class MissionMonitor(Node):
    """Watch route progress until the current mission completes."""

    def __init__(self):
        """Create subscriptions used to detect route completion."""
        super().__init__('mission_results_monitor')
        self.complete = False
        self.current_waypoint = None
        self.start_time = time.monotonic()
        self.create_subscription(
            Int32,
            '/route/current_waypoint_id',
            self._on_waypoint,
            10,
        )
        self.create_subscription(
            String,
            '/route/route_state',
            self._on_route_state,
            10,
        )

    def _on_waypoint(self, message):
        waypoint = int(message.data)
        if waypoint == self.current_waypoint:
            return
        self.current_waypoint = waypoint
        elapsed = time.monotonic() - self.start_time
        print(
            f'[mission] waypoint={waypoint} elapsed={elapsed:.1f}s',
            flush=True,
        )

    def _on_route_state(self, message):
        if message.data == 'route_complete':
            self.complete = True
            print('[mission] route_complete', flush=True)


def command_text(command):
    """Return a shell-readable representation of an argument list."""
    return shlex.join(str(part) for part in command)


def build_launch_command(args):
    """Build the canonical production full-system launch command."""
    command = [
        'ros2',
        'launch',
        'system_bringup',
        'full_system.launch.py',
        f'route_initial_waypoint_index:={args.route_initial_waypoint_index}',
        'enable_motion:=true',
        f"enable_visualization:={'true' if args.visualization else 'false'}",
    ]
    command.extend(args.extra_launch_arg)
    return command


def build_record_command(bag_path):
    """Build the rosbag command containing every analyzer input topic."""
    return [
        'ros2',
        'bag',
        'record',
        '-s',
        'mcap',
        '--storage-preset-profile',
        'zstd_fast',
        '-o',
        str(bag_path),
        '--topics',
        *MISSION_TOPICS,
    ]


def stop_process_group(process, name, interrupt_timeout=15.0):
    """Stop one subprocess group, escalating only when graceful exit stalls."""
    if process is None or process.poll() is not None:
        return
    for current_signal, wait_seconds in (
        (signal.SIGINT, interrupt_timeout),
        (signal.SIGTERM, 5.0),
        (signal.SIGKILL, 2.0),
    ):
        try:
            os.killpg(process.pid, current_signal)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=wait_seconds)
            return
        except subprocess.TimeoutExpired:
            continue
    raise RuntimeError(f'Could not stop {name} process group')


def git_value(*arguments):
    """Read Git metadata without making publication depend on Git."""
    result = subprocess.run(
        ['git', *arguments],
        cwd=PROJECT_ROOT.parent,
        capture_output=True,
        text=True,
        check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def is_git_ignored(path):
    """Return whether Git ignores an evidence path."""
    result = subprocess.run(
        ['git', 'check-ignore', '--quiet', str(path.resolve())],
        cwd=PROJECT_ROOT.parent,
        capture_output=True,
        check=False,
    )
    return result.returncode == 0


def unity_renderer():
    """Read Unity's selected renderer from the most recent player log."""
    player_log = (
        Path.home()
        / '.config/unity3d/DefaultCompany/Autonomous Driving Sim/Player.log'
    )
    if not player_log.exists():
        return None
    match = re.search(
        r'^Renderer:\s*(.+)$',
        player_log.read_text(errors='replace'),
        re.MULTILINE,
    )
    return match.group(1).strip() if match else None


def write_log_manifest(logs_path, destination):
    """Write deterministic SHA-256 evidence for every retained ROS log."""
    lines = []
    for path in sorted(logs_path.rglob('*.log')):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        lines.append(f'{digest}  {path.relative_to(logs_path)}')
    destination.write_text('\n'.join(lines) + ('\n' if lines else ''))


def replace_results_directory(staging, output):
    """Atomically publish results and discard the previous set."""
    output.parent.mkdir(parents=True, exist_ok=True)
    backup = output.with_name(f'.{output.name}.previous')
    if backup.exists():
        shutil.rmtree(backup)
    if output.exists():
        output.rename(backup)
    try:
        staging.rename(output)
    except BaseException:
        if backup.exists() and not output.exists():
            backup.rename(output)
        raise
    if backup.exists():
        shutil.rmtree(backup)


def analyze_and_publish(
    bag_path,
    logs_path,
    output,
    launch_command,
    started_at,
    finished_at,
):
    """Analyze one successful bag and atomically replace mission_results."""
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(
        prefix='.mission-results-',
        dir=output.parent,
    ))
    analyzer = PROJECT_ROOT / 'tools' / 'analyze_mission_run.py'
    command = [
        sys.executable,
        str(analyzer),
        '--bag',
        str(bag_path),
        '--logs',
        str(logs_path),
        '--trajectory-config',
        'src/trajectory_generator/config/trajectory_generator.yaml',
        '--controller-config',
        'src/controller/config/controller.yaml',
        '--output',
        str(staging),
        '--validation-date',
        started_at.date().isoformat(),
    ]
    try:
        subprocess.run(command, cwd=PROJECT_ROOT, check=True)
        metrics = json.loads((staging / 'metrics.json').read_text())
        provenance = {
            'validation_date': started_at.date().isoformat(),
            'started_at': started_at.isoformat(),
            'finished_at': finished_at.isoformat(),
            'launch_command': launch_command,
            'source_branch': git_value('branch', '--show-current'),
            'source_commit': git_value('rev-parse', 'HEAD'),
            'working_tree_dirty': bool(git_value('status', '--porcelain')),
            'unity_renderer': unity_renderer(),
            'raw_evidence': {
                'bag_path': str(bag_path.resolve()),
                'logs_path': str(logs_path.resolve()),
                'git_ignored': (
                    is_git_ignored(bag_path)
                    and is_git_ignored(logs_path)
                ),
            },
            'mission_time_s': metrics.get('mission_time_s'),
            'route_complete': metrics.get('route_complete'),
        }
        (staging / 'validation_provenance.json').write_text(
            json.dumps(provenance, indent=2) + '\n'
        )
        write_log_manifest(logs_path, staging / 'ros_logs.sha256')
        replace_results_directory(staging, output)
    except BaseException:
        if staging.exists():
            shutil.rmtree(staging)
        raise
    print(f'[mission] results={output}', flush=True)
    print(
        f"[mission] mission_time_s={metrics.get('mission_time_s')}",
        flush=True,
    )
    return metrics


def run_live_mission(args):
    """Launch and record one mission, returning its retained evidence paths."""
    timestamp = datetime.now().astimezone().strftime('%Y%m%d-%H%M%S')
    run_root = args.evidence_root / f'mission-{timestamp}'
    logs_path = run_root / 'ros_logs'
    bag_path = run_root / 'bag'
    logs_path.mkdir(parents=True)
    environment = os.environ.copy()
    environment['ROS_LOG_DIR'] = str(logs_path)
    launch_command = build_launch_command(args)
    record_command = build_record_command(bag_path)
    print(f'[mission] evidence={run_root}', flush=True)
    print(f'[mission] launch={command_text(launch_command)}', flush=True)

    launch_process = None
    record_process = None
    started_at = datetime.now().astimezone()
    launch_log_path = logs_path / 'full_system_launch.log'
    record_log_path = logs_path / 'rosbag_record.log'
    try:
        with (
            launch_log_path.open('w') as launch_log,
            record_log_path.open('w') as record_log,
        ):
            record_process = subprocess.Popen(
                record_command,
                cwd=PROJECT_ROOT,
                env=environment,
                stdout=record_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            time.sleep(1.0)
            launch_process = subprocess.Popen(
                launch_command,
                cwd=PROJECT_ROOT,
                env=environment,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            rclpy.init(args=None)
            monitor = MissionMonitor()
            deadline = time.monotonic() + args.timeout
            try:
                while not monitor.complete and time.monotonic() < deadline:
                    if launch_process.poll() is not None:
                        raise RuntimeError(
                            'Full-system launch exited before route_complete; '
                            f'see {launch_log_path}'
                        )
                    rclpy.spin_once(monitor, timeout_sec=0.2)
                if not monitor.complete:
                    raise TimeoutError(
                        f'Mission did not complete within {args.timeout:.1f}s'
                    )
                time.sleep(args.post_complete_delay)
            finally:
                monitor.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
    finally:
        stop_process_group(launch_process, 'full-system launch')
        stop_process_group(record_process, 'rosbag recorder')

    metadata_path = bag_path / 'metadata.yaml'
    if not metadata_path.exists():
        subprocess.run(
            ['ros2', 'bag', 'reindex', '-s', 'mcap', str(bag_path)],
            cwd=PROJECT_ROOT,
            check=True,
        )
    finished_at = datetime.now().astimezone()
    return (
        bag_path,
        logs_path,
        command_text(launch_command),
        started_at,
        finished_at,
    )


def parse_arguments():
    """Parse live-run and retained-evidence publication options."""
    parser = argparse.ArgumentParser(
        description=(
            'Run the full project and regenerate mission_results, or publish '
            'one retained bag with --existing-bag.'
        )
    )
    parser.add_argument('--timeout', type=float, default=800.0)
    parser.add_argument('--post-complete-delay', type=float, default=1.0)
    parser.add_argument('--route-initial-waypoint-index', type=int, default=0)
    parser.add_argument('--visualization', action='store_true')
    parser.add_argument('--extra-launch-arg', action='append', default=[])
    parser.add_argument(
        '--evidence-root',
        type=Path,
        default=PROJECT_ROOT / 'validation_evidence',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=PROJECT_ROOT / 'mission_results',
    )
    parser.add_argument('--existing-bag', type=Path)
    parser.add_argument('--existing-logs', type=Path)
    return parser.parse_args()


def main():
    """Run or analyze one mission and publish its current result set."""
    args = parse_arguments()
    if bool(args.existing_bag) != bool(args.existing_logs):
        raise SystemExit(
            '--existing-bag and --existing-logs must be provided together'
        )
    if args.existing_bag:
        now = datetime.now().astimezone()
        inputs = (
            args.existing_bag,
            args.existing_logs,
            'analysis of retained mission evidence',
            now,
            now,
        )
    else:
        inputs = run_live_mission(args)
    bag_path, logs_path, launch_command, started_at, finished_at = inputs
    analyze_and_publish(
        bag_path,
        logs_path,
        args.output,
        launch_command,
        started_at,
        finished_at,
    )


if __name__ == '__main__':
    main()
