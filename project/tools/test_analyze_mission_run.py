"""Regression tests for mission trajectory acceptance criteria."""

from analyze_mission_run import analyze_trajectory_availability
from analyze_mission_run import require_route_and_trajectory_success

import pytest


def test_missing_trajectory_evidence_fails_availability_audit():
    """A completed route cannot pass when no trajectory was published."""
    availability = analyze_trajectory_availability(
        [], route_complete_ns=100
    )
    assert not availability['trajectory_availability_results'][
        'nonempty_trajectory_evidence'
    ]


def test_excessive_empty_trajectory_interval_fails_availability_audit():
    """A long consecutive empty interval is an explicit audit failure."""
    records = [(0, 10), (1_000_000_000, 0), (12_000_000_000, 0)]
    availability = analyze_trajectory_availability(
        records, route_complete_ns=13_000_000_000
    )
    assert not availability['trajectory_availability_results'][
        'continuous_runtime_availability'
    ]


def test_single_empty_message_uses_recovery_time_as_outage_end():
    """Treat a lone empty message as unavailable until recovery."""
    availability = analyze_trajectory_availability(
        [(0, 10), (1_000_000_000, 0), (12_000_000_000, 10)],
        route_complete_ns=13_000_000_000,
    )
    assert availability['maximum_continuous_empty_trajectory_s'] == 11.0
    assert not availability['trajectory_availability_results'][
        'continuous_runtime_availability'
    ]


def test_trailing_empty_message_uses_route_completion_as_outage_end():
    """An empty tail remains unavailable until route completion."""
    availability = analyze_trajectory_availability(
        [(0, 10), (1_000_000_000, 0)],
        route_complete_ns=20_000_000_000,
    )
    assert availability['maximum_continuous_empty_trajectory_s'] == 19.0
    assert not availability['trajectory_availability_results'][
        'continuous_runtime_availability'
    ]


def test_topic_silence_after_nonempty_message_fails_timeout_gate():
    """Fail topic silence even without an explicit empty message."""
    availability = analyze_trajectory_availability(
        [(0, 10)],
        route_complete_ns=20_000_000_000,
        maximum_message_gap_s=1.05,
    )
    assert availability['trajectory_availability_results'][
        'nonempty_trajectory_evidence'
    ]
    assert not availability['trajectory_availability_results'][
        'runtime_trajectory_message_gap'
    ]


def test_late_first_trajectory_message_fails_mission_start_gap():
    """Trajectory publication must be timely after the mission starts."""
    availability = analyze_trajectory_availability(
        [(20_000_000_000, 10)],
        route_complete_ns=30_000_000_000,
        maximum_message_gap_s=1.05,
        runtime_start_ns=0,
    )
    assert availability['maximum_runtime_trajectory_message_gap_s'] == 20.0
    assert not availability['trajectory_availability_results'][
        'runtime_trajectory_message_gap'
    ]


def test_failed_route_and_trajectory_audit_exits_nonzero():
    """The command-line gate raises SystemExit for an invalid result."""
    with pytest.raises(SystemExit, match='Route or trajectory audit failed'):
        require_route_and_trajectory_success({
            'route_and_trajectory_audit_passed': False,
        })


def test_successful_route_and_trajectory_audit_returns_normally():
    """A valid result does not terminate the analyzer."""
    require_route_and_trajectory_success({
        'route_and_trajectory_audit_passed': True,
    })
