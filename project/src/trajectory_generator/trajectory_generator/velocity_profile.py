from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DynamicLimits:
    max_speed: float
    max_lateral_acceleration: float
    max_longitudinal_acceleration: float
    max_longitudinal_deceleration: float


@dataclass(frozen=True)
class TrajectorySample:
    path_point: object
    target_speed: float
    time_from_start: float


class InfeasibleTrajectoryError(ValueError):
    """Raised when the measured initial state cannot satisfy the horizon."""


def time_parameterize(
    path_points,
    initial_speed,
    speed_limit,
    limits,
    terminal_speed=None,
):
    speed_cap = min(float(speed_limit), limits.max_speed)
    speed_caps = []
    for point in path_points:
        curvature = abs(float(point.curvature))
        curvature_speed = math.inf
        if curvature > 0.0:
            curvature_speed = math.sqrt(
                limits.max_lateral_acceleration / curvature
            )
        speed_caps.append(min(speed_cap, curvature_speed))

    if not speed_caps:
        return []

    measured_initial_speed = max(float(initial_speed), 0.0)
    tolerance = 1e-9
    if measured_initial_speed > speed_caps[0] + tolerance:
        raise InfeasibleTrajectoryError(
            'insufficient braking distance: initial speed '
            f'{measured_initial_speed:.3f} m/s exceeds the first-point cap '
            f'{speed_caps[0]:.3f} m/s'
        )
    speeds = [measured_initial_speed]
    for index in range(1, len(path_points)):
        distance = max(
            0.0,
            float(path_points[index].s - path_points[index - 1].s),
        )
        reachable_speed = math.sqrt(
            speeds[-1] ** 2
            + 2.0 * limits.max_longitudinal_acceleration * distance
        )
        speeds.append(min(speed_caps[index], reachable_speed))

    if terminal_speed is not None:
        requested_terminal_speed = max(float(terminal_speed), 0.0)
        if len(speeds) == 1 and speeds[0] > requested_terminal_speed + tolerance:
            raise InfeasibleTrajectoryError(
                'insufficient braking distance: the terminal constraint is at '
                'the initial point'
            )
        speeds[-1] = min(speeds[-1], requested_terminal_speed)

    for index in range(len(path_points) - 2, -1, -1):
        distance = max(
            0.0,
            float(path_points[index + 1].s - path_points[index].s),
        )
        braking_speed = math.sqrt(
            speeds[index + 1] ** 2
            + 2.0 * limits.max_longitudinal_deceleration * distance
        )
        if index == 0 and speeds[index] > braking_speed + tolerance:
            raise InfeasibleTrajectoryError(
                'insufficient braking distance: initial speed '
                f'{speeds[index]:.3f} m/s exceeds the reachable '
                f'{braking_speed:.3f} m/s boundary'
            )
        speeds[index] = min(speeds[index], braking_speed)

    times = [0.0]
    for index in range(1, len(path_points)):
        distance = max(
            0.0,
            float(path_points[index].s - path_points[index - 1].s),
        )
        speed_sum = speeds[index - 1] + speeds[index]
        duration = 2.0 * distance / max(speed_sum, 0.2)
        times.append(times[-1] + duration)

    return [
        TrajectorySample(
            path_point=point,
            target_speed=speed,
            time_from_start=time_from_start,
        )
        for point, speed, time_from_start in zip(path_points, speeds, times)
    ]


def select_target_speed(
    route_state,
    speed_limit,
    default_speed,
    max_speed,
    stop_speed,
):
    effective_speed_limit = default_speed if speed_limit is None else speed_limit
    normalized_state = (route_state or '').lower()

    if normalized_state == 'route_complete':
        return float(stop_speed)

    return float(min(effective_speed_limit, max_speed))
