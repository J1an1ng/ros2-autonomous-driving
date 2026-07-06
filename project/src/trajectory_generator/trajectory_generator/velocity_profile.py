def select_target_speed(
    route_state,
    speed_limit,
    default_speed,
    max_speed,
    slow_speed,
    stop_speed,
):
    effective_speed_limit = default_speed if speed_limit is None else speed_limit
    normalized_state = (route_state or "").lower()

    if "stop" in normalized_state:
        return float(stop_speed)

    if "intersection" in normalized_state:
        return float(min(effective_speed_limit, slow_speed))

    return float(min(effective_speed_limit, max_speed))
