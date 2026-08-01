from pathlib import Path

import yaml


def load_route(route_file):
    """Load and validate route waypoints from a YAML file."""
    if not route_file:
        raise RuntimeError('Parameter route_file is empty.')

    path = Path(route_file)
    if not path.exists():
        raise RuntimeError(f'Route file does not exist: {route_file}')
    if not path.is_file():
        raise RuntimeError(f'Route path is not a file: {route_file}')

    with path.open('r', encoding='utf-8') as route_stream:
        data = yaml.safe_load(route_stream)

    if not isinstance(data, dict):
        raise RuntimeError(
            'Route file does not contain a valid YAML dictionary.'
        )

    waypoints = data.get('waypoints', [])
    if not isinstance(waypoints, list) or not waypoints:
        raise RuntimeError('No valid waypoints found in route file.')

    for index, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            raise RuntimeError(
                f'Waypoint {index} is not a valid dictionary.'
            )

        missing_fields = [
            field for field in ('x', 'y') if field not in waypoint
        ]
        if missing_fields:
            raise RuntimeError(
                f'Waypoint {index} is missing fields: {missing_fields}'
            )

    return waypoints
