from route_manager.route_map import load_route


def test_load_route_returns_validated_waypoints(tmp_path):
    route_file = tmp_path / 'route.yaml'
    route_file.write_text(
        'waypoints:\n'
        '  - id: 0\n'
        '    x: 1.0\n'
        '    y: 2.0\n',
        encoding='utf-8',
    )

    waypoints = load_route(route_file)

    assert waypoints == [{'id': 0, 'x': 1.0, 'y': 2.0}]
