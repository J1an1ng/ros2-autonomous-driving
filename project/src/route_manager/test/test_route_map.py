import pytest
from route_manager.route_map import load_route
import yaml


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


@pytest.mark.parametrize(
    ('contents', 'expected_message'),
    [
        ('[]\n', 'valid YAML dictionary'),
        ('waypoints: []\n', 'No valid waypoints'),
        ('waypoints:\n  - invalid\n', 'Waypoint 0 is not a valid dictionary'),
        ('waypoints:\n  - x: 1.0\n', 'Waypoint 0 is missing fields'),
    ],
)
def test_load_route_rejects_invalid_route_structure(
    tmp_path,
    contents,
    expected_message,
):
    route_file = tmp_path / 'route.yaml'
    route_file.write_text(contents, encoding='utf-8')

    with pytest.raises(RuntimeError, match=expected_message):
        load_route(route_file)


def test_load_route_rejects_empty_path():
    with pytest.raises(RuntimeError, match='route_file is empty'):
        load_route('')


def test_load_route_rejects_missing_file(tmp_path):
    with pytest.raises(RuntimeError, match='Route file does not exist'):
        load_route(tmp_path / 'missing.yaml')


def test_load_route_rejects_directory(tmp_path):
    with pytest.raises(RuntimeError, match='Route path is not a file'):
        load_route(tmp_path)


def test_load_route_preserves_yaml_parse_errors(tmp_path):
    route_file = tmp_path / 'invalid.yaml'
    route_file.write_text('waypoints: [\n', encoding='utf-8')

    with pytest.raises(yaml.YAMLError):
        load_route(route_file)
