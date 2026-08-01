from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('decision_making'), 'config', 'traffic_light_state_machine.yaml']
    )
    return LaunchDescription(
        [
            Node(
                package='decision_making',
                executable='obstacle_monitor_node',
                name='obstacle_monitor_node',
                output='screen',
                parameters=[config],
            ),
            Node(
                package='decision_making',
                executable='traffic_light_state_machine_node',
                name='traffic_light_state_machine_node',
                output='screen',
                parameters=[config],
            )
        ]
    )
