from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('trajectory_generator'),
        'config',
        'trajectory_generator.yaml',
    ])

    return LaunchDescription([
        Node(
            package='trajectory_generator',
            executable='trajectory_generator_node',
            name='trajectory_generator_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
