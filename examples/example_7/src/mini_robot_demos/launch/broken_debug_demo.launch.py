from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mini_robot_demos',
            executable='fake_distance_sensor',
            name='fake_distance_sensor',
            output='screen'
        ),
        Node(
            package='mini_robot_demos',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[{
                # INTENTIONAL BUG:
                # The sensor publishes /mini_robot/distance,
                # but the decision node listens to /mini_robot/range.
                'distance_topic': '/mini_robot/range',
                'stop_distance': 0.8,
            }]
        ),
        Node(
            package='mini_robot_demos',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
    ])
