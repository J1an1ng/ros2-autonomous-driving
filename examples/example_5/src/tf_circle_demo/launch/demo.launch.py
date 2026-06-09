from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf_circle_demo',
            executable='tf_circle_node',
            output='screen'
        )
    ])