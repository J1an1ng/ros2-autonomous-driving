from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unity_rgb_bridge_ros2',
            executable='unity_rgb_bridge_node',
            name='unity_rgb_bridge',
            output='screen',
            parameters=[{
                'bind_host': '0.0.0.0',
                'port': 9998,
                'flip_vertical': True,
                'publish_camera_info': True,
                'topic_prefix': '',
            }]
        )
    ])
