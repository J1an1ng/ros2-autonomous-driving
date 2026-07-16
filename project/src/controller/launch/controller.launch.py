from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("controller"), "config", "controller.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="controller",
                executable="controller_node",
                name="controller_node",
                output="screen",
                parameters=[config_file],
            )
        ]
    )
