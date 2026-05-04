from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("first_cpp_pkg"),
        "config",
        "talker_params.yaml"
    )

    talker_param_node = Node(
        package="first_cpp_pkg",
        executable="talker_param",
        name="robot_status_talker_param",
        output="screen",
        parameters=[params_file],
    )

    listener_node = Node(
        package="first_cpp_pkg",
        executable="listener",
        name="robot_status_listener",
        output="screen",
    )

    return LaunchDescription([
        talker_param_node,
        listener_node,
    ])