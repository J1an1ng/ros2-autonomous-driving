from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wheelbase = LaunchConfiguration('wheelbase')
    maximum_steering_angle = LaunchConfiguration(
        'maximum_steering_angle'
    )
    config_file = PathJoinSubstitution(
        [FindPackageShare('controller'), 'config', 'controller.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('wheelbase', default_value='2.63'),
            DeclareLaunchArgument(
                'maximum_steering_angle', default_value='0.52'
            ),
            Node(
                package='controller',
                executable='controller_node',
                name='controller_node',
                output='screen',
                parameters=[
                    config_file,
                    {
                        'wheelbase': ParameterValue(
                            wheelbase, value_type=float
                        ),
                        'maximum_steering_angle': ParameterValue(
                            maximum_steering_angle, value_type=float
                        ),
                    },
                ],
            )
        ]
    )
