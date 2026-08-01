from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def package_launch(package_name, launch_file):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare(package_name), 'launch', launch_file]
        )
    )


def generate_launch_description():
    start_unity = LaunchConfiguration('start_unity')
    sensor_bind_host = LaunchConfiguration('sensor_bind_host')
    enable_perception = LaunchConfiguration('enable_perception')
    enable_octomap = LaunchConfiguration('enable_octomap')
    enable_route = LaunchConfiguration('enable_route')
    enable_trajectory = LaunchConfiguration('enable_trajectory')
    enable_decision = LaunchConfiguration('enable_decision')
    enable_controller = LaunchConfiguration('enable_controller')
    enable_visualization = LaunchConfiguration('enable_visualization')
    enable_motion = LaunchConfiguration('enable_motion')
    simulation_config = LaunchConfiguration('simulation_config')
    yolo_model = LaunchConfiguration('yolo_model')
    route_file = LaunchConfiguration('route_file')
    route_initial_waypoint_index = LaunchConfiguration(
        'route_initial_waypoint_index'
    )
    wheelbase = LaunchConfiguration('wheelbase')
    maximum_steering_angle = LaunchConfiguration(
        'maximum_steering_angle'
    )

    default_simulation_config = PathJoinSubstitution(
        [
            FindPackageShare('simulation'),
            'unity_sim',
            'Build_Ubuntu',
            'AD_Sim_Data',
            'StreamingAssets',
            'simulation_config.json',
        ]
    )
    default_route_file = PathJoinSubstitution(
        [FindPackageShare('route_manager'), 'config', 'route_map.yaml']
    )
    controller_config = PathJoinSubstitution(
        [FindPackageShare('controller'), 'config', 'controller.yaml']
    )
    default_yolo_model = PathJoinSubstitution(
        [FindPackageShare('perception'), 'models', 'yolov8n.onnx']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('start_unity', default_value='true'),
            DeclareLaunchArgument('sensor_bind_host', default_value='127.0.0.1'),
            DeclareLaunchArgument('enable_perception', default_value='true'),
            DeclareLaunchArgument('enable_octomap', default_value='true'),
            DeclareLaunchArgument('enable_route', default_value='true'),
            DeclareLaunchArgument('enable_trajectory', default_value='true'),
            DeclareLaunchArgument('enable_decision', default_value='true'),
            DeclareLaunchArgument('enable_controller', default_value='true'),
            DeclareLaunchArgument('enable_visualization', default_value='true'),
            DeclareLaunchArgument('enable_motion', default_value='false'),
            DeclareLaunchArgument(
                'simulation_config', default_value=default_simulation_config
            ),
            DeclareLaunchArgument('yolo_model', default_value=default_yolo_model),
            DeclareLaunchArgument('route_file', default_value=default_route_file),
            DeclareLaunchArgument(
                'route_initial_waypoint_index', default_value='0'
            ),
            DeclareLaunchArgument('wheelbase', default_value='2.63'),
            DeclareLaunchArgument(
                'maximum_steering_angle', default_value='0.52'
            ),
            IncludeLaunchDescription(
                package_launch('simulation', 'simulation.launch.py'),
                launch_arguments={
                    'config_file': simulation_config,
                    'start_unity': start_unity,
                    'sensor_bind_host': sensor_bind_host,
                }.items(),
            ),
            IncludeLaunchDescription(
                package_launch('perception', 'perception.launch.py'),
                launch_arguments={
                    'enable_octomap': enable_octomap,
                    'yolo_model': yolo_model,
                }.items(),
                condition=IfCondition(enable_perception),
            ),
            Node(
                package='route_manager',
                executable='route_manager_node',
                name='route_manager_node',
                output='screen',
                parameters=[
                    {
                        'route_file': route_file,
                        'map_frame': 'world',
                        'vehicle_frame': 'OurCar/INS',
                        'waypoint_reach_distance': 3.0,
                        'lookahead_index': 0,
                        'local_path_size': 5,
                        'initial_waypoint_index': ParameterValue(
                            route_initial_waypoint_index, value_type=int
                        ),
                    }
                ],
                condition=IfCondition(enable_route),
            ),
            IncludeLaunchDescription(
                package_launch(
                    'trajectory_generator', 'trajectory_generator.launch.py'
                ),
                launch_arguments={
                    'wheelbase': wheelbase,
                    'maximum_steering_angle': maximum_steering_angle,
                }.items(),
                condition=IfCondition(enable_trajectory),
            ),
            IncludeLaunchDescription(
                package_launch('decision_making', 'decision_making.launch.py'),
                condition=IfCondition(enable_decision),
            ),
            Node(
                package='controller',
                executable='controller_node',
                name='controller_node',
                output='screen',
                parameters=[
                    controller_config,
                    {
                        'enable_motion': ParameterValue(
                            enable_motion, value_type=bool
                        ),
                        'wheelbase': ParameterValue(
                            wheelbase, value_type=float
                        ),
                        'maximum_steering_angle': ParameterValue(
                            maximum_steering_angle, value_type=float
                        ),
                    },
                ],
                condition=IfCondition(enable_controller),
            ),
            Node(
                package='system_bringup',
                executable='live_visualization',
                name='live_visualization',
                output='screen',
                condition=IfCondition(enable_visualization),
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='driving_overview',
                arguments=[
                    '-d',
                    PathJoinSubstitution(
                        [FindPackageShare('system_bringup'), 'rviz', 'driving_overview.rviz']
                    ),
                ],
                output='screen',
                condition=IfCondition(enable_visualization),
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='telemetry_charts',
                arguments=[
                    '-d',
                    PathJoinSubstitution(
                        [FindPackageShare('system_bringup'), 'rviz', 'telemetry_charts.rviz']
                    ),
                ],
                output='screen',
                condition=IfCondition(enable_visualization),
            ),
        ]
    )
