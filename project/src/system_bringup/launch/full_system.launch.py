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
            [FindPackageShare(package_name), "launch", launch_file]
        )
    )


def generate_launch_description():
    start_unity = LaunchConfiguration("start_unity")
    enable_perception = LaunchConfiguration("enable_perception")
    enable_octomap = LaunchConfiguration("enable_octomap")
    enable_route = LaunchConfiguration("enable_route")
    enable_trajectory = LaunchConfiguration("enable_trajectory")
    enable_decision = LaunchConfiguration("enable_decision")
    enable_controller = LaunchConfiguration("enable_controller")
    enable_motion = LaunchConfiguration("enable_motion")
    simulation_config = LaunchConfiguration("simulation_config")
    route_file = LaunchConfiguration("route_file")

    default_simulation_config = PathJoinSubstitution(
        [
            FindPackageShare("simulation"),
            "unity_sim",
            "Build_Ubuntu",
            "AD_Sim_Data",
            "StreamingAssets",
            "simulation_config.json",
        ]
    )
    default_route_file = PathJoinSubstitution(
        [FindPackageShare("route_manager"), "config", "route_map.yaml"]
    )
    controller_config = PathJoinSubstitution(
        [FindPackageShare("controller"), "config", "controller.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_unity", default_value="true"),
            DeclareLaunchArgument("enable_perception", default_value="true"),
            DeclareLaunchArgument("enable_octomap", default_value="true"),
            DeclareLaunchArgument("enable_route", default_value="true"),
            DeclareLaunchArgument("enable_trajectory", default_value="true"),
            DeclareLaunchArgument("enable_decision", default_value="true"),
            DeclareLaunchArgument("enable_controller", default_value="true"),
            DeclareLaunchArgument("enable_motion", default_value="false"),
            DeclareLaunchArgument(
                "simulation_config", default_value=default_simulation_config
            ),
            DeclareLaunchArgument("route_file", default_value=default_route_file),
            IncludeLaunchDescription(
                package_launch("simulation", "simulation.launch.py"),
                launch_arguments={
                    "config_file": simulation_config,
                    "start_unity": start_unity,
                }.items(),
            ),
            IncludeLaunchDescription(
                package_launch("perception", "perception.launch.py"),
                launch_arguments={"enable_octomap": enable_octomap}.items(),
                condition=IfCondition(enable_perception),
            ),
            Node(
                package="route_manager",
                executable="route_manager_node",
                name="route_manager_node",
                output="screen",
                parameters=[
                    {
                        "route_file": route_file,
                        "map_frame": "world",
                        "vehicle_frame": "OurCar/INS",
                        "waypoint_reach_distance": 3.0,
                        "lookahead_index": 0,
                        "local_path_size": 5,
                    }
                ],
                condition=IfCondition(enable_route),
            ),
            IncludeLaunchDescription(
                package_launch(
                    "trajectory_generator", "trajectory_generator.launch.py"
                ),
                condition=IfCondition(enable_trajectory),
            ),
            IncludeLaunchDescription(
                package_launch("decision_making", "decision_making.launch.py"),
                condition=IfCondition(enable_decision),
            ),
            Node(
                package="controller",
                executable="controller_node",
                name="controller_node",
                output="screen",
                parameters=[
                    controller_config,
                    {
                        "enable_motion": ParameterValue(
                            enable_motion, value_type=bool
                        )
                    },
                ],
                condition=IfCondition(enable_controller),
            ),
        ]
    )
