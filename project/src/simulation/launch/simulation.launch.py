import os
from pathlib import Path
import shutil
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def static_tf(name, args):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[str(a) for a in args],
        output='screen',
    )


def vehicle_sensor_transforms():
    return [
        static_tf('tf_nominal_center', [0, 0, -0.7180116, 0, 0, 0, 1, 'OurCar/INS', 'OurCar/Center']),
        static_tf('tf_sensors', [0.8199998, 0, 1.269, 0, 0, 0, 1, 'OurCar/Center', 'OurCar/Sensors/SensorBase']),
        static_tf('tf_semantic_camera', [0, 0, 0, 0.5, -0.5, 0.5, -0.5, 'OurCar/Sensors/SensorBase', 'OurCar/Sensors/SemanticCamera']),
        static_tf('tf_depth_camera', [0, 0, 0, 0.5, -0.5, 0.5, -0.5, 'OurCar/Sensors/SensorBase', 'OurCar/Sensors/DepthCamera']),
        static_tf('tf_rgb_camera_left', [0, 0.2, 0, 0.5, -0.5, 0.5, -0.5, 'OurCar/Sensors/SensorBase', 'OurCar/Sensors/RGBCameraLeft']),
        static_tf('tf_rgb_camera_right', [0, -0.2, 0, 0.5, -0.5, 0.5, -0.5, 'OurCar/Sensors/SensorBase', 'OurCar/Sensors/RGBCameraRight']),
    ]


def stage_unity_runtime(source_build, config_file, runtime_parent=None):
    """Create a lightweight Unity runtime overlay with the selected config."""
    source_build = Path(source_build).resolve(strict=True)
    config_file = Path(config_file).resolve(strict=True)
    source_executable = source_build / "AD_Sim.x86_64"
    source_data = source_build / "AD_Sim_Data"
    source_streaming = source_data / "StreamingAssets"

    if runtime_parent is not None:
        runtime_parent = Path(runtime_parent)
        runtime_parent.mkdir(parents=True, exist_ok=True)
    runtime_dir = Path(
        tempfile.mkdtemp(
            prefix="ros2_unity_runtime_",
            dir=str(runtime_parent) if runtime_parent is not None else None,
        )
    )
    runtime_executable = runtime_dir / source_executable.name
    shutil.copy2(source_executable, runtime_executable)

    for name in ("UnityPlayer.so", "LinuxPlayer_s.debug", "UnityPlayer_s.debug"):
        source_entry = source_build / name
        if source_entry.exists():
            os.symlink(source_entry, runtime_dir / name)

    runtime_data = runtime_dir / source_data.name
    runtime_data.mkdir()
    for source_entry in source_data.iterdir():
        if source_entry.name == source_streaming.name:
            continue
        os.symlink(
            source_entry,
            runtime_data / source_entry.name,
            target_is_directory=source_entry.is_dir(),
        )

    runtime_streaming = runtime_data / source_streaming.name
    runtime_streaming.mkdir()
    for source_entry in source_streaming.iterdir():
        if source_entry.name == "simulation_config.json":
            continue
        os.symlink(
            source_entry,
            runtime_streaming / source_entry.name,
            target_is_directory=source_entry.is_dir(),
        )
    shutil.copy2(config_file, runtime_streaming / "simulation_config.json")
    return runtime_dir, runtime_executable


def remove_unity_runtime(runtime_dir):
    runtime_dir = Path(runtime_dir)
    if not runtime_dir.name.startswith("ros2_unity_runtime_"):
        raise ValueError(f"Refusing to remove unexpected runtime path: {runtime_dir}")
    if runtime_dir.exists():
        shutil.rmtree(runtime_dir)


def cleanup_unity_runtime(context, runtime_dir):
    del context
    remove_unity_runtime(runtime_dir)
    return []


def launch_staged_unity(context, executable, config_file):
    executable_path = Path(context.perform_substitution(executable)).resolve(strict=True)
    config_path = Path(context.perform_substitution(config_file)).resolve(strict=True)
    runtime_dir, runtime_executable = stage_unity_runtime(
        executable_path.parent, config_path
    )
    process = ExecuteProcess(
        cmd=[str(runtime_executable)],
        cwd=str(runtime_dir),
        output="screen",
    )
    cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=process,
            on_exit=[
                OpaqueFunction(
                    function=cleanup_unity_runtime,
                    kwargs={"runtime_dir": runtime_dir},
                )
            ],
        )
    )
    return [process, cleanup]


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    start_unity = LaunchConfiguration('start_unity')
    sensor_bind_host = LaunchConfiguration('sensor_bind_host')
    default_config = PathJoinSubstitution([
        FindPackageShare('simulation'),
        'unity_sim', 'Build_Ubuntu', 'AD_Sim_Data', 'StreamingAssets', 'simulation_config.json'
    ])
    unity_executable = PathJoinSubstitution([
        FindPackageShare('simulation'),
        'unity_sim', 'Build_Ubuntu', 'AD_Sim.x86_64'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('start_unity', default_value='true'),
        DeclareLaunchArgument('sensor_bind_host', default_value='127.0.0.1'),
        Node(
            package='simulation',
            executable='JSON_param_reader',
            name='JSON_param_reader',
            output='screen',
            parameters=[{'config_file': config_file}],
        ),
        Node(
            package='simulation',
            executable='unity_TCP_stream_receiver',
            name='Unity_ROS_message_Rx',
            output='screen',
            parameters=[{'config_file': config_file, 'bind_host': sensor_bind_host}],
        ),
        Node(
            package='simulation',
            executable='ROS_command_transmitter',
            name='ROS_Unity_command_Tx',
            output='screen',
            parameters=[{'config_file': config_file}],
        ),
        *vehicle_sensor_transforms(),
        OpaqueFunction(
            function=launch_staged_unity,
            args=[unity_executable, config_file],
            condition=IfCondition(start_unity),
        ),
    ])
