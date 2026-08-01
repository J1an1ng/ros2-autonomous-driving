from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_resolution = LaunchConfiguration('map_resolution')
    map_resolution_float = ParameterValue(map_resolution, value_type=float)
    map_frame = LaunchConfiguration('map_frame')
    octomap_executable = LaunchConfiguration('octomap_executable')
    enable_octomap = LaunchConfiguration('enable_octomap')
    yolo_model = LaunchConfiguration('yolo_model')

    default_yolo_model = PathJoinSubstitution(
        [FindPackageShare('perception'), 'models', 'yolov8n.onnx']
    )
    yolo_config = PathJoinSubstitution(
        [FindPackageShare('perception'), 'config', 'yolo_rgbd.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_resolution', default_value='0.25'),
        DeclareLaunchArgument('map_frame', default_value='world'),
        DeclareLaunchArgument(
            'octomap_executable', default_value='octomap_server_node'
        ),
        DeclareLaunchArgument('enable_octomap', default_value='true'),
        DeclareLaunchArgument('yolo_model', default_value=default_yolo_model),

        Node(
            package='perception',
            executable='occupancy_map_node',
            name='occupancy_map_node',
            output='screen',
            parameters=[{
                'image_topic': '/OurCar/Sensors/DepthCamera/image_raw',
                'camera_info_topic':
                    '/OurCar/Sensors/DepthCamera/camera_info',
                'alternate_image_topic':
                    '/Unity_ROS_message_Rx/OurCar/Sensors/'
                    'DepthCamera/image_raw',
                'alternate_camera_info_topic':
                    '/Unity_ROS_message_Rx/OurCar/Sensors/'
                    'DepthCamera/camera_info',
                'target_frame': map_frame,
                'pointcloud_topic': '/perception/pcl/points',
                'preprocessed_pointcloud_topic':
                    '/perception/pcl/points_preprocessed',
                'projected_map_topic':
                    '/perception/depth/projected_map',
                'extended_map_topic': '/perception/extended_map',
                'pixel_step': 4,
                'min_range_m': 0.25,
                'max_range_m': 35.0,
                'min_obstacle_z': 0.15,
                'max_obstacle_z': 2.5,
                'map_resolution_m': map_resolution_float,
                'map_width_m': 80.0,
                'map_height_m': 80.0,
                'inflation_radius_m': 0.75,
            }],
        ),

        Node(
            package='octomap_server',
            executable=octomap_executable,
            name='octomap_server',
            output='screen',
            remappings=[
                ('cloud_in', '/perception/pcl/points_preprocessed'),
                ('octomap_binary', '/perception/octomap/octomap_binary'),
                ('octomap_full', '/perception/octomap/octomap_full'),
                ('projected_map', '/perception/octomap/projected_map'),
            ],
            parameters=[{
                'frame_id': map_frame,
                'resolution': map_resolution_float,
                'sensor_model.max_range': 35.0,
                'pointcloud_min_z': 0.15,
                'pointcloud_max_z': 2.5,
                'filter_ground': False,
                'latch': False,
            }],
            condition=IfCondition(enable_octomap),
        ),

        Node(
            package='perception',
            executable='yolo_rgbd_detector_node',
            name='yolo_rgbd_detector_node',
            output='screen',
            parameters=[
                yolo_config,
                {'model_path': yolo_model},
            ],
        ),
    ])
