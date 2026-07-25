from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    map_resolution = LaunchConfiguration('map_resolution')
    map_resolution_float = ParameterValue(map_resolution, value_type=float)
    map_frame = LaunchConfiguration('map_frame')
    octomap_executable = LaunchConfiguration('octomap_executable')
    enable_octomap = LaunchConfiguration('enable_octomap')

    return LaunchDescription([
        DeclareLaunchArgument('map_resolution', default_value='0.25'),
        DeclareLaunchArgument('map_frame', default_value='world'),
        DeclareLaunchArgument('octomap_executable', default_value='octomap_server_node'),
        DeclareLaunchArgument('enable_octomap', default_value='true'),

        Node(
            package='perception',
            executable='occupancy_map_node',
            name='occupancy_map_node',
            output='screen',
            parameters=[{
                'image_topic': '/OurCar/Sensors/DepthCamera/image_raw',
                'camera_info_topic': '/OurCar/Sensors/DepthCamera/camera_info',
                'alternate_image_topic': '/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw',
                'alternate_camera_info_topic': '/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/camera_info',
                'target_frame': map_frame,
                'pointcloud_topic': '/perception/pcl/points',
                'preprocessed_pointcloud_topic': '/perception/pcl/points_preprocessed',
                'projected_map_topic': '/perception/depth/projected_map',
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
            executable='traffic_light_detector_node',
            name='traffic_light_detector_node',
            output='screen',
            parameters=[{
                'image_topic': '/OurCar/Sensors/RGBCameraLeft/image_raw',
                'semantic_topic': '/OurCar/Sensors/SemanticCamera/image_raw',
                'state_topic': '/perception/traffic_light_state',
                'debug_topic': '/perception/traffic_light_debug',
                'max_semantic_stamp_delta_s': 1.0,
                # Treat vertically separated high/low lamps for the same
                # approach as one group while excluding other road approaches.
                'target_light': 'signal_group',
                'signal_group_max_dx_ratio': 0.16,
                'signal_group_min_size_ratio': 0.45,
                # Strong green (little/no red evidence) may release after two
                # frames. Mixed red/green evidence still needs four frames.
                'consecutive_frames': 4,
                'confident_green_frames': 2,
                'confident_green_max_red_score': 4,
                'confident_green_min_score': 20,
            }],
        ),

        # This node is intentionally diagnostic-only for the first validation
        # stage. Decision making does not yet consume its distance output.
        Node(
            package='perception',
            executable='semantic_vehicle_detector_node',
            name='semantic_vehicle_detector_node',
            output='screen',
            parameters=[{
                'semantic_topic': '/OurCar/Sensors/SemanticCamera/image_raw',
                'depth_topic': '/OurCar/Sensors/DepthCamera/image_raw',
                'camera_info_topic':
                    '/OurCar/Sensors/DepthCamera/camera_info',
                'detected_topic': '/perception/front_vehicle_detected',
                'distance_topic': '/perception/front_vehicle_distance',
                'debug_topic': '/perception/front_vehicle_debug',
                'semantic_r': 255,
                'semantic_g': 0,
                'semantic_b': 0,
                'semantic_tolerance': 45,
                'roi_x_min': 0.12,
                'roi_y_min': 0.10,
                'roi_x_max': 0.88,
                'roi_y_max': 0.98,
                'min_component_pixels': 80,
                'min_valid_depth_pixels': 20,
                'distance_percentile': 0.20,
                'max_depth_stamp_delta_s': 1.0,
                # A baseline road-side red component at waypoint 48 measured
                # at least about 3 m off the optical centreline.
                'max_abs_lateral_m': 2.8,
            }],
        ),
    ])
