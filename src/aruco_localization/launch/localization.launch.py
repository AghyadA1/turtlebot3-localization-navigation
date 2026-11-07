from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_localization')

    marker_yaml = os.path.join(pkg_share, 'config', 'marker_map_poses.yaml')
    map_yaml = os.path.join(pkg_share, 'config', 'my_map.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'localization.rviz')

    return LaunchDescription([
        Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_node',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'camera_info_topic': '/camera/camera_info',
                'marker_size': 0.16,
                'aruco_dictionary_id': 'DICT_5X5_250',
                'use_sim_time': True,
            }]
        ),

        Node(
            package='aruco_localization',
            executable='marker_localizer',
            name='marker_localizer',
            output='screen',
            parameters=[{
                'aruco_topic': '/aruco_markers',
                'marker_map_yaml': marker_yaml,
                'map_frame': 'world',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'camera_frame': 'camera_rgb_frame',
                'marker_z': 0.20,
                'marker_roll': 1.5708,
                'marker_pitch': 0.0,
                'use_sim_time': True,
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world_tf',
            arguments=[
                '--x', '-0.425',
                '--y', '2.02',
                '--z', '0.0',
                '--yaw', '1.57',
                '--pitch', '0.0',
                '--roll', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'world',
            ]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'frame_id': 'map',
                'use_sim_time': True,
            }]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server'],
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),
    ])
