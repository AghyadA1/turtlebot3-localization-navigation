import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_apf = get_package_share_directory('apf_nav')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    rviz_config = LaunchConfiguration('rviz_config')

    default_map = os.path.join(pkg_apf, 'maps', 'my_map.yaml')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            os.path.join(pkg_apf, 'config', 'amcl_params.yaml'),
            {'yaml_filename': map_yaml, 'use_sim_time': use_sim_time},
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(pkg_apf, 'config', 'amcl_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            os.path.join(pkg_apf, 'config', 'amcl_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    apf_controller = Node(
        package='apf_nav',
        executable='apf_controller',
        name='apf_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'goal_topic': '/goal_pose',
            'initialpose_topic': '/initialpose',
            'scan_topic': '/scan',
            'amcl_pose_topic': '/amcl_pose',
            'cmd_vel_topic': '/cmd_vel',
            'path_topic': '/apf_path',
            'base_frame': 'base_footprint',
            'global_frame': 'map',
            'control_rate': 10.0,
            'k_att': 1.2,
            'k_rep': 0.25,
            'repulsive_distance': 0.75,
            'robot_radius': 0.12,
            'goal_tolerance': 0.12,
            'yaw_tolerance': 0.20,
            'max_linear_speed': 0.18,
            'max_angular_speed': 1.0,
            'angular_gain': 1.8,
            'linear_gain': 0.8,
            'front_angle_window_deg': 140.0,
            'emergency_stop_distance': 0.16,
            'progress_timeout': 6.0,
            'progress_epsilon': 0.08,
            'escape_turn_speed': 0.7,
            'escape_turn_duration': 2.5,
            'min_scan_range': 0.03,
            'max_scan_range': 3.5,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz'
            )
        ),
        sim_launch,
        map_server,
        amcl,
        lifecycle_manager,
        apf_controller,
        rviz,
    ])
