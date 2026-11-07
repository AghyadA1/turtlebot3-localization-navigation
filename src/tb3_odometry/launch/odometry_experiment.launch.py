"""Start the wheel-odometry estimator and the live comparison plot.

Both nodes run on simulation time. This matters for the estimator: wheel
velocities are in simulated rad/s, so integrating them over wall-clock dt
scales the estimated distance by 1 / real-time-factor whenever Gazebo runs
slower than real time.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    save_path = LaunchConfiguration('save_path')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('save_path', default_value='',
                              description='Where to save the final plot (PNG).'),
        Node(package='tb3_odometry', executable='wheel_odometry',
             output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='tb3_odometry', executable='odom_plotter',
             output='screen', parameters=[{'use_sim_time': use_sim_time,
                                           'save_path': save_path}]),
    ])
