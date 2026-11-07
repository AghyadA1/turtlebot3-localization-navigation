"""Spawn the three ArUco marker boards into a running Gazebo (gz sim) world.

Requires GZ_SIM_RESOURCE_PATH to include this package's ``models`` directory so
that ``model://aruco_marker_N`` can be resolved.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

# (model name, x, y, z, qz, qw) in the Gazebo world frame.
# The same x, y (and the board yaw) are listed in config/marker_map_poses.yaml.
MARKERS = [
    ('aruco_marker_1', -1.00, -0.82, 0.2, 0.7071, 0.7071),
    ('aruco_marker_2', -1.00, -0.55, 0.2, 0.7071, 0.7071),
    ('aruco_marker_3', -1.05, -0.27, 0.2, 0.7071, 0.7071),
]


def spawn(name, x, y, z, qz, qw):
    request = (
        f'sdf_filename: "model://{name}", name: "{name}", '
        f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}}}'
    )
    return ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/default/create',
             '--reqtype', 'gz.msgs.EntityFactory',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000',
             '--req', request],
        output='screen',
    )


def generate_launch_description():
    # Stagger the requests by one second so the world is not flooded.
    return LaunchDescription([
        TimerAction(period=float(i), actions=[spawn(*m)])
        for i, m in enumerate(MARKERS)
    ])
