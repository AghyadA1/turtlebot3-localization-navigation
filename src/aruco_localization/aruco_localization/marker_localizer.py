#!/usr/bin/env python3
"""Global localization from ArUco markers with known map poses.

For every detected marker i with a known pose T_map_marker_i::

    T_base_marker  = T_base_camera @ T_camera_marker          (TF + detection)
    T_map_base_i   = T_map_marker_i @ inv(T_base_marker)      (one estimate)

The planar estimates (x, y, yaw) of all visible markers are averaged (yaw by a
circular mean) and turned into the ``map -> odom`` correction::

    T_map_odom = T_map_base @ inv(T_odom_base)

which is broadcast on TF, exactly the role AMCL plays in a Nav2 stack.
"""

import math
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from ros2_aruco_interfaces.msg import ArucoMarkers


def quat_to_rot(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n == 0.0:
        return np.eye(3)
    qx /= n
    qy /= n
    qz /= n
    qw /= n

    return np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
    ])


def rot_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]

    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    return qx, qy, qz, qw


def rpy_to_rot(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    Rx = np.array([
        [1, 0, 0],
        [0, cr, -sr],
        [0, sr, cr]
    ])

    Ry = np.array([
        [cp, 0, sp],
        [0, 1, 0],
        [-sp, 0, cp]
    ])

    Rz = np.array([
        [cy, -sy, 0],
        [sy, cy, 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx


def pose_to_matrix(position, orientation):
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(
        orientation.x, orientation.y, orientation.z, orientation.w
    )
    T[0, 3] = position.x
    T[1, 3] = position.y
    T[2, 3] = position.z
    return T


def transform_to_matrix(tf_msg):
    T = np.eye(4)
    q = tf_msg.transform.rotation
    t = tf_msg.transform.translation
    T[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
    T[0, 3] = t.x
    T[1, 3] = t.y
    T[2, 3] = t.z
    return T


def xyyaw_marker_to_matrix(x, y, yaw, marker_z, marker_roll, marker_pitch):
    T = np.eye(4)
    T[:3, :3] = rpy_to_rot(marker_roll, marker_pitch, yaw)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = marker_z
    return T


def xyyaw_to_matrix(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    T = np.eye(4)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    T[0, 3] = x
    T[1, 3] = y
    return T


def matrix_to_xyyaw(T):
    x = T[0, 3]
    y = T[1, 3]
    yaw = math.atan2(T[1, 0], T[0, 0])
    return x, y, yaw


def matrix_to_transform(T, parent, child, stamp):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child

    msg.transform.translation.x = float(T[0, 3])
    msg.transform.translation.y = float(T[1, 3])
    msg.transform.translation.z = float(T[2, 3])

    qx, qy, qz, qw = rot_to_quat(T[:3, :3])
    msg.transform.rotation.x = float(qx)
    msg.transform.rotation.y = float(qy)
    msg.transform.rotation.z = float(qz)
    msg.transform.rotation.w = float(qw)
    return msg


class MarkerLocalizer(Node):
    def __init__(self):
        super().__init__('marker_localizer')

        self.declare_parameter('aruco_topic', '/aruco_markers')
        self.declare_parameter('marker_map_yaml', '')
        self.declare_parameter('map_frame', 'world')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_rgb_frame')

        # fixed marker mounting parameters
        self.declare_parameter('marker_z', 0.20)
        self.declare_parameter('marker_roll', 1.5708)
        self.declare_parameter('marker_pitch', 0.0)

        self.aruco_topic = self.get_parameter('aruco_topic').value
        self.marker_map_yaml = self.get_parameter('marker_map_yaml').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.marker_z = float(self.get_parameter('marker_z').value)
        self.marker_roll = float(self.get_parameter('marker_roll').value)
        self.marker_pitch = float(self.get_parameter('marker_pitch').value)

        self.marker_map_poses = self.load_marker_map(self.marker_map_yaml)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            ArucoMarkers,
            self.aruco_topic,
            self.aruco_callback,
            10
        )

        self.get_logger().info(f'Listening to {self.aruco_topic}')
        self.get_logger().info(
            f'Loaded marker poses for IDs: {list(self.marker_map_poses.keys())}')
        self.get_logger().info(f'Publishing {self.map_frame} -> {self.odom_frame}')

    def load_marker_map(self, yaml_path):
        if not yaml_path:
            raise RuntimeError('marker_map_yaml parameter is empty')

        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        raw = data.get('marker_map_poses', {})
        marker_map = {}

        for marker_id, pose in raw.items():
            mid = int(marker_id)
            x, y, yaw = pose
            marker_map[mid] = xyyaw_marker_to_matrix(
                float(x), float(y), float(yaw),
                self.marker_z, self.marker_roll, self.marker_pitch
            )

        return marker_map

    def aruco_callback(self, msg):
        if not msg.marker_ids:
            return

        try:
            T_base_camera = transform_to_matrix(
                self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.camera_frame,
                    Time()
                )
            )

            T_odom_base = transform_to_matrix(
                self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    self.base_frame,
                    Time()
                )
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        candidates = []

        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            mid = int(marker_id)
            if mid not in self.marker_map_poses:
                continue

            T_map_marker = self.marker_map_poses[mid]
            T_camera_marker = pose_to_matrix(pose.position, pose.orientation)

            T_base_marker = T_base_camera @ T_camera_marker
            T_map_base = T_map_marker @ np.linalg.inv(T_base_marker)

            x, y, yaw = matrix_to_xyyaw(T_map_base)
            candidates.append((x, y, yaw))

        if not candidates:
            return

        xs = [c[0] for c in candidates]
        ys = [c[1] for c in candidates]
        yaws = [c[2] for c in candidates]

        x = float(np.mean(xs))
        y = float(np.mean(ys))
        yaw = math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))

        T_map_base = xyyaw_to_matrix(x, y, yaw)
        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)

        tf_msg = matrix_to_transform(
            T_map_odom,
            self.map_frame,
            self.odom_frame,
            self.get_clock().now().to_msg()
        )
        self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = MarkerLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
