#!/usr/bin/env python3
"""Dead-reckoning wheel odometry for a differential-drive TurtleBot3.

Wheel angular velocities are read from ``/joint_states`` and integrated with
the standard unicycle forward-kinematics model::

    v     = (v_r + v_l) / 2
    omega = (v_r - v_l) / L
    x     += v * cos(theta) * dt
    y     += v * sin(theta) * dt
    theta += omega * dt

The estimate is published as ``nav_msgs/Odometry`` so that it can be compared
against the simulator's ``/odom`` topic (see ``odom_plotter``).
"""

import math

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Return the quaternion of a pure rotation ``yaw`` about the z axis."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WheelOdometry(Node):
    """Integrate wheel velocities into a planar pose estimate."""

    def __init__(self) -> None:
        super().__init__('wheel_odometry')

        # TurtleBot3 Burger nominal geometry.
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.160)
        self.declare_parameter('left_joint', 'wheel_left_joint')
        self.declare_parameter('right_joint', 'wheel_right_joint')
        self.declare_parameter('odom_topic', '/wheel_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 10.0)
        # Off by default: the simulator already broadcasts odom -> base_footprint,
        # and two publishers of the same transform corrupt the TF tree.
        self.declare_parameter('publish_tf', False)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.left_joint = self.get_parameter('left_joint').value
        self.right_joint = self.get_parameter('right_joint').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_left = 0.0
        self.v_right = 0.0
        self.prev_time = None

        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self.update)
        self.get_logger().info('Wheel odometry node started.')

    def joint_callback(self, msg: JointState) -> None:
        """Convert wheel angular velocities (rad/s) to linear velocities (m/s)."""
        try:
            left = msg.name.index(self.left_joint)
            right = msg.name.index(self.right_joint)
        except ValueError:
            return
        if len(msg.velocity) <= max(left, right):
            return
        self.v_left = msg.velocity[left] * self.wheel_radius
        self.v_right = msg.velocity[right] * self.wheel_radius

    def update(self) -> None:
        """Integrate the kinematic model and publish the estimate."""
        now = self.get_clock().now()
        if self.prev_time is None:
            self.prev_time = now
            return
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        v = (self.v_right + self.v_left) / 2.0
        w = (self.v_right - self.v_left) / self.wheel_separation

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = math.atan2(math.sin(self.theta + w * dt),
                                math.cos(self.theta + w * dt))

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
