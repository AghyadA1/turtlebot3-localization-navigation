#!/usr/bin/env python3
"""Artificial Potential Field (APF) local controller for a TurtleBot3.

The robot pose comes from AMCL (``/amcl_pose``) and the goal from RViz
(``/goal_pose``). At every control step an attractive force towards the goal
(proportional to the goal vector in the robot frame) is summed with repulsive
forces from laser returns within ``repulsive_distance`` (Khatib-style
``k_rep * (1/d - 1/d0) / d^2``). The heading of the resulting force sets the
angular velocity and its magnitude sets the forward speed. A progress monitor
detects stagnation (a likely local minimum) and triggers a timed escape turn.
"""

import math
from collections import deque
from typing import Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ApfController(Node):
    def __init__(self) -> None:
        super().__init__('apf_controller')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('path_topic', '/apf_path')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('control_rate', 10.0)

        self.declare_parameter('k_att', 2.0)
        self.declare_parameter('k_rep', 0.03)
        self.declare_parameter('repulsive_distance', 0.30)
        self.declare_parameter('robot_radius', 0.08)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('yaw_tolerance', 0.20)
        self.declare_parameter('max_linear_speed', 0.18)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('angular_gain', 1.0)
        self.declare_parameter('linear_gain', 1.0)
        self.declare_parameter('front_angle_window_deg', 90.0)
        self.declare_parameter('emergency_stop_distance', 0.05)
        self.declare_parameter('progress_timeout', 10.0)
        self.declare_parameter('progress_epsilon', 0.05)
        self.declare_parameter('escape_turn_speed', 0.4)
        self.declare_parameter('escape_turn_duration', 1.0)
        self.declare_parameter('min_scan_range', 0.03)
        self.declare_parameter('max_scan_range', 3.5)

        self.goal_topic = self.get_parameter('goal_topic').value
        self.initialpose_topic = self.get_parameter('initialpose_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.amcl_pose_topic = self.get_parameter('amcl_pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.control_rate = float(self.get_parameter('control_rate').value)

        self.k_att = float(self.get_parameter('k_att').value)
        self.k_rep = float(self.get_parameter('k_rep').value)
        self.repulsive_distance = float(self.get_parameter('repulsive_distance').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.front_angle_window = math.radians(
            float(self.get_parameter('front_angle_window_deg').value))
        self.emergency_stop_distance = float(self.get_parameter('emergency_stop_distance').value)
        self.progress_timeout = float(self.get_parameter('progress_timeout').value)
        self.progress_epsilon = float(self.get_parameter('progress_epsilon').value)
        self.escape_turn_speed = float(self.get_parameter('escape_turn_speed').value)
        self.escape_turn_duration = float(self.get_parameter('escape_turn_duration').value)
        self.min_scan_range = float(self.get_parameter('min_scan_range').value)
        self.max_scan_range = float(self.get_parameter('max_scan_range').value)

        self.goal_msg: Optional[PoseStamped] = None
        self.initial_pose_msg: Optional[PoseWithCovarianceStamped] = None
        self.current_pose_msg: Optional[PoseWithCovarianceStamped] = None
        self.scan_msg: Optional[LaserScan] = None

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.global_frame

        self.escape_until = None
        self.best_goal_distance = float('inf')
        self.last_progress_time = self.get_clock().now()
        self.recent_positions = deque(maxlen=100)

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, self.initialpose_topic, self.initialpose_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, self.amcl_pose_topic, self.amcl_pose_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info('APF controller started.')

    def goal_callback(self, msg: PoseStamped) -> None:
        self.goal_msg = msg
        self.best_goal_distance = float('inf')
        self.last_progress_time = self.get_clock().now()
        self.escape_until = None
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.global_frame
        self.get_logger().info(
            f"Received goal: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}"
        )

    def initialpose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.initial_pose_msg = msg
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.global_frame
        self.get_logger().info(
            f"Received initial pose: x={msg.pose.pose.position.x:.3f}, "
            f"y={msg.pose.pose.position.y:.3f}"
        )

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.current_pose_msg = msg
        self.append_path_pose(msg)

    def scan_callback(self, msg: LaserScan) -> None:
        self.scan_msg = msg

    def append_path_pose(self, pose_msg: PoseWithCovarianceStamped) -> None:
        ps = PoseStamped()
        ps.header = pose_msg.header
        ps.pose = pose_msg.pose.pose
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        if not self.path_msg.poses:
            self.path_msg.poses.append(ps)
            return

        last = self.path_msg.poses[-1].pose.position
        dx = ps.pose.position.x - last.x
        dy = ps.pose.position.y - last.y
        if math.hypot(dx, dy) > 0.03:
            self.path_msg.poses.append(ps)

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        if self.current_pose_msg is None:
            return None

        p = self.current_pose_msg.pose.pose.position
        q = self.current_pose_msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        return p.x, p.y, yaw

    def get_goal_pose(self) -> Optional[Tuple[float, float, float]]:
        if self.goal_msg is None:
            return None

        p = self.goal_msg.pose.position
        q = self.goal_msg.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        return p.x, p.y, yaw

    def stop_robot(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(msg)

    def compute_goal_vector_in_base(self, rx: float, ry: float, ryaw: float,
                                    gx: float, gy: float) -> Tuple[float, float, float]:
        dx = gx - rx
        dy = gy - ry
        goal_distance = math.hypot(dx, dy)

        local_x = math.cos(-ryaw) * dx - math.sin(-ryaw) * dy
        local_y = math.sin(-ryaw) * dx + math.cos(-ryaw) * dy

        return local_x, local_y, goal_distance

    def emergency_obstacle(self) -> bool:
        if self.scan_msg is None:
            return False

        count = 0
        angle = self.scan_msg.angle_min

        for r in self.scan_msg.ranges:
            if math.isfinite(r) and self.min_scan_range < r < self.emergency_stop_distance:
                if abs(normalize_angle(angle)) < math.radians(20.0):
                    count += 1
            angle += self.scan_msg.angle_increment

        return count >= 3

    def compute_repulsive_force(self) -> Tuple[float, float]:
        if self.scan_msg is None:
            return 0.0, 0.0

        fx = 0.0
        fy = 0.0

        angle = self.scan_msg.angle_min
        half_window = self.front_angle_window / 2.0

        for r in self.scan_msg.ranges:
            max_r = min(self.max_scan_range, self.repulsive_distance)
            in_range = self.min_scan_range < r < max_r
            if math.isfinite(r) and in_range:
                bearing = normalize_angle(angle)
                if abs(bearing) <= half_window:
                    ox = r * math.cos(bearing)
                    oy = r * math.sin(bearing)

                    obs_dist = math.hypot(ox, oy)
                    d = max(obs_dist - self.robot_radius, 0.08)

                    if d < self.repulsive_distance:
                        mag = self.k_rep * ((1.0 / d) - (1.0 / self.repulsive_distance)) / (d * d)

                        # reduce repulsion from side walls, keep stronger reaction in front
                        angle_weight = max(0.0, math.cos(bearing)) ** 2

                        # push away from obstacle
                        away_x = -ox / obs_dist
                        away_y = -oy / obs_dist

                        fx += angle_weight * mag * away_x
                        fy += 0.35 * angle_weight * mag * away_y

            angle += self.scan_msg.angle_increment

        return fx, fy

    def reached_goal(self, rx: float, ry: float, ryaw: float,
                     gx: float, gy: float, gyaw: float) -> bool:
        pos_err = math.hypot(gx - rx, gy - ry)
        yaw_err = abs(normalize_angle(gyaw - ryaw))
        return pos_err <= self.goal_tolerance and yaw_err <= self.yaw_tolerance

    def progress_check(self, goal_distance: float) -> bool:
        now = self.get_clock().now()

        if goal_distance + 1e-6 < self.best_goal_distance - self.progress_epsilon:
            self.best_goal_distance = goal_distance
            self.last_progress_time = now
            return True

        stuck = (now - self.last_progress_time) > Duration(seconds=self.progress_timeout)
        return not stuck

    def in_escape_mode(self) -> bool:
        if self.escape_until is None:
            return False
        return self.get_clock().now() < self.escape_until

    def start_escape_mode(self, reason: str) -> None:
        now = self.get_clock().now()
        self.escape_until = now + Duration(seconds=self.escape_turn_duration)
        # Restart the progress timer, otherwise the robot is still considered
        # stuck as soon as the escape turn ends and it never drives again.
        self.last_progress_time = now
        self.get_logger().warn(f'{reason} Starting escape turn.')

    def control_loop(self) -> None:
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

        robot_pose = self.get_robot_pose()
        goal_pose = self.get_goal_pose()

        if robot_pose is None or goal_pose is None or self.scan_msg is None:
            self.stop_robot()
            return

        rx, ry, ryaw = robot_pose
        gx, gy, gyaw = goal_pose

        if self.reached_goal(rx, ry, ryaw, gx, gy, gyaw):
            self.stop_robot()
            return

        # Position reached but heading not: rotate in place to the goal yaw.
        if math.hypot(gx - rx, gy - ry) <= self.goal_tolerance:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.angular.z = clamp(
                self.angular_gain * normalize_angle(gyaw - ryaw),
                -self.max_angular_speed,
                self.max_angular_speed
            )
            self.cmd_pub.publish(msg)
            return

        if self.in_escape_mode():
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.angular.z = self.escape_turn_speed
            self.cmd_pub.publish(msg)
            return

        goal_x_local, goal_y_local, goal_distance = self.compute_goal_vector_in_base(
            rx, ry, ryaw, gx, gy
        )

        attractive_fx = self.k_att * goal_x_local
        attractive_fy = self.k_att * goal_y_local

        repulsive_fx, repulsive_fy = self.compute_repulsive_force()

        total_fx = attractive_fx + repulsive_fx
        total_fy = attractive_fy + repulsive_fy

        total_norm = math.hypot(total_fx, total_fy)
        heading_error = math.atan2(total_fy, total_fx)

        if self.emergency_obstacle():
            # Do not just stop (that deadlocks in front of the obstacle):
            # rotating in place is safe for a circular robot.
            self.start_escape_mode('Obstacle too close.')
            return

        if total_norm < 1e-5:
            self.start_escape_mode('Zero net force (local minimum).')
            return

        if not self.progress_check(goal_distance):
            self.start_escape_mode('No progress towards the goal (local minimum suspected).')
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.angular.z = clamp(
            self.angular_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        forward_term = self.linear_gain * total_norm * max(0.0, math.cos(heading_error))
        msg.twist.linear.x = clamp(forward_term, 0.0, self.max_linear_speed)

        if abs(heading_error) > 1.0:
            msg.twist.linear.x *= 0.25
        elif abs(heading_error) > 0.6:
            msg.twist.linear.x *= 0.5

        self.cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ApfController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
