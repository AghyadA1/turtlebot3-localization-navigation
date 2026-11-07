#!/usr/bin/env python3
"""Open-loop square trajectory for a TurtleBot3.

The robot repeats ``drive forward -> stop -> rotate in place -> stop`` four
times using timed velocity commands only. No feedback is used, which is the
point of the experiment: the executed path differs from the commanded square
because of wheel slip, acceleration limits and timing jitter.
"""

import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node


class SquareMovement(Node):
    """Publish timed velocity commands that nominally trace a square."""

    def __init__(self) -> None:
        super().__init__('square_movement')
        self.declare_parameter('linear_speed', 0.2)     # m/s
        self.declare_parameter('forward_duration', 3.0)  # s
        self.declare_parameter('angular_speed', 1.0)    # rad/s
        self.declare_parameter('turn_duration', 5.0)    # s, tuned empirically
        self.declare_parameter('sides', 4)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def publish_velocity(self, linear: float = 0.0, angular: float = 0.0) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.pub.publish(msg)

    def move(self, linear: float, angular: float, seconds: float) -> None:
        # Wall-clock timing on purpose: this node never spins, so a ROS clock
        # driven by /clock (use_sim_time) would never advance here.
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            self.publish_velocity(linear, angular)
            time.sleep(0.1)

    def stop(self) -> None:
        self.publish_velocity(0.0, 0.0)
        time.sleep(0.5)

    def run(self) -> None:
        p = self.get_parameter
        sides = int(p('sides').value)
        for i in range(sides):
            self.get_logger().info(f'Side {i + 1}/{sides}: forward')
            self.move(float(p('linear_speed').value), 0.0,
                      float(p('forward_duration').value))
            self.stop()
            self.get_logger().info(f'Side {i + 1}/{sides}: turn')
            self.move(0.0, float(p('angular_speed').value),
                      float(p('turn_duration').value))
            self.stop()
        self.get_logger().info('Square complete.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SquareMovement()
    time.sleep(1.0)  # let the publisher connect
    try:
        node.run()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
