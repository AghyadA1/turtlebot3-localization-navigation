#!/usr/bin/env python3
"""Live plot of ground-truth vs. estimated odometry with drift statistics.

Blue: reference trajectory (simulator ``/odom``).
Green: estimated trajectory (``/wheel_odom`` from ``wheel_odometry``).
Red dashed: instantaneous position error between the two.

When the window is closed, the final and mean position errors are printed and,
if ``save_path`` is set, the figure is written to disk.
"""

from collections import deque
import math

import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class OdomPlotter(Node):
    """Subscribe to two odometry topics and plot them side by side."""

    def __init__(self) -> None:
        super().__init__('odom_plotter')
        self.declare_parameter('reference_topic', '/odom')
        self.declare_parameter('estimate_topic', '/wheel_odom')
        self.declare_parameter('max_points', 1000)
        self.declare_parameter('save_path', '')

        n = int(self.get_parameter('max_points').value)
        self.ref = (deque(maxlen=n), deque(maxlen=n))
        self.est = (deque(maxlen=n), deque(maxlen=n))
        self.errors = []

        self.create_subscription(
            Odometry, self.get_parameter('reference_topic').value,
            lambda m: self.on_odom(m, self.ref), 10)
        self.create_subscription(
            Odometry, self.get_parameter('estimate_topic').value,
            lambda m: self.on_odom(m, self.est), 10)

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ref_line, = self.ax.plot([], [], 'b-', lw=2.5, label='Ground truth (/odom)')
        self.est_line, = self.ax.plot([], [], 'g-', lw=2.5,
                                      label='Wheel odometry (/wheel_odom)')
        self.err_line, = self.ax.plot([], [], 'r--', lw=1.5, alpha=0.6, label='Drift error')
        self.ref_dot, = self.ax.plot([], [], 'bo', ms=8)
        self.est_dot, = self.ax.plot([], [], 'go', ms=8)
        self.ax.set_xlabel('x [m]')
        self.ax.set_ylabel('y [m]')
        self.ax.set_title('Odometry comparison: ground truth vs. wheel odometry')
        self.ax.grid(True, alpha=0.3, linestyle='--')
        self.ax.legend(loc='upper left')
        self.ax.set_aspect('equal', adjustable='datalim')

    def on_odom(self, msg: Odometry, store) -> None:
        store[0].append(msg.pose.pose.position.x)
        store[1].append(msg.pose.pose.position.y)

    def redraw(self) -> None:
        self.ref_line.set_data(*self.ref)
        self.est_line.set_data(*self.est)
        if self.ref[0] and self.est[0]:
            rx, ry = self.ref[0][-1], self.ref[1][-1]
            ex, ey = self.est[0][-1], self.est[1][-1]
            self.ref_dot.set_data([rx], [ry])
            self.est_dot.set_data([ex], [ey])
            self.err_line.set_data([rx, ex], [ry, ey])
            self.errors.append(math.hypot(rx - ex, ry - ey))
        self.ax.relim()
        self.ax.autoscale_view()

    def run(self) -> None:
        while rclpy.ok() and plt.get_fignums():
            rclpy.spin_once(self, timeout_sec=0.01)
            self.redraw()
            plt.pause(0.05)
        self.report()

    def report(self) -> None:
        save_path = self.get_parameter('save_path').value
        if save_path:
            self.fig.savefig(save_path, dpi=150, bbox_inches='tight')
        if not self.errors:
            return
        mean = sum(self.errors) / len(self.errors)
        rmse = math.sqrt(sum(e * e for e in self.errors) / len(self.errors))
        self.get_logger().info(
            f'Final error: {self.errors[-1]:.4f} m | mean: {mean:.4f} m | '
            f'RMSE: {rmse:.4f} m | samples: {len(self.errors)}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomPlotter()
    try:
        node.run()
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
