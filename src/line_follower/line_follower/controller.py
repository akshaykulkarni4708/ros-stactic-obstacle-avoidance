#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LineController(Node):
    def __init__(self):
        super().__init__('line_controller')

        self.declare_parameter('linear_x', 0.08)
        self.declare_parameter('min_linear_x', 0.02)
        self.declare_parameter('k_p', 0.008)
        self.declare_parameter('max_ang_z', 0.80)
        self.declare_parameter('steer_sign', -1.0)
        self.declare_parameter('lost_sentinel', -1000.0)
        self.declare_parameter('search_w', 0.18)
        self.declare_parameter('search_linear_x', 0.03)
        self.declare_parameter('slowdown_error', 80.0)
        self.declare_parameter('turn_in_place_error', 240.0)
        self.declare_parameter('error_deadband', 10.0)
        self.declare_parameter('angular_alpha', 0.35)
        self.declare_parameter('lost_timeout_sec', 6.0)

        self.linear_x = float(self.get_parameter('linear_x').value)
        self.min_linear_x = float(self.get_parameter('min_linear_x').value)
        self.k_p = float(self.get_parameter('k_p').value)
        self.max_ang_z = float(self.get_parameter('max_ang_z').value)
        self.steer_sign = float(self.get_parameter('steer_sign').value)
        self.lost_sentinel = float(self.get_parameter('lost_sentinel').value)
        self.search_w = float(self.get_parameter('search_w').value)
        self.search_linear_x = float(self.get_parameter('search_linear_x').value)
        self.slowdown_error = float(self.get_parameter('slowdown_error').value)
        self.turn_in_place_error = float(self.get_parameter('turn_in_place_error').value)
        self.error_deadband = max(0.0, float(self.get_parameter('error_deadband').value))
        self.angular_alpha = self.clamp(
            float(self.get_parameter('angular_alpha').value), 0.0, 1.0
        )
        self.lost_timeout_sec = max(
            0.0, float(self.get_parameter('lost_timeout_sec').value)
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_line', 10)
        self.err_sub = self.create_subscription(Float32, '/line_error', self.on_error, 10)
        self.last_turn_sign = 1.0
        self.last_angular_z = 0.0
        self.lost_since = None
        self.stopped_after_timeout = False

        self.get_logger().info(
            f'Controller started: cmd_vel=/cmd_vel_line, linear_x={self.linear_x}, '
            f'min_linear_x={self.min_linear_x}, k_p={self.k_p}, max_ang_z={self.max_ang_z}, '
            f'error_deadband={self.error_deadband}, angular_alpha={self.angular_alpha}'
        )

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def on_error(self, msg: Float32) -> None:
        err = float(msg.data)
        cmd = Twist()

        if not math.isfinite(err) or err == self.lost_sentinel or err == -1.0:
            now = self.get_clock().now()
            if self.lost_since is None:
                self.lost_since = now
            lost_elapsed = (now - self.lost_since).nanoseconds / 1e9

            # A one-directional in-place search can, if it runs long enough,
            # rotate the robot a full half-turn before it happens to face the
            # line again -- which then sends it back the way it came instead
            # of continuing forward. On a short, finite line (as opposed to a
            # closed loop) that reversal repeats forever: dodge an obstacle,
            # reach the end of the line, spin around, drive straight back
            # into the same obstacle. Capping how long the search is allowed
            # to keep turning and stopping once genuinely lost (rather than
            # momentarily occluded) prevents that reversal instead of relying
            # on luck to avoid it.
            if lost_elapsed >= self.lost_timeout_sec:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                if not self.stopped_after_timeout:
                    self.stopped_after_timeout = True
                    self.get_logger().warn(
                        f'Line lost for {lost_elapsed:.1f}s (>= lost_timeout_sec='
                        f'{self.lost_timeout_sec:.1f}s) -- stopping instead of continuing '
                        'to search, to avoid spinning all the way around and heading back '
                        'the way we came.'
                    )
            else:
                cmd.linear.x = self.search_linear_x
                cmd.angular.z = self.last_turn_sign * self.search_w
            self.last_angular_z = cmd.angular.z
            self.cmd_pub.publish(cmd)
            return

        self.lost_since = None
        self.stopped_after_timeout = False

        if abs(err) <= self.error_deadband:
            target_angular_z = 0.0
        else:
            target_angular_z = self.clamp(
                self.steer_sign * self.k_p * err,
                -self.max_ang_z,
                self.max_ang_z,
            )

        angular_z = (
            self.last_angular_z
            + self.angular_alpha * (target_angular_z - self.last_angular_z)
        )
        angular_z = self.clamp(angular_z, -self.max_ang_z, self.max_ang_z)
        self.last_angular_z = angular_z
        if abs(angular_z) > 1e-4:
            self.last_turn_sign = 1.0 if angular_z > 0.0 else -1.0

        abs_error = abs(err)
        if abs_error >= self.turn_in_place_error:
            linear_x = 0.0
        elif abs_error <= self.slowdown_error:
            linear_x = self.linear_x
        else:
            span = max(1.0, self.turn_in_place_error - self.slowdown_error)
            ratio = (abs_error - self.slowdown_error) / span
            linear_x = self.linear_x - ratio * (self.linear_x - self.min_linear_x)

        cmd.linear.x = max(0.0, linear_x)
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LineController()
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
