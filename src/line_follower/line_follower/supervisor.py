import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def is_nonzero(t: Twist, eps: float = 1e-4) -> bool:
    return (abs(t.linear.x) > eps or abs(t.linear.y) > eps or abs(t.linear.z) > eps or
            abs(t.angular.x) > eps or abs(t.angular.y) > eps or abs(t.angular.z) > eps)


class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')

        self.declare_parameter('line_topic', '/cmd_vel_line')
        self.declare_parameter('avoid_topic', '/cmd_vel_obstacle')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('rate_hz', 20.0)

        self.line_topic = self.get_parameter('line_topic').value
        self.avoid_topic = self.get_parameter('avoid_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.last_line = Twist()
        self.last_avoid = Twist()

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.line_topic, self.on_line, 10)
        self.create_subscription(Twist, self.avoid_topic, self.on_avoid, 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(
            f"Supervisor: {self.line_topic} + {self.avoid_topic} -> "
            f"{self.output_topic} at {self.rate_hz} Hz (priority: avoid > line)"
        )

    def on_line(self, msg: Twist):
        self.last_line = msg

    def on_avoid(self, msg: Twist):
        self.last_avoid = msg

    def on_timer(self):
        out = self.last_avoid if is_nonzero(self.last_avoid) else self.last_line
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Supervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
