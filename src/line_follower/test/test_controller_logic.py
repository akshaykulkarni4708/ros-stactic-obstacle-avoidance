import unittest

import rclpy
from std_msgs.msg import Float32

from line_follower.controller import LineController


class TestLineControllerLogic(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = LineController()
        self.published = []
        self.node.cmd_pub.publish = self.published.append

    def tearDown(self):
        self.node.destroy_node()

    def _send(self, value):
        self.node.on_error(Float32(data=value))
        return self.published[-1]

    def test_lost_sentinel_triggers_search_behavior(self):
        cmd = self._send(self.node.lost_sentinel)
        self.assertAlmostEqual(cmd.linear.x, self.node.search_linear_x)
        self.assertNotEqual(cmd.angular.z, 0.0)

    def test_small_error_within_deadband_goes_straight(self):
        cmd = self._send(5.0)
        self.assertEqual(cmd.angular.z, 0.0)
        self.assertGreater(cmd.linear.x, 0.0)

    def test_positive_error_steers_toward_negative_angular_z(self):
        # Positive error means the line's centroid is right of image center;
        # the robot must yaw right (negative angular.z, REP103) to recenter.
        cmd = None
        for _ in range(5):
            cmd = self._send(100.0)
        self.assertLess(cmd.angular.z, 0.0)

    def test_large_error_triggers_turn_in_place(self):
        cmd = None
        for _ in range(5):
            cmd = self._send(300.0)
        self.assertEqual(cmd.linear.x, 0.0)

    def test_search_stops_after_lost_timeout_instead_of_spinning_forever(self):
        # An unbounded one-directional search can rotate the robot a full
        # half-turn before it happens to face the line again, sending it
        # back the way it came. Past lost_timeout_sec it should stop instead
        # of continuing to spin.
        cmd = self._send(self.node.lost_sentinel)
        self.assertNotEqual(cmd.angular.z, 0.0)

        self.node.lost_since = self.node.get_clock().now() - rclpy.duration.Duration(
            seconds=self.node.lost_timeout_sec + 1.0
        )
        cmd = self._send(self.node.lost_sentinel)
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(cmd.angular.z, 0.0)

    def test_reacquiring_line_resets_lost_timer(self):
        self._send(self.node.lost_sentinel)
        self.node.lost_since = self.node.get_clock().now() - rclpy.duration.Duration(
            seconds=self.node.lost_timeout_sec + 1.0
        )
        self._send(5.0)
        self.assertIsNone(self.node.lost_since)


if __name__ == '__main__':
    unittest.main()
