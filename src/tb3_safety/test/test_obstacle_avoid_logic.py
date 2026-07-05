import math
import time
import unittest

import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from tb3_safety.obstacle_avoid import ObstacleAvoid


def make_scan(distance, num_points=360):
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = (2 * math.pi) / num_points
    scan.ranges = [float(distance)] * num_points
    return scan


class TestObstacleAvoidLogic(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = ObstacleAvoid()
        # Shrink the state timers so the test doesn't need to wait seconds.
        self.node.stop_time = 0.05
        self.node.turn_time = 0.05
        self.node.turn_time_cap = 0.05
        self.node.forward_time = 0.05

    def tearDown(self):
        self.node.destroy_node()

    def test_idle_transitions_to_stop_on_close_obstacle(self):
        self.node.last_scan = make_scan(0.1)
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.STOP)

    def test_idle_stays_idle_when_clear(self):
        self.node.last_scan = make_scan(5.0)
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.IDLE)

    def test_scan_distances_subtract_lidar_to_hull_margin(self):
        # base_scan sits behind the robot's center while the hull extends
        # ahead of it, so a raw range must be reduced before it represents
        # true clearance to the hull -- comparing raw ranges directly
        # against emergency_distance let the hull reach the obstacle
        # before EMERGENCY ever triggered.
        self.node.last_scan = make_scan(1.0)
        front, left, right = self.node._scan_distances()
        expected = 1.0 - self.node.lidar_to_hull_margin
        self.assertAlmostEqual(front, expected, places=3)
        self.assertAlmostEqual(left, expected, places=3)
        self.assertAlmostEqual(right, expected, places=3)

    def test_side_sectors_are_contiguous_with_front_sector(self):
        # The side sectors are defined to start exactly at
        # front_half_angle, by construction (there is no separate
        # side_sector_min parameter to drift out of sync with it), so
        # there is no angular gap an obstacle could sit in undetected
        # during a turn -- every angle just past the front cone belongs
        # to a side sector.
        just_past_front = self.node.front_half_angle + 1e-6
        self.assertTrue(self.node._in_sector(
            just_past_front, self.node.front_half_angle, self.node.side_sector_max))
        self.assertFalse(self.node._in_sector(
            just_past_front, -self.node.front_half_angle, self.node.front_half_angle))

    def test_side_sectors_cover_nearly_the_full_circle(self):
        # side_sector_max_deg was widened from 100 to 170 degrees after
        # live testing found an obstacle could sweep from the old
        # 80-degree-per-side blind wedge into view only once already very
        # close, while the robot changed heading navigating a bend -- the
        # very first detection for that encounter showed negative
        # computed clearance instead of an early warning. A reading just
        # inside the new limit must fall in a side sector.
        near_limit = math.radians(169.0)
        self.assertTrue(self.node._in_sector(
            near_limit, self.node.front_half_angle, self.node.side_sector_max))
        self.assertTrue(self.node._in_sector(
            -near_limit, -self.node.side_sector_max, -self.node.front_half_angle))

    def test_full_avoidance_cycle_reaches_search_line(self):
        self.node.last_scan = make_scan(0.1)
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.STOP)

        time.sleep(0.06)
        self.node.last_scan = make_scan(5.0)
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.TURN)

        time.sleep(0.06)
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.FORWARD)

        # forward_distance defaults to 1.0m; without odometry updates
        # _distance_travelled() stays 0, so completion falls back to the
        # (shrunk) forward_time timer.
        time.sleep(0.06)
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.TURN_BACK)

        time.sleep(0.06)
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.SEARCH_LINE)

    def test_turn_waits_for_actual_rotation_not_just_timer(self):
        # Real acceleration lag means commanded angular velocity x time
        # consistently under-rotates relative to the nominal angle -- TURN
        # must keep turning until odometry confirms the intended angle was
        # actually reached, not exit once a timer expires while the robot
        # is still mid-turn (which starves FORWARD of real lateral
        # clearance even though FORWARD's own distance check is satisfied).
        self.node.turn_time_cap = 5.0  # long enough that only rotation should trigger the exit
        self.node.intended_turn_angle = 0.5
        self.node.state = ObstacleAvoid.TURN
        self.node.state_until = self.node._current_time() + Duration(seconds=5.0)
        self.node.turn_dir = 1.0
        self.node.turn_start_yaw = 0.0
        self.node.current_yaw = 0.2  # short of the intended 0.5 rad
        self.node.last_scan = make_scan(5.0)
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.TURN)

        self.node.current_yaw = 0.6  # now past the intended angle
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.FORWARD)

    def test_forward_exits_early_once_distance_travelled(self):
        # FORWARD should exit as soon as odometry confirms genuine
        # along-track clearance, without waiting for forward_time to
        # elapse -- this is what makes it robust to accumulated turn error
        # instead of assuming a fixed timer always corresponds to having
        # cleared the obstacle.
        self.node.forward_time = 5.0  # long enough that only distance should trigger the exit
        self.node.forward_distance = 1.0
        self.node.state = ObstacleAvoid.FORWARD
        self.node.state_until = self.node._current_time() + Duration(seconds=5.0)
        self.node.forward_start_x = 0.0
        self.node.forward_start_y = 0.0
        self.node.current_x = 1.5
        self.node.current_y = 0.0
        self.node.last_scan = make_scan(5.0)
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.TURN_BACK)

    def test_turn_back_servos_toward_measured_target_yaw(self):
        # TURN_BACK must keep correcting while actual (odometry-measured)
        # yaw disagrees with the pre-obstacle target, regardless of how it
        # got there.
        published = []
        self.node.cmd_pub.publish = published.append
        self.node.state = ObstacleAvoid.TURN_BACK
        self.node.target_yaw = 1.0
        self.node.current_yaw = 0.0
        self.node.last_scan = make_scan(5.0)

        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.TURN_BACK)
        self.assertGreater(published[-1].angular.z, 0.0)

        self.node.current_yaw = 1.0  # now matches target within tolerance
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.SEARCH_LINE)

    def test_turn_back_falls_back_to_search_without_odom(self):
        self.node.state = ObstacleAvoid.TURN_BACK
        self.node.target_yaw = None
        self.node.current_yaw = None
        self.node.last_scan = make_scan(5.0)
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.SEARCH_LINE)

    def test_search_line_returns_to_idle_once_line_reacquired(self):
        self.node.state = ObstacleAvoid.SEARCH_LINE
        self.node.line_search_confirm_count = 2
        self.node.last_scan = make_scan(5.0)
        self.node.on_line_error(Float32(data=0.0))
        self.node.on_timer()
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.IDLE)

    def test_select_turn_direction_prefers_more_open_side(self):
        self.node._select_turn_direction(left=5.0, right=1.0)
        self.assertEqual(self.node.turn_dir, 1.0)
        self.node._select_turn_direction(left=1.0, right=5.0)
        self.assertEqual(self.node.turn_dir, -1.0)

    def test_select_turn_direction_holds_within_hysteresis_band(self):
        # Readings this close together are within sensor/geometry noise --
        # flipping which way to turn on an essentially-tied reading was
        # observed (live testing) to make some retry-heavy episodes
        # oscillate between directions instead of making progress.
        self.node.turn_direction_hysteresis = 0.10
        self.node.turn_dir = -1.0
        self.node._select_turn_direction(left=1.0, right=0.95)  # 0.05 < hysteresis
        self.assertEqual(self.node.turn_dir, -1.0)
        self.node._select_turn_direction(left=1.0, right=0.85)  # 0.15 > hysteresis
        self.assertEqual(self.node.turn_dir, 1.0)

    def test_emergency_during_forward_recomputes_turn_direction(self):
        # A stale turn_dir left over from the original detection would make
        # repeated emergency-triggered retries all turn the same way
        # regardless of what's actually still in front of the robot now.
        calls = []
        self.node._select_turn_direction = lambda left, right: calls.append((left, right))
        self.node.state = ObstacleAvoid.FORWARD
        self.node.state_until = self.node._current_time() + Duration(seconds=1.0)
        self.node.last_scan = make_scan(0.05)
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.STOP)
        self.assertEqual(len(calls), 1)

    def test_emergency_fires_during_forward_when_side_gets_close(self):
        # FORWARD starts right after a TURN, so whatever was dodged is now
        # off to one side of the new heading rather than dead ahead --
        # checking only 'front' let a straight translation clip it without
        # ever registering as close.
        calls = []
        self.node._select_turn_direction = lambda left, right: calls.append((left, right))
        self.node.state = ObstacleAvoid.FORWARD
        self.node.state_until = self.node._current_time() + Duration(seconds=1.0)
        self.node.last_scan = make_scan(5.0)
        self.node._scan_distances = lambda: (5.0, 0.05, 5.0)
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.STOP)
        self.assertEqual(len(calls), 1)

    def test_emergency_fires_during_turn_when_side_gets_close(self):
        # TURN used to publish blindly for its full duration with no
        # proximity check at all -- an obstacle that swings into range as
        # the robot rotates went completely unnoticed until FORWARD.
        calls = []
        self.node._select_turn_direction = lambda left, right: calls.append((left, right))
        self.node.state = ObstacleAvoid.TURN
        self.node.state_until = self.node._current_time() + Duration(seconds=1.0)
        self.node.turn_dir = 1.0
        self.node.last_scan = make_scan(5.0)
        self.node._scan_distances = lambda: (5.0, 0.05, 5.0)
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.STOP)
        self.assertEqual(len(calls), 1)

    def test_stop_always_backs_off(self):
        # STOP must command a real reverse, not just zero velocity: if it
        # was entered because the robot is already inside
        # emergency_distance, committing straight to another turn from a
        # zero-velocity stop can immediately re-trip the emergency check
        # before any rotation happens.
        published = []
        self.node.cmd_pub.publish = published.append
        self.node.state = ObstacleAvoid.STOP
        self.node.state_until = self.node._current_time() + Duration(seconds=1.0)
        self.node.last_scan = make_scan(5.0)
        self.node.on_timer()
        self.assertLess(published[-1].linear.x, 0.0)
        self.assertEqual(self.node.state, ObstacleAvoid.STOP)

    def test_stop_duration_grows_with_consecutive_retries(self):
        # A fixed-length back-off was found (live testing) to sometimes be
        # insufficient when the obstacle ends up beside rather than ahead
        # of the robot -- each consecutive emergency-triggered retry within
        # one episode should get a longer back-off, capped at
        # back_off_max_time_sec.
        self.node.retry_count = 0
        self.node._enter_stop(is_retry=True)
        first_duration = self.node.state_until - self.node._current_time()
        self.node._enter_stop(is_retry=True)
        second_duration = self.node.state_until - self.node._current_time()
        self.assertGreater(
            second_duration.nanoseconds, first_duration.nanoseconds)

    def test_retry_count_resets_after_clean_forward_completion(self):
        self.node.retry_count = 3
        self.node.state = ObstacleAvoid.FORWARD
        self.node.state_until = self.node._current_time() - Duration(seconds=0.01)
        self.node.last_scan = make_scan(5.0)
        self.node.on_timer()
        self.assertEqual(self.node.retry_count, 0)

    def test_search_line_starts_toward_the_side_the_line_is_actually_on(self):
        # A dodge that turned left (turn_dir > 0) then drove forward along
        # that heading ends up with the line off to the right of the
        # restored heading, not the left -- starting the search sweep
        # left instead first rotates away from the line before ever
        # turning toward it.
        self.node.current_yaw = 0.0
        self.node.turn_dir = 1.0  # dodged left
        self.node._enter_state(ObstacleAvoid.SEARCH_LINE)
        self.assertEqual(self.node.search_dir, -1.0)

        self.node.turn_dir = -1.0  # dodged right
        self.node._enter_state(ObstacleAvoid.SEARCH_LINE)
        self.assertEqual(self.node.search_dir, 1.0)

    def test_search_line_reverses_direction_at_yaw_bound(self):
        # A straight, featureless line looks identical whether the robot
        # faces along it or the exact opposite way, so an unbounded search
        # spin can lock onto the line facing backward. The sweep must stay
        # bounded around the heading TURN_BACK already restored
        # (search_start_yaw) rather than rotate freely.
        self.node.current_yaw = 0.0
        self.node._enter_state(ObstacleAvoid.SEARCH_LINE)
        self.node.search_max_yaw_deviation = math.radians(10.0)
        self.node.last_scan = make_scan(5.0)

        self.node.current_yaw = math.radians(15.0)  # past the bound
        self.node.on_timer()
        self.assertEqual(self.node.search_dir, -1.0)

        self.node.current_yaw = math.radians(-15.0)  # past the bound the other way
        self.node.on_timer()
        self.assertEqual(self.node.search_dir, 1.0)

    def test_search_line_gives_up_after_timeout(self):
        # If the line genuinely can't be found within the bounded sweep,
        # give up and hand back to IDLE rather than sweep back and forth
        # forever.
        self.node.current_yaw = 0.0
        self.node._enter_state(ObstacleAvoid.SEARCH_LINE)
        self.node.search_timeout = 0.05
        self.node.last_scan = make_scan(5.0)
        time.sleep(0.06)
        self.node.on_timer()
        self.assertEqual(self.node.state, ObstacleAvoid.IDLE)

    def test_search_turn_speed_is_slower_than_dodge_turn_speed(self):
        # SEARCH_LINE spins the robot in place to reacquire the line; the
        # onboard camera's zero-pitch mount makes detected line position
        # very sensitive to yaw rate, so this must stay well below the
        # (faster, already-tuned) obstacle-dodge turn_speed or the search
        # never settles long enough to confirm the line is found.
        self.assertLess(self.node.search_turn_speed, self.node.turn_speed)


if __name__ == '__main__':
    unittest.main()
