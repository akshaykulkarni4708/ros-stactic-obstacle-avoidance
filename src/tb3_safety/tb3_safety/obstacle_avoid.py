#!/usr/bin/env python3
# LiDAR-based static obstacle avoidance for TurtleBot3.
#
# Reactive state machine: detect an obstacle ahead, stop, turn away from
# it, drive past it, turn back to the original heading, then spin in
# place to reacquire the line before handing control back to
# line-following. Matches the project proposal's own scope for this
# component ("use LiDAR to detect obstacles; if an obstacle is detected,
# avoid it and then return to following the line") and its fallback
# ("use simpler control logic if integration is complex"): dodge geometry
# is sized generously so a single clean attempt is the normal case, and
# recovery after the dodge relies on restoring heading plus a
# camera-based line search rather than a closed-loop path-following
# controller.

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class ObstacleAvoid(Node):
    IDLE = 0
    STOP = 1
    TURN = 2
    FORWARD = 3
    TURN_BACK = 4
    SEARCH_LINE = 5

    def __init__(self):
        super().__init__('obstacle_avoid')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_obstacle')
        self.declare_parameter('line_error_topic', '/line_error')

        # The front sector spans +/-front_half_angle_deg; the side sectors
        # start exactly where the front sector ends and run out to
        # side_sector_max_deg on each side, so there is never an angular
        # gap between them that an obstacle could sit in undetected.
        #
        # side_sector_max_deg was originally 100 degrees, leaving an
        # 80-degree blind wedge toward the rear on each side (160 degrees
        # of the robot's surroundings never measured at all). Live testing
        # at a bend -- where the robot changes heading while navigating
        # past an obstacle -- found this let an obstacle sweep from that
        # blind wedge into view only once already very close: the very
        # first 'OBSTACLE detected' reading for that encounter showed
        # negative computed clearance (contact already made) rather than
        # the expected early warning. Widened to cover nearly the full
        # circle (a 10-degree blind wedge directly behind on each side is
        # what's left, which a forward-moving robot is in no danger of
        # backing into) so a heading change can no longer hide an
        # obstacle from the safety checks.
        self.declare_parameter('front_half_angle_deg', 25.0)
        self.declare_parameter('side_sector_max_deg', 170.0)

        # The LDS-01 (base_scan) is mounted ~0.064m behind the robot's
        # center while the Waffle Pi's hull extends ~0.14m ahead of it, so
        # a raw scan range overstates true clearance to the hull by about
        # 0.20m. Every distance used below is meant to represent hull
        # clearance, so this margin is subtracted from every sector
        # reading up front, once.
        self.declare_parameter('lidar_to_hull_margin', 0.20)

        # avoid_distance sets how far from the obstacle TURN begins, which
        # sets how much perpendicular clearance the dodge buys during the
        # pass: for turn angle theta, closest approach is approximately
        # (avoid_distance + hull_half_length + obstacle_half_depth) *
        # sin(theta), minus the obstacle's and the robot's own
        # half-widths. IDLE also reuses this threshold to decide whether
        # the robot is still too close once the dodge finishes, and the
        # resting distance from the obstacle (dominated by
        # forward_distance_m * sin(theta)) needs to clear it with margin.
        # This gives ~0.38m closest-approach clearance on paper, comfortably
        # above emergency_distance.
        #
        # A larger avoid_distance/forward_distance_m was tried after live
        # testing found genuine negative clearance (real contact) during
        # retry-heavy episodes near a harder obstacle -- but that made
        # every dodge bigger and more sluggish (a much larger lateral
        # drift that left SEARCH_LINE spinning in place for minutes
        # afterward, elsewhere reported as "the robot just stood there"),
        # which is a worse trade for a benefit the *recovery* fixes below
        # turned out to already provide on their own: the retry-heavy
        # episodes were getting stuck not because avoid_distance was too
        # small, but because the old back-off (0.08 m/s, capped at 3s)
        # couldn't create real separation once close, and turn_dir
        # flip-flopped between retries instead of committing to an escape
        # direction. With those two fixed, the original, tighter
        # avoid_distance/emergency_distance keep the dodge compact while
        # the recovery path is what actually prevents contact.
        self.declare_parameter('avoid_distance', 0.80)
        self.declare_parameter('emergency_distance', 0.25)

        # STOP always includes a brief reverse nudge, not just a pause: if
        # this state was entered because TURN or FORWARD got too close,
        # the robot may already be inside emergency_distance, and
        # committing straight to another turn from there can immediately
        # re-trip the emergency check before any rotation happens. A
        # reverse guarantees real separation first. back_off_growth_sec
        # lengthens that reverse on each consecutive retry within the same
        # episode (capped at back_off_max_time_sec) for the case where the
        # obstacle ends up beside rather than ahead of the robot, where a
        # short reverse was found to not always be enough; it resets once
        # a dodge actually clears the obstacle. Both back_off_speed and
        # back_off_max_time_sec were raised after live testing showed a
        # too-timid reverse (0.08 m/s, capped at 3s) failing to create
        # real separation once the hull was already touching an
        # obstacle -- a firmer, longer reverse (up to 0.6m total) is
        # needed to reliably break contact rather than just reduce it.
        # This -- not a bigger base geometry -- is what actually stops the
        # retry loop from drifting into contact.
        self.declare_parameter('stop_time_sec', 0.60)
        self.declare_parameter('back_off_speed', 0.12)
        self.declare_parameter('back_off_growth_sec', 0.40)
        self.declare_parameter('back_off_max_time_sec', 5.00)

        # Readings this close together are within sensor/geometry noise;
        # requiring a clear margin before switching which way to turn
        # stops the direction from flip-flopping retry to retry when
        # left/right are nearly tied, which was observed to make some
        # retry-heavy episodes oscillate instead of making progress.
        self.declare_parameter('turn_direction_hysteresis_m', 0.10)

        # ~40 degrees at turn_speed: steep enough to clear a compact
        # obstacle's ~0.2m half-width plus the robot's own footprint,
        # shallow enough that most of forward_distance_m still counts as
        # progress past the obstacle (cos(40deg) ~= 0.77 vs cos(85deg)
        # ~= 0.09 for a near-right-angle turn, which would waste almost
        # all of that distance and require re-approaching the obstacle).
        self.declare_parameter('turn_speed', 0.30)
        self.declare_parameter('turn_time_sec', 2.30)

        self.declare_parameter('forward_speed', 0.12)
        # Exit FORWARD once the robot has genuinely travelled this far
        # (by odometry), not just after a fixed timer. Sized so the
        # ALONG-TRACK component (forward_distance_m * cos(turn angle))
        # carries the robot's hull past the obstacle's FAR edge, not just
        # past its half-depth -- 1.60m undershot this in live testing:
        # the robot's resting x-position ended up only barely past the
        # obstacle's near edge, so getting back onto the line (which
        # passes through the obstacle's position) immediately walked it
        # back into the same obstacle from the side, repeatedly, and the
        # camera lost the line entirely during the resulting churn. The
        # LATERAL component (forward_distance_m * sin(turn angle)) also
        # clears the obstacle's half-width with room to spare at this
        # value, and is what IDLE's re-detection check (see avoid_distance
        # above) depends on. Bigger than this was tried (2.40m) but its
        # extra lateral drift left SEARCH_LINE spinning in place for
        # minutes trying to reacquire the line, for no matching safety
        # benefit -- kept at the smaller value that still clears the
        # obstacle. forward_time_sec is only a safety cap in case odometry
        # is unavailable.
        self.declare_parameter('forward_distance_m', 2.20)
        self.declare_parameter('forward_time_sec', 15.00)

        # TURN_BACK restores heading via measured odometry yaw rather than
        # a timed turn: commanded angular velocity x time doesn't account
        # for real acceleration lag, so a dead-reckoned "undo" can leave
        # the robot facing the wrong way.
        self.declare_parameter('yaw_tolerance_deg', 5.0)

        # Deliberately slower than turn_speed: the onboard camera is
        # mounted with zero pitch, so the ground horizon sits at the
        # image's vertical midpoint and pixels near the top of the
        # cropped ROI correspond to line segments far down the track --
        # optically very sensitive to yaw rate. Spinning to search at the
        # full dodge turn_speed swings the detected line position too
        # fast to ever hold steady long enough to confirm reacquisition.
        self.declare_parameter('search_turn_speed', 0.12)
        self.declare_parameter('line_search_error_threshold', 20.0)
        self.declare_parameter('line_search_confirm_count', 8)

        # A straight, featureless line looks identical in the camera
        # whether the robot is facing along it or exactly the opposite way,
        # so "the line looks centered" alone is not enough to confirm
        # reacquisition -- an unbounded search spin can (and, in testing,
        # did) rotate almost 180 degrees past the heading TURN_BACK just
        # restored and lock onto the line facing backward, sending the
        # robot the entire length of the track the wrong way. Bounding how
        # far SEARCH_LINE is allowed to sweep away from the heading it
        # entered with (which TURN_BACK already aligned to the pre-dodge
        # direction of travel) keeps every candidate detection within a
        # window that's still facing forward. search_timeout_sec is a
        # fallback in case the line genuinely can't be seen within that
        # window (e.g. odometry drift) -- give up and hand back to IDLE
        # rather than sweep back and forth forever.
        self.declare_parameter('search_max_yaw_deviation_deg', 130.0)
        self.declare_parameter('search_timeout_sec', 45.0)

        self.declare_parameter('publish_rate_hz', 20.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.line_error_topic = self.get_parameter('line_error_topic').value

        self.front_half_angle = math.radians(
            float(self.get_parameter('front_half_angle_deg').value))
        self.side_sector_max = math.radians(
            float(self.get_parameter('side_sector_max_deg').value))
        self.lidar_to_hull_margin = float(self.get_parameter('lidar_to_hull_margin').value)

        self.avoid_distance = float(self.get_parameter('avoid_distance').value)
        self.emergency_distance = float(self.get_parameter('emergency_distance').value)

        self.stop_time = float(self.get_parameter('stop_time_sec').value)
        self.back_off_speed = float(self.get_parameter('back_off_speed').value)
        self.back_off_growth = float(self.get_parameter('back_off_growth_sec').value)
        self.back_off_max_time = float(self.get_parameter('back_off_max_time_sec').value)
        self.turn_direction_hysteresis = float(
            self.get_parameter('turn_direction_hysteresis_m').value)

        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.turn_time = float(self.get_parameter('turn_time_sec').value)
        self.intended_turn_angle = self.turn_speed * self.turn_time
        # TURN's real exit condition is achieving intended_turn_angle (see
        # on_timer); turn_time alone would consistently fire first under
        # real acceleration lag and mask that check, so the duration
        # passed to _enter_state is padded to actually give the angle
        # check room to be the one that governs -- this remains a safety
        # cap, not the primary exit condition.
        self.turn_time_cap = self.turn_time * 1.6

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.forward_distance = float(self.get_parameter('forward_distance_m').value)
        self.forward_time = float(self.get_parameter('forward_time_sec').value)

        self.yaw_tolerance = math.radians(float(self.get_parameter('yaw_tolerance_deg').value))

        self.search_turn_speed = float(self.get_parameter('search_turn_speed').value)
        self.line_search_error_threshold = float(
            self.get_parameter('line_search_error_threshold').value)
        self.line_search_confirm_count = int(
            self.get_parameter('line_search_confirm_count').value)
        self.search_max_yaw_deviation = math.radians(
            float(self.get_parameter('search_max_yaw_deviation_deg').value))
        self.search_timeout = float(self.get_parameter('search_timeout_sec').value)

        publish_rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.line_error_sub = self.create_subscription(
            Float32, self.line_error_topic, self.on_line_error, 10)

        self.timer = self.create_timer(1.0 / publish_rate, self.on_timer)

        self.last_scan = None
        self.line_error = None
        self.line_seen_count = 0
        self.state = self.IDLE
        self.state_until = self.get_clock().now()
        self.turn_dir = 1.0
        self.current_yaw = None
        self.target_yaw = None
        self.current_x = None
        self.current_y = None
        self.forward_start_x = None
        self.forward_start_y = None
        self.turn_start_yaw = None
        self.retry_count = 0
        self.search_start_yaw = None
        self.search_dir = 1.0
        self.search_entered_time = None
        self.get_logger().info(
            f'ObstacleAvoid started: scan={self.scan_topic}, cmd_vel={self.cmd_vel_topic}, '
            f'avoid={self.avoid_distance:.2f}, emergency={self.emergency_distance:.2f}, '
            f'turn={self.turn_time:.2f}s @ {self.turn_speed:.2f} rad/s, '
            f'forward={self.forward_distance:.2f}m @ {self.forward_speed:.2f} m/s'
        )

    def on_scan(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def on_line_error(self, msg: Float32) -> None:
        if math.isfinite(msg.data) and msg.data != -1.0:
            self.line_error = float(msg.data)
        else:
            self.line_error = None

    def _current_time(self):
        return self.get_clock().now()

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _in_sector(self, angle, start, end):
        angle = self._normalize_angle(angle)
        start = self._normalize_angle(start)
        end = self._normalize_angle(end)
        if start <= end:
            return start <= angle <= end
        return angle >= start or angle <= end

    def _sector_min(self, ranges, angle_min, angle_inc, start, end):
        if not ranges or angle_inc == 0.0:
            return float('inf')
        minimum = float('inf')
        for index, value in enumerate(ranges):
            if not math.isfinite(value) or value <= 0.0:
                continue
            angle = angle_min + index * angle_inc
            if self._in_sector(angle, start, end):
                minimum = min(minimum, value)
        return minimum

    def _scan_distances(self):
        scan = self.last_scan
        if scan is None:
            return float('inf'), float('inf'), float('inf')
        front = self._sector_min(
            scan.ranges, scan.angle_min, scan.angle_increment,
            -self.front_half_angle, self.front_half_angle)
        left = self._sector_min(
            scan.ranges, scan.angle_min, scan.angle_increment,
            self.front_half_angle, self.side_sector_max)
        right = self._sector_min(
            scan.ranges, scan.angle_min, scan.angle_increment,
            -self.side_sector_max, -self.front_half_angle)
        margin = self.lidar_to_hull_margin
        return front - margin, left - margin, right - margin

    def _publish(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _enter_state(self, state: int, duration_sec: float = 0.0) -> None:
        self.state = state
        self.state_until = self._current_time() + Duration(seconds=duration_sec)
        if state == self.STOP:
            self.get_logger().info('STATE=STOP')
        elif state == self.TURN:
            direction = 'left' if self.turn_dir > 0 else 'right'
            self.get_logger().info(f'STATE=TURN direction={direction}')
        elif state == self.FORWARD:
            self.get_logger().info('STATE=FORWARD')
        elif state == self.TURN_BACK:
            self.get_logger().info('STATE=TURN_BACK')
        elif state == self.SEARCH_LINE:
            self.search_start_yaw = self.current_yaw
            # The dodge turned turn_dir to go around the obstacle and then
            # drove forward along that turned heading, so the line (which
            # runs through the obstacle's position) is now off to the
            # opposite side of the restored heading -- e.g. a left dodge
            # (turn_dir > 0) leaves the robot to the left of the line, which
            # means sweeping right (negative) finds it directly, while
            # starting the sweep left instead first rotates away from the
            # line before ever turning toward it.
            self.search_dir = -self.turn_dir if self.turn_dir != 0.0 else 1.0
            self.search_entered_time = self._current_time()
            self.get_logger().info('STATE=SEARCH_LINE')
        elif state == self.IDLE:
            self.get_logger().info('STATE=IDLE')

    def _select_turn_direction(self, left: float, right: float) -> None:
        if left >= right + self.turn_direction_hysteresis:
            self.turn_dir = 1.0
        elif right >= left + self.turn_direction_hysteresis:
            self.turn_dir = -1.0
        # else: within the hysteresis band -- keep the previous turn_dir
        # rather than flip-flopping on essentially-tied readings.

    def _has_line(self) -> bool:
        return (
            self.line_error is not None
            and abs(self.line_error) <= self.line_search_error_threshold
        )

    def _enter_stop(self, is_retry: bool) -> None:
        if is_retry:
            self.retry_count += 1
        duration = min(
            self.stop_time + self.retry_count * self.back_off_growth,
            self.back_off_max_time,
        )
        self._enter_state(self.STOP, duration)

    def _distance_travelled(self) -> float:
        if self.current_x is None or self.forward_start_x is None:
            return 0.0
        return math.hypot(
            self.current_x - self.forward_start_x, self.current_y - self.forward_start_y)

    def on_timer(self) -> None:
        front, left, right = self._scan_distances()
        now = self._current_time()

        if self.state == self.IDLE:
            self._publish(0.0, 0.0)
            if front < self.avoid_distance:
                self.target_yaw = self.current_yaw
                self.retry_count = 0
                self._select_turn_direction(left, right)
                self.get_logger().warn(
                    f'OBSTACLE detected front={front:.2f} left={left:.2f} right={right:.2f}')
                self._enter_stop(is_retry=False)
            return

        if self.state == self.STOP:
            self._publish(-self.back_off_speed, 0.0)
            if now >= self.state_until:
                self.turn_start_yaw = self.current_yaw
                self._enter_state(self.TURN, self.turn_time_cap)
            return

        if self.state == self.TURN:
            # Turning in place sweeps the robot's corners outward (up to
            # the hull's half-diagonal, wider than the half-length the
            # front sector alone accounts for), so check every direction,
            # not just front, before continuing to rotate.
            if min(front, left, right) < self.emergency_distance:
                self.get_logger().warn(
                    f'EMERGENCY during turn front={front:.2f} left={left:.2f} '
                    f'right={right:.2f}')
                self._select_turn_direction(left, right)
                self._enter_stop(is_retry=True)
                return
            self._publish(0.0, self.turn_dir * self.turn_speed)
            # Exit once the robot has actually rotated the intended angle
            # (by odometry), not just once turn_time has elapsed: real
            # acceleration lag means the commanded angular velocity x time
            # consistently under-rotates relative to the nominal angle,
            # which then starves FORWARD of the lateral clearance it
            # needs even though its own odometry-distance exit condition
            # is satisfied -- live testing traced a same-obstacle
            # re-encounter after a "clean" dodge to exactly this. The
            # timer remains a safety cap for when odometry is unavailable
            # or something is stuck.
            rotated_enough = (
                self.turn_start_yaw is not None
                and self.current_yaw is not None
                and abs(self._normalize_angle(self.current_yaw - self.turn_start_yaw))
                >= self.intended_turn_angle
            )
            if rotated_enough or now >= self.state_until:
                self.forward_start_x = self.current_x
                self.forward_start_y = self.current_y
                self._enter_state(self.FORWARD, self.forward_time)
            return

        if self.state == self.FORWARD:
            # FORWARD starts right after TURN, so the dodged obstacle is
            # now off to one side of the new heading, not ahead of it --
            # checking only 'front' would let a straight translation clip
            # it without ever reading close.
            if min(front, left, right) < self.emergency_distance:
                self.get_logger().warn(
                    f'EMERGENCY obstacle front={front:.2f} left={left:.2f} '
                    f'right={right:.2f}')
                self._select_turn_direction(left, right)
                self._enter_stop(is_retry=True)
                return
            self._publish(self.forward_speed, 0.0)
            if self._distance_travelled() >= self.forward_distance or now >= self.state_until:
                self.retry_count = 0
                self._enter_state(self.TURN_BACK)
            return

        if self.state == self.TURN_BACK:
            if self.current_yaw is None or self.target_yaw is None:
                self._enter_state(self.SEARCH_LINE)
                return
            yaw_error = self._normalize_angle(self.target_yaw - self.current_yaw)
            if abs(yaw_error) <= self.yaw_tolerance:
                self._enter_state(self.SEARCH_LINE)
                return
            self._publish(0.0, math.copysign(self.turn_speed, yaw_error))
            return

        if self.state == self.SEARCH_LINE:
            if self._has_line():
                self.line_seen_count += 1
                if self.line_seen_count >= self.line_search_confirm_count:
                    self.get_logger().info('LINE reacquired')
                    self.line_seen_count = 0
                    self._enter_state(self.IDLE)
                    return
            else:
                self.line_seen_count = 0

            if self.search_entered_time is not None:
                searched_for = (now - self.search_entered_time).nanoseconds / 1e9
                if searched_for >= self.search_timeout:
                    self.get_logger().warn(
                        f'SEARCH_LINE gave up after {searched_for:.1f}s without '
                        'confirming the line -- returning to IDLE instead of '
                        'continuing to sweep.'
                    )
                    self.line_seen_count = 0
                    self._enter_state(self.IDLE)
                    return

            # A straight, featureless line looks the same from either end,
            # so the sweep is bounded to a window around the heading
            # TURN_BACK already restored (search_start_yaw) rather than
            # left free to rotate until anything centers -- otherwise it
            # can lock onto the line facing back the way it came. Once the
            # bound is hit, reverse toward center; line_error only picks
            # the direction inside that window.
            yaw_dev = 0.0
            if self.current_yaw is not None and self.search_start_yaw is not None:
                yaw_dev = self._normalize_angle(self.current_yaw - self.search_start_yaw)

            if yaw_dev >= self.search_max_yaw_deviation:
                self.search_dir = -1.0
            elif yaw_dev <= -self.search_max_yaw_deviation:
                self.search_dir = 1.0
            elif self.line_error is not None:
                self.search_dir = -math.copysign(1.0, self.line_error)

            self._publish(0.0, self.search_dir * self.search_turn_speed)
            return

    def destroy_node(self):
        self._publish(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoid()
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
