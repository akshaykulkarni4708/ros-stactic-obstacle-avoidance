#!/usr/bin/env python3

import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge


class LineDetector(Node):
    def __init__(self):
        super().__init__("line_detector")

        # The TurtleBot3 onboard camera is mounted with zero pitch, so the
        # ground plane's horizon sits at the image's vertical midpoint;
        # anything below it (roi_start ~0.48-0.50) is ground, above it is sky.
        self.declare_parameter("roi_start", 0.48)
        self.declare_parameter("line_is_dark", True)
        self.declare_parameter("use_adaptive", True)
        self.declare_parameter("adaptive_block", 31)
        self.declare_parameter("adaptive_c", 5)
        self.declare_parameter("fixed_thresh", 140)
        self.declare_parameter("use_hsv", True)
        self.declare_parameter("hsv_lower_h", 15)
        self.declare_parameter("hsv_lower_s", 80)
        self.declare_parameter("hsv_lower_v", 80)
        self.declare_parameter("hsv_upper_h", 40)
        self.declare_parameter("hsv_upper_s", 255)
        self.declare_parameter("hsv_upper_v", 255)
        self.declare_parameter("kernel_size", 5)
        self.declare_parameter("lost_sentinel", -1.0)
        self.declare_parameter("min_nonzero", 50)
        self.declare_parameter("max_fill_ratio", 0.60)
        self.declare_parameter("min_contour_area", 250.0)
        self.declare_parameter("ema_alpha", 0.25)
        self.declare_parameter("max_contour_jump", 120.0)
        self.declare_parameter("contour_switch_confirm_frames", 3)

        self._roi_start = float(self.get_parameter("roi_start").value)
        self._line_is_dark = bool(self.get_parameter("line_is_dark").value)
        self._use_adaptive = bool(self.get_parameter("use_adaptive").value)
        self._adaptive_block = int(self.get_parameter("adaptive_block").value)
        self._adaptive_c = int(self.get_parameter("adaptive_c").value)
        self._fixed_thresh = int(self.get_parameter("fixed_thresh").value)
        self._use_hsv = bool(self.get_parameter("use_hsv").value)
        self._hsv_lower = np.array([
            int(self.get_parameter("hsv_lower_h").value),
            int(self.get_parameter("hsv_lower_s").value),
            int(self.get_parameter("hsv_lower_v").value),
        ], dtype=np.uint8)
        self._hsv_upper = np.array([
            int(self.get_parameter("hsv_upper_h").value),
            int(self.get_parameter("hsv_upper_s").value),
            int(self.get_parameter("hsv_upper_v").value),
        ], dtype=np.uint8)
        self._kernel_size = int(self.get_parameter("kernel_size").value)
        self._lost_sentinel = float(self.get_parameter("lost_sentinel").value)
        self._min_nonzero = int(self.get_parameter("min_nonzero").value)
        self._max_fill_ratio = float(self.get_parameter("max_fill_ratio").value)
        self._min_contour_area = float(self.get_parameter("min_contour_area").value)
        self._ema_alpha = float(self.get_parameter("ema_alpha").value)
        self._max_contour_jump = float(self.get_parameter("max_contour_jump").value)
        self._switch_confirm_frames = max(
            1, int(self.get_parameter("contour_switch_confirm_frames").value)
        )

        if self._adaptive_block < 3:
            self._adaptive_block = 3
        if self._adaptive_block % 2 == 0:
            self._adaptive_block += 1

        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.on_image,
            10,
        )
        self.err_pub = self.create_publisher(Float32, "/line_error", 10)
        self.mask_pub = self.create_publisher(Image, "/line_mask", 10)
        self.nonzero_pub = self.create_publisher(Int32, "/line_mask_nonzero", 10)

        self.bridge = CvBridge()
        self.get_logger().info(
            "LineDetector started. Subscribed: /camera/image_raw, "
            "Publishing: /line_error, /line_mask, /line_mask_nonzero"
        )

        self._last_error = self._lost_sentinel
        self._last_nonzero = 0
        self._tracked_cx = None
        self._filtered_error = None
        self._lost_frames = 0
        self._pending_cx = None
        self._pending_count = 0
        self._publish_timer = self.create_timer(0.1, self._publish_latest)
        self._last_log_t = 0.0

    def _publish_latest(self):
        self.err_pub.publish(Float32(data=float(self._last_error)))
        self.nonzero_pub.publish(Int32(data=int(self._last_nonzero)))

    def _threshold_mask(self, roi):
        if self._use_hsv:
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            return cv2.inRange(hsv, self._hsv_lower, self._hsv_upper)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
        if self._use_adaptive:
            return cv2.adaptiveThreshold(
                gray_blur,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV if self._line_is_dark else cv2.THRESH_BINARY,
                self._adaptive_block,
                self._adaptive_c,
            )

        mode = cv2.THRESH_BINARY_INV if self._line_is_dark else cv2.THRESH_BINARY
        _, mask = cv2.threshold(gray_blur, self._fixed_thresh, 255, mode)
        return mask

    def _select_contour(self, contours, width):
        candidates = []
        center_x = width / 2.0
        reference_x = self._tracked_cx if self._tracked_cx is not None else center_x

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self._min_contour_area:
                continue

            moments = cv2.moments(contour)
            if moments["m00"] <= 0.0:
                continue

            cx = float(moments["m10"] / moments["m00"])
            candidates.append({
                "cx": cx,
                "area": area,
                "jump": abs(cx - reference_x),
            })

        if not candidates:
            return None

        if self._tracked_cx is None:
            return max(candidates, key=lambda item: item["area"])

        near_candidates = [
            item for item in candidates if item["jump"] <= self._max_contour_jump
        ]
        pool = near_candidates if near_candidates else candidates
        return min(pool, key=lambda item: (item["jump"], -item["area"]))

    def _accept_candidate(self, candidate_cx):
        if self._tracked_cx is None:
            self._pending_cx = None
            self._pending_count = 0
            return candidate_cx

        jump = abs(candidate_cx - self._tracked_cx)
        if jump <= self._max_contour_jump:
            self._pending_cx = None
            self._pending_count = 0
            return candidate_cx

        if self._pending_cx is not None and abs(candidate_cx - self._pending_cx) <= 25.0:
            self._pending_count += 1
        else:
            self._pending_cx = candidate_cx
            self._pending_count = 1

        if self._pending_count >= self._switch_confirm_frames:
            self._pending_cx = None
            self._pending_count = 0
            return candidate_cx

        return self._tracked_cx

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w, _ = frame.shape

        roi_start_px = int(h * self._roi_start)
        roi_start_px = max(0, min(h - 1, roi_start_px))
        roi = frame[roi_start_px:h, 0:w]

        mask = self._threshold_mask(roi)

        k = max(1, self._kernel_size)
        kernel = np.ones((k, k), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)
        except Exception:
            pass

        nonzero = int(cv2.countNonZero(mask))
        self._last_nonzero = nonzero
        roi_area = float(mask.shape[0] * mask.shape[1])
        fill_ratio = nonzero / roi_area if roi_area > 0.0 else 1.0

        candidate = None
        if nonzero >= self._min_nonzero and fill_ratio <= self._max_fill_ratio:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            candidate = self._select_contour(contours, w)

        if candidate is None:
            self._lost_frames += 1
            self._last_error = self._lost_sentinel
            if self._lost_frames >= 5:
                self._tracked_cx = None
                self._filtered_error = None
                self._pending_cx = None
                self._pending_count = 0
        else:
            self._lost_frames = 0
            effective_cx = self._accept_candidate(candidate["cx"])
            self._tracked_cx = effective_cx
            raw_error = effective_cx - (w / 2.0)
            if self._filtered_error is None:
                self._filtered_error = raw_error
            else:
                alpha = max(0.0, min(1.0, self._ema_alpha))
                self._filtered_error = alpha * raw_error + (1.0 - alpha) * self._filtered_error
            self._last_error = self._filtered_error

        now = time.monotonic()
        if now - self._last_log_t > 1.0:
            self._last_log_t = now
            self.get_logger().info(
                f"roi_start={self._roi_start:.2f}, use_hsv={self._use_hsv}, "
                f"nonzero={nonzero}, fill={fill_ratio:.2f}, "
                f"tracked_cx={self._tracked_cx}, last_error={self._last_error:.1f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
