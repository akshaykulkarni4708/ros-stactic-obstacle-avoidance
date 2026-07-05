import unittest

import cv2
import numpy as np
import rclpy

from line_follower.line_detector import LineDetector


def make_contour(cx, cy, w=20, h=20, canvas=(100, 200)):
    mask = np.zeros(canvas, dtype=np.uint8)
    x0, y0 = int(cx - w / 2), int(cy - h / 2)
    cv2.rectangle(mask, (x0, y0), (x0 + w, y0 + h), 255, -1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours[0]


class TestLineDetectorLogic(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = LineDetector()

    def tearDown(self):
        self.node.destroy_node()

    def test_select_contour_picks_largest_when_untracked(self):
        small = make_contour(50, 50, 20, 20)
        large = make_contour(150, 50, 60, 60)
        result = self.node._select_contour([small, large], width=200)
        self.assertAlmostEqual(result['cx'], 150, delta=2)

    def test_select_contour_ignores_area_below_min_contour_area(self):
        tiny = make_contour(50, 50, 5, 5)
        result = self.node._select_contour([tiny], width=200)
        self.assertIsNone(result)

    def test_select_contour_prefers_nearest_to_tracked_position(self):
        self.node._tracked_cx = 50.0
        near = make_contour(60, 50, 30, 30)
        far = make_contour(180, 50, 30, 30)
        result = self.node._select_contour([near, far], width=200)
        self.assertAlmostEqual(result['cx'], 60, delta=2)

    def test_accept_candidate_rejects_large_jump_until_confirmed(self):
        self.node._tracked_cx = 100.0
        confirm_frames = self.node._switch_confirm_frames
        result = 100.0
        for _ in range(confirm_frames - 1):
            result = self.node._accept_candidate(300.0)
            self.assertEqual(result, 100.0)
        result = self.node._accept_candidate(300.0)
        self.assertEqual(result, 300.0)

    def test_accept_candidate_accepts_small_jump_immediately(self):
        self.node._tracked_cx = 100.0
        result = self.node._accept_candidate(150.0)
        self.assertEqual(result, 150.0)


if __name__ == '__main__':
    unittest.main()
