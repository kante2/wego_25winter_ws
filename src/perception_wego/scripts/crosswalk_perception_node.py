#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Bool, Float64


class CrosswalkPerceptionNode:
    def __init__(self):
        rospy.init_node("crosswalk_perception_node")

        # ---------- params ----------
        self.show_window = rospy.get_param("~show_window", True)
        self.enabled = True

        self.white_ratio_threshold = float(rospy.get_param("~white_ratio_threshold", 0.15))

        self.roi_top_y_ratio     = float(rospy.get_param("~roi_top_y_ratio",     0.60))
        self.roi_left_top_ratio  = float(rospy.get_param("~roi_left_top_ratio",  0.22))
        self.roi_right_top_ratio = float(rospy.get_param("~roi_right_top_ratio", 0.78))
        self.roi_left_bot_ratio  = float(rospy.get_param("~roi_left_bot_ratio", -0.40))
        self.roi_right_bot_ratio = float(rospy.get_param("~roi_right_bot_ratio", 1.40))

        # 송도/대회장 전환
        self.use_yellow = bool(rospy.get_param("~use_yellow_lanes", False))
        self.use_white  = bool(rospy.get_param("~use_white_lanes",  True))

        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_rect_color")

        self.crosswalk_topic   = rospy.get_param("~crosswalk_topic",   "/crosswalk_detected")
        self.white_ratio_topic = rospy.get_param("~white_ratio_topic", "/perception/white_ratio")
        self.enable_topic      = rospy.get_param("~enable_topic",      "/perception/crosswalk/enable")

        # ---------- HSV ranges (C++ 동일) ----------
        self.yellow_lower = np.array([10, 80, 60], dtype=np.uint8)
        self.yellow_upper = np.array([45, 255, 255], dtype=np.uint8)

        self.white_lower = np.array([0, 0, 170], dtype=np.uint8)
        self.white_upper = np.array([179, 60, 255], dtype=np.uint8)

        self.bridge = CvBridge()

        # ---------- pub/sub ----------
        self.pub_crosswalk = rospy.Publisher(self.crosswalk_topic, Bool, queue_size=1)
        self.pub_white_ratio = rospy.Publisher(self.white_ratio_topic, Float64, queue_size=1)

        self.sub_enable = rospy.Subscriber(self.enable_topic, Bool, self.enable_cb, queue_size=1)
        self.sub_image  = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=2)

        # ---------- debug window ----------
        self.win_bev = "crosswalk_bev"
        if self.show_window:
            cv2.namedWindow(self.win_bev, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.win_bev, 960, 540)

        rospy.loginfo("crosswalk_perception_node running...")
        rospy.loginfo("  use_yellow_lanes = %s", "true" if self.use_yellow else "false")
        rospy.loginfo("  use_white_lanes  = %s", "true" if self.use_white else "false")
        rospy.loginfo("  white_ratio_threshold = %.3f", self.white_ratio_threshold)

    def enable_cb(self, msg: Bool):
        self.enabled = bool(msg.data)

    def image_cb(self, msg: Image):
        if not self.enabled:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logwarn_throttle(1.0, "CvBridgeError: %s", str(e))
            return

        if bgr is None or bgr.size == 0:
            return

        if len(bgr.shape) == 2 or (len(bgr.shape) == 3 and bgr.shape[2] == 1):
            bgr = cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)

        h, w = bgr.shape[:2]

        roi_poly = self.make_roi_polygon(h, w)
        bev_bgr = self.warp_to_bev(bgr, roi_poly)
        bev_binary = self.binarize_lanes(bev_bgr)

        white_ratio = self.compute_white_ratio(bev_binary)
        self.pub_white_ratio.publish(Float64(data=white_ratio))

        crosswalk_detected = (white_ratio > self.white_ratio_threshold)
        self.pub_crosswalk.publish(Bool(data=crosswalk_detected))

        if self.show_window:
            cv2.imshow(self.win_bev, bev_binary)
            cv2.waitKey(1)

    def compute_white_ratio(self, binary: np.ndarray) -> float:
        h, w = binary.shape[:2]
        if w <= 0 or h <= 0:
            return 0.0
        total_pixels = int(w * h)
        total_white = int(cv2.countNonZero(binary))
        ratio = float(total_white) / float(total_pixels)

        rospy.loginfo_throttle(
            1.0,
            "[crosswalk_perception] white_ratio=%.3f thr=%.3f",
            ratio, self.white_ratio_threshold
        )
        return ratio

    def make_roi_polygon(self, h: int, w: int):
        y_top = int(h * self.roi_top_y_ratio)
        y_bot = h - 1
        x_lt = int(w * self.roi_left_top_ratio)
        x_rt = int(w * self.roi_right_top_ratio)
        x_lb = int(w * self.roi_left_bot_ratio)
        x_rb = int(w * self.roi_right_bot_ratio)

        # BL, TL, TR, BR
        return [(x_lb, y_bot), (x_lt, y_top), (x_rt, y_top), (x_rb, y_bot)]

    def warp_to_bev(self, bgr: np.ndarray, roi_poly):
        h, w = bgr.shape[:2]

        BL = np.array(roi_poly[0], dtype=np.float32)
        TL = np.array(roi_poly[1], dtype=np.float32)
        TR = np.array(roi_poly[2], dtype=np.float32)
        BR = np.array(roi_poly[3], dtype=np.float32)

        # y만 클리핑 (C++ 동일)
        BL[1] = np.clip(BL[1], 0.0, float(h - 1))
        TL[1] = np.clip(TL[1], 0.0, float(h - 1))
        TR[1] = np.clip(TR[1], 0.0, float(h - 1))
        BR[1] = np.clip(BR[1], 0.0, float(h - 1))

        src = np.array([BL, TL, TR, BR], dtype=np.float32)
        dst = np.array([[0, h - 1], [0, 0], [w - 1, 0], [w - 1, h - 1]], dtype=np.float32)

        M = cv2.getPerspectiveTransform(src, dst)
        bev = cv2.warpPerspective(
            bgr, M, (w, h),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0)
        )
        return bev

    def binarize_lanes(self, bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        mask_y = None
        mask_w = None

        if self.use_yellow:
            mask_y = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
            mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, kernel, iterations=1)

        if self.use_white:
            mask_w = cv2.inRange(hsv, self.white_lower, self.white_upper)
            mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_OPEN, kernel, iterations=1)

        if mask_y is not None and mask_w is not None:
            return cv2.bitwise_or(mask_y, mask_w)
        if mask_y is not None:
            return mask_y
        if mask_w is not None:
            return mask_w

        return np.zeros(hsv.shape[:2], dtype=np.uint8)

    def spin(self):
        rospy.spin()
        if self.show_window:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    node = CrosswalkPerceptionNode()
    node.spin()
