#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Yellow Lane Filter Debug Node
- BEV 변환 후 노란색 필터 시각화
- 전체 프레임 대비 노란색 픽셀 비율 계산
- 실시간 HSV 범위 조정 (trackbar)
"""

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32


class YellowFilterDebugger:
    def __init__(self):
        rospy.init_node('yellow_filter_debug', anonymous=True)
        
        self.bridge = CvBridge()
        
        # ========== ROI Parameters ==========
        self.roi_top_y_ratio = rospy.get_param('~roi_top_y_ratio', 0.60)
        self.roi_left_top_ratio = rospy.get_param('~roi_left_top_ratio', 0.10)
        self.roi_right_top_ratio = rospy.get_param('~roi_right_top_ratio', 0.85)
        self.roi_left_bot_ratio = rospy.get_param('~roi_left_bot_ratio', -0.45)
        self.roi_right_bot_ratio = rospy.get_param('~roi_right_bot_ratio', 1.40)
        
        # ========== HSV Range (Tunable) ==========
        # Yellow lane: H: 18-38, S: 100-255, V: 110-230
        self.h_lower = 18
        self.h_upper = 38
        self.s_lower = 100
        self.s_upper = 255
        self.v_lower = 110
        self.v_upper = 230
        
        # ========== Subscriber ==========
        self.sub_image = rospy.Subscriber(
            '/usb_cam/image_raw/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        # ========== Publisher ==========
        self.pub_yellow_ratio = rospy.Publisher(
            '/debug/yellow_ratio',
            Float32,
            queue_size=1
        )
        
        # ========== Debug Windows ==========
        cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
        cv2.namedWindow('ROI Polygon', cv2.WINDOW_NORMAL)
        cv2.namedWindow('BEV Image', cv2.WINDOW_NORMAL)
        cv2.namedWindow('HSV', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Yellow Mask', cv2.WINDOW_NORMAL)
        cv2.namedWindow('HSV Range Control', cv2.WINDOW_NORMAL)
        
        cv2.resizeWindow('Original', 640, 480)
        cv2.resizeWindow('ROI Polygon', 640, 480)
        cv2.resizeWindow('BEV Image', 640, 480)
        cv2.resizeWindow('HSV', 640, 480)
        cv2.resizeWindow('Yellow Mask', 640, 480)
        cv2.resizeWindow('HSV Range Control', 500, 300)
        
        # ========== Trackbars for HSV tuning ==========
        cv2.createTrackbar('H Lower', 'HSV Range Control', self.h_lower, 179, self.on_h_lower)
        cv2.createTrackbar('H Upper', 'HSV Range Control', self.h_upper, 179, self.on_h_upper)
        cv2.createTrackbar('S Lower', 'HSV Range Control', self.s_lower, 255, self.on_s_lower)
        cv2.createTrackbar('S Upper', 'HSV Range Control', self.s_upper, 255, self.on_s_upper)
        cv2.createTrackbar('V Lower', 'HSV Range Control', self.v_lower, 255, self.on_v_lower)
        cv2.createTrackbar('V Upper', 'HSV Range Control', self.v_upper, 255, self.on_v_upper)
        
        rospy.loginfo("="*70)
        rospy.loginfo("Yellow Filter Debug Node Started")
        rospy.loginfo("Current HSV Range:")
        rospy.loginfo(f"  H: [{self.h_lower:3d}, {self.h_upper:3d}]")
        rospy.loginfo(f"  S: [{self.s_lower:3d}, {self.s_upper:3d}]")
        rospy.loginfo(f"  V: [{self.v_lower:3d}, {self.v_upper:3d}]")
        rospy.loginfo("="*70)
    
    def on_h_lower(self, val):
        self.h_lower = val
    
    def on_h_upper(self, val):
        self.h_upper = val
    
    def on_s_lower(self, val):
        self.s_lower = val
    
    def on_s_upper(self, val):
        self.s_upper = val
    
    def on_v_lower(self, val):
        self.v_lower = val
    
    def on_v_upper(self, val):
        self.v_upper = val
    
    def make_roi_polygon(self, h, w):
        """Create trapezoid ROI polygon"""
        y_top = int(h * self.roi_top_y_ratio)
        y_bot = h - 1
        x_lt = int(w * self.roi_left_top_ratio)
        x_rt = int(w * self.roi_right_top_ratio)
        x_lb = int(w * self.roi_left_bot_ratio)
        x_rb = int(w * self.roi_right_bot_ratio)
        
        # Clamp to image bounds
        x_lb = max(0, min(w-1, x_lb))
        x_lt = max(0, min(w-1, x_lt))
        x_rt = max(0, min(w-1, x_rt))
        x_rb = max(0, min(w-1, x_rb))
        
        poly = np.array([
            [x_lb, y_bot],
            [x_lt, y_top],
            [x_rt, y_top],
            [x_rb, y_bot]
        ], dtype=np.float32)
        
        return poly
    
    def warp_to_bev(self, bgr, roi_poly):
        """Perspective transform to BEV"""
        h, w = bgr.shape[:2]
        
        src_pts = roi_poly.astype(np.float32)
        dst_pts = np.array([
            [0, h-1],
            [0, 0],
            [w-1, 0],
            [w-1, h-1]
        ], dtype=np.float32)
        
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        bev = cv2.warpPerspective(bgr, M, (w, h),
                                   flags=cv2.INTER_LINEAR,
                                   borderMode=cv2.BORDER_CONSTANT,
                                   borderValue=(0, 0, 0))
        
        return bev
    
    def image_callback(self, msg: CompressedImage):
        """Process image and display debug info"""
        try:
            bgr = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logwarn(f"CvBridgeError: {e}")
            return
        
        if bgr is None or bgr.size == 0:
            return
        
        h, w = bgr.shape[:2]
        
        # ===== 1. Draw ROI on original image =====
        roi_poly = self.make_roi_polygon(h, w)
        roi_vis = bgr.copy()
        roi_points = roi_poly.astype(np.int32)
        cv2.polylines(roi_vis, [roi_points], True, (0, 255, 0), 2)
        
        # ===== 2. Warp to BEV =====
        bev_bgr = self.warp_to_bev(bgr, roi_poly)
        
        # ===== 3. Convert to HSV =====
        hsv = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
        
        # ===== 4. Apply yellow filter with trackbar values =====
        lower = np.array([self.h_lower, self.s_lower, self.v_lower])
        upper = np.array([self.h_upper, self.s_upper, self.v_upper])
        mask = cv2.inRange(hsv, lower, upper)
        
        # ===== 5. Calculate yellow pixel ratio =====
        total_pixels = h * w
        yellow_pixels = cv2.countNonZero(mask)
        yellow_ratio = (yellow_pixels / total_pixels) * 100.0
        
        # ===== 6. Morphological operations =====
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask_processed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # ===== 7. Create visualization =====
        # Yellow mask with contours
        mask_vis = cv2.cvtColor(mask_processed, cv2.COLOR_GRAY2BGR)
        contours, _ = cv2.findContours(mask_processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(mask_vis, contours, -1, (0, 255, 0), 2)
        
        # HSV display
        hsv_vis = hsv.copy()
        cv2.putText(hsv_vis, f"H: {self.h_lower}-{self.h_upper}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(hsv_vis, f"S: {self.s_lower}-{self.s_upper}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(hsv_vis, f"V: {self.v_lower}-{self.v_upper}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Add statistics to images
        stats_text = [
            f"Yellow Pixels: {yellow_pixels}",
            f"Total Pixels: {total_pixels}",
            f"Yellow Ratio: {yellow_ratio:.2f}%"
        ]
        
        # Add stats to original
        roi_vis_copy = roi_vis.copy()
        for i, text in enumerate(stats_text):
            cv2.putText(roi_vis_copy, text, (10, 30 + i*30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Add stats to BEV
        bev_vis = bev_bgr.copy()
        for i, text in enumerate(stats_text):
            cv2.putText(bev_vis, text, (10, 30 + i*30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Add stats to mask
        mask_vis_copy = mask_vis.copy()
        for i, text in enumerate(stats_text):
            cv2.putText(mask_vis_copy, text, (10, 30 + i*30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # ===== 8. Display =====
        cv2.imshow('Original', roi_vis_copy)
        cv2.imshow('ROI Polygon', roi_vis)
        cv2.imshow('BEV Image', bev_vis)
        cv2.imshow('HSV', hsv_vis)
        cv2.imshow('Yellow Mask', mask_vis_copy)
        
        # ===== 9. Log and publish =====
        rospy.loginfo_throttle(1.0,
            f"[Yellow Filter] Ratio: {yellow_ratio:.2f}% | "
            f"Pixels: {yellow_pixels}/{total_pixels} | "
            f"H:[{self.h_lower}-{self.h_upper}] "
            f"S:[{self.s_lower}-{self.s_upper}] "
            f"V:[{self.v_lower}-{self.v_upper}]")
        
        msg_ratio = Float32()
        msg_ratio.data = float(yellow_ratio)
        self.pub_yellow_ratio.publish(msg_ratio)
        
        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User quit")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = YellowFilterDebugger()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
