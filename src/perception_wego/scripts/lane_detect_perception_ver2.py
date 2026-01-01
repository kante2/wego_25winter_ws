#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lane Center Detection Node (BEV + Sliding Window)
- Based on C++ lane_center_node.cpp
- ROI 폴리곤 (사다리꼴) 기반 관심 영역 설정
- Bird's Eye View (BEV) 원근 변환
- 노란 차선 감지 (HSV 필터링)
- 슬라이딩 윈도우로 좌/우 차선 중심 추적
- 곡률 계산 (차선 곡도)
"""

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
import math


class LaneCenterDetector:
    def __init__(self):
        rospy.init_node('lane_detect_perception_ver2', anonymous=True)
        
        self.bridge = CvBridge()
        
        # ========== 파라미터 ==========
        # Lane width in pixels (BEV space)
        self.lane_width_px = rospy.get_param('~lane_width_px', 340.0)
        
        # Sliding window parameters
        self.num_windows = rospy.get_param('~num_windows', 12)
        self.window_margin = rospy.get_param('~window_margin', 80)
        self.minpix_recenter = rospy.get_param('~minpix_recenter', 50)
        self.min_lane_sep = rospy.get_param('~min_lane_sep', 80)
        self.center_ema_alpha = rospy.get_param('~center_ema_alpha', 0.8)
        
        # ROI polygon ratios
        self.roi_top_y_ratio = rospy.get_param('~roi_top_y_ratio', 0.60)
        self.roi_left_top_ratio = rospy.get_param('~roi_left_top_ratio', 0.10)
        self.roi_right_top_ratio = rospy.get_param('~roi_right_top_ratio', 0.85)
        self.roi_left_bot_ratio = rospy.get_param('~roi_left_bot_ratio', -0.45)
        self.roi_right_bot_ratio = rospy.get_param('~roi_right_bot_ratio', 1.40)
        
        # HSV range for WHITE lane detection (inha25-winter-ros 기준)
        self.white_lower = np.array([0, 0, 200])
        self.white_upper = np.array([180, 30, 255])
        
        # ========== Publishers ==========
        # Ver1 토픽으로 발행 (BEV 기반)
        self.pub_steering_offset = rospy.Publisher(
            '/webot/steering_offset',
            Float32,
            queue_size=1
        )
        self.pub_lane_speed = rospy.Publisher(
            '/webot/lane_speed',
            Float32,
            queue_size=1
        )
        
        # Debug visualization
        self.pub_bev_image = rospy.Publisher(
            '/webot/lane_bev_debug',
            Image,
            queue_size=1
        )
        
        # ========== Subscriber ==========
        self.image_sub = rospy.Subscriber(
            '/usb_cam/image_raw/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo("="*60)
        rospy.loginfo("Lane Center Detector (BEV + Sliding Window) initialized")
        rospy.loginfo(f"ROI Top Y Ratio: {self.roi_top_y_ratio}")
        rospy.loginfo(f"Num Windows: {self.num_windows}")
        rospy.loginfo(f"Lane Width (px): {self.lane_width_px}")
        rospy.loginfo("Steering Offset topic: /webot/steering_offset (BEV-based)")
        rospy.loginfo("Lane Speed topic: /webot/lane_speed")
        rospy.loginfo("="*60)
    
    def make_roi_polygon(self, h, w):
        """
        Create trapezoid ROI polygon
        Returns: list of 4 corner points [(x_lb, y_bot), (x_lt, y_top), (x_rt, y_top), (x_rb, y_bot)]
        """
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
        """
        Perspective transform to Bird's Eye View
        roi_poly: 4x2 array of corner points
        """
        h, w = bgr.shape[:2]
        
        # Source points (image frame)
        src_pts = roi_poly.astype(np.float32)
        
        # Destination points (BEV frame - rectangle)
        dst_pts = np.array([
            [0, h-1],       # bottom-left
            [0, 0],         # top-left
            [w-1, 0],       # top-right
            [w-1, h-1]      # bottom-right
        ], dtype=np.float32)
        
        # Get perspective transform matrix
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        
        # Warp perspective
        bev = cv2.warpPerspective(bgr, M, (w, h),
                                   flags=cv2.INTER_LINEAR,
                                   borderMode=cv2.BORDER_CONSTANT,
                                   borderValue=(0, 0, 0))
        
        return bev
    
    def binarize_lanes(self, bgr):
        """
        Convert BGR to HSV and create binary mask for white lanes
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        
        # White color range
        mask_y = cv2.inRange(hsv, self.white_lower, self.white_upper)
        
        # Morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, kernel)
        
        return mask_y
    
    def run_sliding_window(self, binary_mask):
        """
        Sliding window algorithm to find lane centers
        Returns: (debug_image, left_centers, right_centers)
        """
        h, w = binary_mask.shape[:2]
        
        # Convert to BGR for visualization
        debug_img = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)
        
        # Find non-zero points
        nz_pts = cv2.findNonZero(binary_mask)
        if nz_pts is None:
            nz_pts = np.empty((0, 1, 2), dtype=np.int32)
        else:
            nz_pts = nz_pts.reshape(-1, 2)
        
        # Histogram of bottom half
        lower_half = binary_mask[h//2:, :]
        hist = np.sum(lower_half, axis=0)
        
        # Find left and right base positions
        midpoint = w // 2
        left_base = np.argmax(hist[:midpoint])
        right_base = np.argmax(hist[midpoint:]) + midpoint
        
        window_height = h // self.num_windows
        left_current = int(left_base)
        right_current = int(right_base)
        
        left_window_centers = []
        right_window_centers = []
        left_indices = []
        right_indices = []
        
        # Sliding window loop
        for win in range(self.num_windows):
            y_low = h - (win + 1) * window_height
            y_high = h - win * window_height
            
            # Draw window rectangles
            if left_current >= 0:
                cv2.rectangle(debug_img,
                             (left_current - self.window_margin, y_low),
                             (left_current + self.window_margin, y_high),
                             (255, 0, 0), 2)
            
            if right_current >= 0:
                cv2.rectangle(debug_img,
                             (right_current - self.window_margin, y_low),
                             (right_current + self.window_margin, y_high),
                             (255, 0, 0), 2)
            
            # Find good indices for left and right
            good_left = []
            good_right = []
            
            for i, (px, py) in enumerate(nz_pts):
                px, py = int(px), int(py)
                
                if y_low <= py < y_high:
                    # Left lane
                    if (left_current >= 0 and
                        left_current - self.window_margin <= px < left_current + self.window_margin):
                        good_left.append(i)
                    
                    # Right lane
                    if (right_current >= 0 and
                        right_current - self.window_margin <= px < right_current + self.window_margin):
                        good_right.append(i)
            
            # If lanes are too close, suppress one
            if (left_current >= 0 and right_current >= 0 and
                abs(left_current - right_current) < self.min_lane_sep):
                if len(good_left) < len(good_right):
                    good_left = []
                else:
                    good_right = []
            
            left_indices.extend(good_left)
            right_indices.extend(good_right)
            
            y_center = (y_low + y_high) // 2
            
            # Process left lane
            if good_left:
                x_mean = np.mean(nz_pts[good_left, 0])
                left_window_centers.append([x_mean, y_center])
                cv2.circle(debug_img, (int(x_mean), y_center), 4, (0, 0, 255), -1)
                
                # EMA update
                left_current = int(self.center_ema_alpha * left_current +
                                  (1.0 - self.center_ema_alpha) * x_mean)
            
            # Process right lane
            if good_right:
                x_mean = np.mean(nz_pts[good_right, 0])
                right_window_centers.append([x_mean, y_center])
                cv2.circle(debug_img, (int(x_mean), y_center), 4, (0, 255, 255), -1)
                
                # EMA update
                right_current = int(self.center_ema_alpha * right_current +
                                   (1.0 - self.center_ema_alpha) * x_mean)
        
        # Color detected pixels (debug)
        for idx in left_indices:
            px, py = int(nz_pts[idx, 0]), int(nz_pts[idx, 1])
            px = max(0, min(w-1, px))
            py = max(0, min(h-1, py))
            debug_img[py, px] = [0, 0, 255]
        
        for idx in right_indices:
            px, py = int(nz_pts[idx, 0]), int(nz_pts[idx, 1])
            px = max(0, min(w-1, px))
            py = max(0, min(h-1, py))
            debug_img[py, px] = [0, 255, 0]
        
        left_centers = np.array(left_window_centers, dtype=np.float32)
        right_centers = np.array(right_window_centers, dtype=np.float32)
        
        return debug_img, left_centers, right_centers
    
    def compute_center_point(self, left_centers, right_centers):
        """
        Compute lane center point from left and right lane centers
        Returns: (center_point, found_flag) where center_point is (x, y)
        """
        half_w = 0.5 * self.lane_width_px
        
        has_left = len(left_centers) > 0
        has_right = len(right_centers) > 0
        
        if has_left and has_right:
            left_mean = np.mean(left_centers, axis=0)
            right_mean = np.mean(right_centers, axis=0)
            center = 0.5 * (left_mean + right_mean)
            return center, True
        
        if has_left:
            left_mean = np.mean(left_centers, axis=0)
            center = left_mean + np.array([half_w, 0])
            return center, True
        
        if has_right:
            right_mean = np.mean(right_centers, axis=0)
            center = right_mean - np.array([half_w, 0])
            return center, True
        
        return np.array([0, 0]), False
    
    def compute_curvature(self, centers):
        """
        Compute curvature from lane center points
        Using 3-point circle method
        Returns: curvature (1/radius)
        """
        if len(centers) < 3:
            return 0.0
        
        # Use first, middle, and last points
        p1 = centers[0]
        p2 = centers[len(centers)//2]
        p3 = centers[-1]
        
        x1, y1 = p1[0], p1[1]
        x2, y2 = p2[0], p2[1]
        x3, y3 = p3[0], p3[1]
        
        # Distances
        a = math.hypot(x2 - x1, y2 - y1)
        b = math.hypot(x3 - x2, y3 - y2)
        c = math.hypot(x3 - x1, y3 - y1)
        
        # Signed area (2x)
        area2 = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)
        A = abs(area2) * 0.5
        
        if A < 1e-6 or a < 1e-6 or b < 1e-6 or c < 1e-6:
            return 0.0
        
        # Circumradius
        R = (a * b * c) / (4.0 * A)
        kappa = 1.0 / R
        
        # Add sign based on turn direction
        sign = 1.0 if area2 > 0 else -1.0
        
        return sign * kappa
    
    def image_callback(self, msg):
        """Image callback - main processing"""
        try:
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if bgr is None or bgr.size == 0:
                return
            
            h, w = bgr.shape[:2]
            cx_cam = w // 2
            cy_cam = h // 2
            
            # 1) Create ROI polygon
            roi_poly = self.make_roi_polygon(h, w)
            
            # Visualize ROI
            src_vis = bgr.copy()
            overlay = bgr.copy()
            roi_poly_int = roi_poly.astype(np.int32)
            cv2.fillPoly(overlay, [roi_poly_int], (0, 255, 0))
            cv2.addWeighted(overlay, 0.25, bgr, 0.75, 0, src_vis)
            cv2.polylines(src_vis, [roi_poly_int], True, (0, 0, 0), 2)
            
            # Camera frame center (purple point)
            cv2.circle(src_vis, (cx_cam, cy_cam), 6, (255, 0, 255), -1)
            
            # 2) Warp to BEV
            bev_bgr = self.warp_to_bev(bgr, roi_poly)
            
            # 3) Binarize (yellow lanes)
            bev_binary = self.binarize_lanes(bev_bgr)
            
            # 4) Sliding window
            debug_img, left_centers, right_centers = self.run_sliding_window(bev_binary)
            
            # 5) Compute center point
            center_pt, found = self.compute_center_point(left_centers, right_centers)
            
            # BEV 중심 x좌표 (320 = 640/2)
            bev_center_x = 320.0
            
            if found:
                # dx = center_pt[0] - bev_center_x (오른쪽이 양수)
                # steering_offset: 좌측 편향 = 음수, 우측 편향 = 양수
                dx = center_pt[0] - bev_center_x
                
                # Publish steering offset (BEV 기반)
                offset_msg = Float32()
                offset_msg.data = float(dx)
                self.pub_steering_offset.publish(offset_msg)
                
                # Draw center point
                cv2.circle(debug_img,
                          (int(center_pt[0]), int(center_pt[1])),
                          6, (255, 0, 255), -1)
            else:
                # No lane detected
                offset_msg = Float32()
                offset_msg.data = 0.0
                self.pub_steering_offset.publish(offset_msg)
            
            # 6) Compute curvature -> lane speed
            lane_pts = left_centers if len(left_centers) >= len(right_centers) else right_centers
            curvature = self.compute_curvature(lane_pts)
            
            # 곡률에 따른 속도 조절
            # 곡률이 크면 (급커브) 속도 감소
            base_speed = 0.3  # m/s
            speed_factor = max(0.1, 1.0 - abs(curvature) * 100.0)  # 곡률이 클수록 느림
            lane_speed = base_speed * speed_factor
            
            speed_msg = Float32()
            speed_msg.data = float(lane_speed)
            self.pub_lane_speed.publish(speed_msg)
            
            # Debug text
            cv2.putText(debug_img,
                       f"Left centers: {len(left_centers)}",
                       (10, 24),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                       (255, 255, 255), 2)
            cv2.putText(debug_img,
                       f"Right centers: {len(right_centers)}",
                       (10, 48),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                       (255, 255, 255), 2)
            cv2.putText(debug_img,
                       f"Curvature: {curvature:.6f}",
                       (10, 72),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                       (255, 255, 255), 2)
            
            # Publish debug image
            if self.pub_bev_image.get_num_connections() > 0:
                self.pub_bev_image.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))
            
            # Throttled log
            rospy.loginfo_throttle(1.0,
                f"[lane_bev] Left: {len(left_centers)} | Right: {len(right_centers)} | "
                f"Curvature: {curvature:.6f} | Speed: {lane_speed:.2f} m/s | dx: {dx if found else 'N/A'}")
        
        except cv2.error as e:
            rospy.logwarn(f"[lane_ver2] OpenCV error: {e}")
        except Exception as e:
            rospy.logwarn(f"[lane_ver2] Error: {e}")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = LaneCenterDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass
