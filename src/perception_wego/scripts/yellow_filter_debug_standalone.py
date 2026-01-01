#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Yellow Lane Filter Debug (Standalone - No ROS Required)
- 저장된 이미지나 웹캠에서 노란색 필터 시각화
- BEV 변환 후 HSV 범위 조정
- 실시간 비율 계산
"""

import cv2
import numpy as np
import sys
from pathlib import Path


class YellowFilterDebugStandalone:
    def __init__(self, image_path=None, use_webcam=False):
        # ========== ROI Parameters ==========
        self.roi_top_y_ratio = 0.60
        self.roi_left_top_ratio = 0.10
        self.roi_right_top_ratio = 0.85
        self.roi_left_bot_ratio = -0.45
        self.roi_right_bot_ratio = 1.40
        
        # ========== HSV Range (Tunable) ==========
        self.h_lower = 18
        self.h_upper = 38
        self.s_lower = 100
        self.s_upper = 255
        # ========== HSV Range (Tunable) ==========
        # WHITE lane: H: 0-180, S: 0-30, V: 200-255
        self.h_lower = 0
        self.h_upper = 180
        self.s_lower = 0
        self.s_upper = 30
        self.v_lower = 200
        self.v_upper = 255
        
        # ========== Debug Windows ==========
        cv2.namedWindow('Original + ROI', cv2.WINDOW_NORMAL)
        cv2.namedWindow('BEV Image', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Yellow Mask', cv2.WINDOW_NORMAL)
        cv2.namedWindow('HSV Range Control', cv2.WINDOW_NORMAL)
        
        cv2.resizeWindow('Original + ROI', 640, 480)
        cv2.resizeWindow('BEV Image', 640, 480)
        cv2.resizeWindow('Yellow Mask', 640, 480)
        cv2.resizeWindow('HSV Range Control', 500, 300)
        
        # ========== Trackbars for HSV tuning ==========
        cv2.createTrackbar('H Lower', 'HSV Range Control', self.h_lower, 179, self.on_h_lower)
        cv2.createTrackbar('H Upper', 'HSV Range Control', self.h_upper, 179, self.on_h_upper)
        cv2.createTrackbar('S Lower', 'HSV Range Control', self.s_lower, 255, self.on_s_lower)
        cv2.createTrackbar('S Upper', 'HSV Range Control', self.s_upper, 255, self.on_s_upper)
        cv2.createTrackbar('V Lower', 'HSV Range Control', self.v_lower, 255, self.on_v_lower)
        cv2.createTrackbar('V Upper', 'HSV Range Control', self.v_upper, 255, self.on_v_upper)
        
        print("="*70)
        print("Yellow Filter Debug (Standalone)")
        print("Current HSV Range:")
        print(f"  H: [{self.h_lower:3d}, {self.h_upper:3d}]")
        print(f"  S: [{self.s_lower:3d}, {self.s_upper:3d}]")
        print(f"  V: [{self.v_lower:3d}, {self.v_upper:3d}]")
        print("Controls:")
        print("  - Adjust trackbars to find optimal HSV range")
        print("  - Press 'q' to quit")
        print("="*70)
    
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
    
    def process_image(self, bgr):
        """Process image and display debug info"""
        if bgr is None or bgr.size == 0:
            return
        
        h, w = bgr.shape[:2]
        
        # ===== 1. Draw ROI on original image =====
        roi_poly = self.make_roi_polygon(h, w)
        roi_vis = bgr.copy()
        roi_points = roi_poly.astype(np.int32)
        cv2.polylines(roi_vis, [roi_points], True, (0, 255, 0), 3)
        
        # ===== 2. Warp to BEV =====
        bev_bgr = self.warp_to_bev(bgr, roi_poly)
        
        # ===== 3. Convert to HSV =====
        hsv = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
        
        # ===== 4. Apply white filter with trackbar values =====
        lower = np.array([self.h_lower, self.s_lower, self.v_lower])
        upper = np.array([self.h_upper, self.s_upper, self.v_upper])
        mask = cv2.inRange(hsv, lower, upper)
        
        # ===== 5. Calculate white pixel ratio =====
        total_pixels = h * w
        white_pixels = cv2.countNonZero(mask)
        white_ratio = (white_pixels / total_pixels) * 100.0
        
        # ===== 6. Morphological operations =====
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask_processed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # ===== 7. Create visualization =====
        # White mask with contours
        mask_vis = cv2.cvtColor(mask_processed, cv2.COLOR_GRAY2BGR)
        contours, _ = cv2.findContours(mask_processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(mask_vis, contours, -1, (0, 255, 0), 2)
        
        # Add statistics
        stats_text = [
            f"White Pixels: {white_pixels}",
            f"Total Pixels: {total_pixels}",
            f"White Ratio: {white_ratio:.2f}%"
        ]
        
        # Add stats to images
        for img in [roi_vis, bev_bgr, mask_vis]:
            for i, text in enumerate(stats_text):
                cv2.putText(img, text, (10, 30 + i*35),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        
        # Add HSV range info
        hsv_info = [
            f"H: [{self.h_lower:3d}, {self.h_upper:3d}]",
            f"S: [{self.s_lower:3d}, {self.s_upper:3d}]",
            f"V: [{self.v_lower:3d}, {self.v_upper:3d}]"
        ]
        
        for i, text in enumerate(hsv_info):
            cv2.putText(bev_bgr, text, (10, h-80 + i*35),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
        
        # ===== 8. Display =====
        cv2.imshow('Original + ROI', roi_vis)
        cv2.imshow('BEV Image', bev_bgr)
        cv2.imshow('White Mask', mask_vis)
        
        # ===== 9. Log info =====
        print(f"\r[White Lane Filter] Ratio: {white_ratio:.2f}% |"
              f"Pixels: {yellow_pixels}/{total_pixels} | "
              f"H:[{self.h_lower}-{self.h_upper}] "
              f"S:[{self.s_lower}-{self.s_upper}] "
              f"V:[{self.v_lower}-{self.v_upper}]", end='')
    
    def run(self):
        """Main loop"""
        if self.use_webcam:
            # Use webcam
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("Error: Could not open webcam")
                return
            
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                self.process_image(frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            cap.release()
        
        elif self.image_path:
            # Load and process single image
            img = cv2.imread(str(self.image_path))
            if img is None:
                print(f"Error: Could not load image from {self.image_path}")
                return
            
            print(f"Loaded image: {self.image_path}")
            
            while True:
                self.process_image(img)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
        else:
            print("Error: No image path or webcam specified")
        
        cv2.destroyAllWindows()
    
    def print_current_hsv(self):
        """Print current HSV range"""
        print("\n")
        print("="*50)
        print("Current HSV Range:")
        print(f"  yellow_lower = np.array([{self.h_lower}, {self.s_lower}, {self.v_lower}])")
        print(f"  yellow_upper = np.array([{self.h_upper}, {self.s_upper}, {self.v_upper}])")
        print("="*50)


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Yellow Filter Debug Tool')
    parser.add_argument('--image', type=str, help='Path to image file')
    parser.add_argument('--webcam', action='store_true', help='Use webcam')
    args = parser.parse_args()
    
    if args.image:
        debugger = YellowFilterDebugStandalone(image_path=args.image, use_webcam=False)
    elif args.webcam:
        debugger = YellowFilterDebugStandalone(use_webcam=True)
    else:
        print("Usage:")
        print("  python3 yellow_filter_debug_standalone.py --image <path_to_image>")
        print("  python3 yellow_filter_debug_standalone.py --webcam")
        sys.exit(1)
    
    debugger.run()
    debugger.print_current_hsv()
