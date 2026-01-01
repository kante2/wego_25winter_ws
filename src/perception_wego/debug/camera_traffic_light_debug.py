#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Traffic Light Detection Debug Node for WEGO
- HSV-based red/green light detection
- Shows detection stats and frame percentage on CV window
- Useful for tuning detection parameters
- Does NOT publish to ROS (debug only)
"""

import rospy
import cv2
import numpy as np
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from dynamic_reconfigure.server import Server

# Dynamic reconfigure import - try both possible locations
try:
    from wego.cfg import TrafficLightConfig
except ImportError:
    from wego_cfg.cfg import TrafficLightConfig


class TrafficLightDebugger:
    def __init__(self):
        rospy.init_node('traffic_light_debug_node', anonymous=True)
        
        self.bridge = CvBridge()
        self.current_state = "UNKNOWN"
        
        # Load fisheye calibration
        self.camera_matrix, self.dist_coeffs = self._load_calibration()
        
        # Dynamic Reconfigure
        self.config = None
        self.srv = Server(TrafficLightConfig, self.reconfigure_callback)
        
        # Image dimensions
        self.img_width = 640
        self.img_height = 480
        
        # Stats tracking
        self.frame_count = 0
        self.red_detection_count = 0
        self.green_detection_count = 0
        
        # Image subscriber
        self.image_sub = rospy.Subscriber(
            '/usb_cam/image_raw/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo("="*60)
        rospy.loginfo("Traffic Light Debug Node initialized")
        rospy.loginfo("Display window: traffic_light_debug")
        rospy.loginfo("Shows: Detections, ROI %, Frame %, Stats")
        rospy.loginfo("="*60)
    
    def _load_calibration(self):
        """Load fisheye camera calibration"""
        try:
            calib_file = rospy.get_param('~calibration_file',
                '/home/wego/catkin_ws/src/usb_cam/calibration/usb_cam.yaml')
            with open(calib_file, 'r') as f:
                calib = yaml.safe_load(f)
            camera_matrix = np.array(calib['camera_matrix']['data']).reshape(3, 3)
            dist_coeffs = np.array(calib['distortion_coefficients']['data'])
            rospy.loginfo("[TrafficLightDebug] Calibration loaded from %s", calib_file)
            return camera_matrix, dist_coeffs
        except Exception as e:
            rospy.logwarn("[TrafficLightDebug] Calibration load failed: %s", str(e))
            return None, None
    
    def undistort(self, img):
        """Apply fisheye undistortion"""
        if self.camera_matrix is None:
            return img
        h, w = img.shape[:2]
        try:
            new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                self.camera_matrix, self.dist_coeffs, (w, h), np.eye(3), balance=0.0
            )
            return cv2.fisheye.undistortImage(
                img, self.camera_matrix, self.dist_coeffs, Knew=new_K
            )
        except Exception as e:
            rospy.logwarn_throttle(5, f"[TrafficLightDebug] Undistort failed: {e}")
            return img
    
    def reconfigure_callback(self, config, level):
        self.config = config
        rospy.loginfo("[TrafficLightDebug] Config updated")
        return config
    
    def detect_color(self, hsv_image, lower1, upper1, lower2=None, upper2=None):
        """Detect color in HSV image, optionally with two ranges (for red)"""
        mask = cv2.inRange(hsv_image, lower1, upper1)
        if lower2 is not None and upper2 is not None:
            mask2 = cv2.inRange(hsv_image, lower2, upper2)
            mask = cv2.bitwise_or(mask, mask2)
        return mask
    
    def check_rectangularity(self, contour):
        """Check if contour is rectangular (traffic light shape)"""
        area = cv2.contourArea(contour)
        if area < self.config.min_area:
            return False, 0
            
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return False, 0
            
        circularity = 4 * np.pi * area / (perimeter ** 2)
        return circularity > self.config.circularity_thresh, area
    
    def image_callback(self, msg):
        if self.config is None:
            return
            
        try:
            self.frame_count += 1
            
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                return
            
            # Get image dimensions
            height, width = cv_image.shape[:2]
            self.img_width = width
            self.img_height = height
            frame_area = height * width
            
            # Apply fisheye undistortion
            cv_image = self.undistort(cv_image)
            
            # Apply brightness adjustment
            adjusted = cv2.convertScaleAbs(cv_image, alpha=self.config.brightness_factor, beta=0)
            
            # Extract ROI (traffic light is typically at top of image)
            roi_top = self.config.roi_top
            roi_bottom = self.config.roi_bottom
            roi_left = self.config.roi_left
            roi_right = self.config.roi_right
            roi = adjusted[roi_top:roi_bottom, roi_left:roi_right]
            roi_area = (roi_bottom - roi_top) * (roi_right - roi_left)
            roi_percent = (roi_area / frame_area) * 100 if frame_area > 0 else 0
            
            # Convert to HSV
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # Detect red (two ranges for hue wrap-around)
            red_lower1 = np.array([self.config.red_h_low1, self.config.red_s_low, self.config.red_v_low])
            red_upper1 = np.array([self.config.red_h_high1, self.config.red_s_high, self.config.red_v_high])
            red_lower2 = np.array([self.config.red_h_low2, self.config.red_s_low, self.config.red_v_low])
            red_upper2 = np.array([self.config.red_h_high2, self.config.red_s_high, self.config.red_v_high])
            red_mask = self.detect_color(hsv, red_lower1, red_upper1, red_lower2, red_upper2)
            
            # Detect green
            green_lower = np.array([self.config.green_h_low, self.config.green_s_low, self.config.green_v_low])
            green_upper = np.array([self.config.green_h_high, self.config.green_s_high, self.config.green_v_high])
            green_mask = self.detect_color(hsv, green_lower, green_upper)
            
            # Find contours
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Check for valid detections
            red_detected = False
            green_detected = False
            red_area_total = 0
            green_area_total = 0
            
            for cnt in red_contours:
                is_valid, area = self.check_rectangularity(cnt)
                if is_valid:
                    red_detected = True
                    red_area_total += area
            
            for cnt in green_contours:
                is_valid, area = self.check_rectangularity(cnt)
                if is_valid:
                    green_detected = True
                    green_area_total += area
            
            # Update stats
            if red_detected:
                self.red_detection_count += 1
            if green_detected:
                self.green_detection_count += 1
            
            # State machine
            prev_state = self.current_state
            
            if red_detected and not green_detected:
                self.current_state = "RED"
            elif green_detected and not red_detected:
                self.current_state = "GREEN"
            else:
                self.current_state = "UNKNOWN"
            
            # Log state changes
            if self.current_state != prev_state:
                rospy.loginfo(f"[TrafficLightDebug] State: {prev_state} -> {self.current_state}")
            
            # ===== Visualization =====
            debug_image = cv_image.copy()
            
            # Draw ROI rectangle
            cv2.rectangle(debug_image, 
                          (roi_left, roi_top),
                          (roi_right, roi_bottom),
                          (255, 255, 0), 2)
            
            # Draw detections with bounding boxes
            offset_x, offset_y = roi_left, roi_top
            red_color = (0, 0, 255)
            green_color = (0, 255, 0)
            
            for cnt in red_contours:
                is_valid, _ = self.check_rectangularity(cnt)
                if is_valid:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(debug_image, (x+offset_x, y+offset_y), 
                                  (x+w+offset_x, y+h+offset_y), red_color, 3)
            
            for cnt in green_contours:
                is_valid, _ = self.check_rectangularity(cnt)
                if is_valid:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(debug_image, (x+offset_x, y+offset_y), 
                                  (x+w+offset_x, y+h+offset_y), green_color, 3)
            
            # ===== Text Information =====
            y_pos = 30
            line_height = 25
            text_color = (255, 255, 255)
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 1
            
            # State
            state_color = red_color if self.current_state == "RED" else \
                          green_color if self.current_state == "GREEN" else (128, 128, 128)
            cv2.putText(debug_image, f"State: {self.current_state}", (10, y_pos),
                        font, 0.8, state_color, 2)
            y_pos += line_height + 10
            
            # Frame info
            cv2.putText(debug_image, f"Frame: {width}x{height}", (10, y_pos),
                        font, font_scale, text_color, thickness)
            y_pos += line_height
            
            # ROI percentage
            cv2.putText(debug_image, f"ROI: {roi_percent:.1f}% of frame", (10, y_pos),
                        font, font_scale, (255, 255, 0), thickness)
            y_pos += line_height
            
            # Red detection info
            red_color_text = (0, 0, 255)
            red_mask_percent = (cv2.countNonZero(red_mask) / roi_area * 100) if roi_area > 0 else 0
            cv2.putText(debug_image, f"Red: {red_detected} | Mask: {red_mask_percent:.1f}%", (10, y_pos),
                        font, font_scale, red_color_text, thickness)
            y_pos += line_height
            
            # Green detection info
            green_color_text = (0, 255, 0)
            green_mask_percent = (cv2.countNonZero(green_mask) / roi_area * 100) if roi_area > 0 else 0
            cv2.putText(debug_image, f"Green: {green_detected} | Mask: {green_mask_percent:.1f}%", (10, y_pos),
                        font, font_scale, green_color_text, thickness)
            y_pos += line_height
            
            # Statistics
            red_ratio = (self.red_detection_count / self.frame_count * 100) if self.frame_count > 0 else 0
            green_ratio = (self.green_detection_count / self.frame_count * 100) if self.frame_count > 0 else 0
            
            cv2.putText(debug_image, f"Stats | Frames: {self.frame_count}", (10, y_pos),
                        font, font_scale, text_color, thickness)
            y_pos += line_height
            
            cv2.putText(debug_image, f"Red detect rate: {red_ratio:.1f}%", (10, y_pos),
                        font, font_scale, red_color_text, thickness)
            y_pos += line_height
            
            cv2.putText(debug_image, f"Green detect rate: {green_ratio:.1f}%", (10, y_pos),
                        font, font_scale, green_color_text, thickness)
            y_pos += line_height
            
            # HSV ranges info (top right)
            y_pos_right = 30
            cv2.putText(debug_image, "=== HSV Ranges ===", (width - 250, y_pos_right),
                        font, font_scale * 0.9, (200, 200, 200), 1)
            y_pos_right += line_height
            
            cv2.putText(debug_image, f"Red H: {self.config.red_h_low1}-{self.config.red_h_high1}", 
                        (width - 250, y_pos_right), font, font_scale * 0.8, (100, 100, 255), 1)
            y_pos_right += line_height - 5
            
            cv2.putText(debug_image, f"Green H: {self.config.green_h_low}-{self.config.green_h_high}", 
                        (width - 250, y_pos_right), font, font_scale * 0.8, (100, 255, 100), 1)
            y_pos_right += line_height - 5
            
            cv2.putText(debug_image, f"S: {self.config.red_s_low}-{self.config.red_s_high}", 
                        (width - 250, y_pos_right), font, font_scale * 0.8, text_color, 1)
            y_pos_right += line_height - 5
            
            cv2.putText(debug_image, f"V: {self.config.red_v_low}-{self.config.red_v_high}", 
                        (width - 250, y_pos_right), font, font_scale * 0.8, text_color, 1)
            
            # Display window
            cv2.imshow('traffic_light_debug', debug_image)
            cv2.waitKey(1)
            
            # Log every 30 frames
            if self.frame_count % 30 == 0:
                rospy.loginfo(
                    "[TL_Debug] Frame:%d | State:%s | ROI:%.1f%% | Red:%.1f%% | Green:%.1f%%",
                    self.frame_count, self.current_state, roi_percent, 
                    red_mask_percent, green_mask_percent
                )
                
        except Exception as e:
            rospy.logerr("[TrafficLightDebug] Error: %s", str(e))
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        debugger = TrafficLightDebugger()
        debugger.run()
    except rospy.ROSInterruptException:
        pass

