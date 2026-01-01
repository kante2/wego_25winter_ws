#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Traffic Light Detection Node for WEGO
- HSV-based red/green light detection
- Subscribes to camera stream
- Publishes traffic light state (RED/GREEN/UNKNOWN)
- Does NOT publish motor commands (decision is delegated to mission module)
"""

import rospy
import cv2
import numpy as np
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from dynamic_reconfigure.server import Server

# Dynamic reconfigure import - try both possible locations
try:
    from wego.cfg import TrafficLightConfig
except ImportError:
    from wego_cfg.cfg import TrafficLightConfig


class TrafficLightDetector:
    def __init__(self):
        rospy.init_node('traffic_light_detect_node', anonymous=True)
        
        self.bridge = CvBridge()
        self.current_state = "UNKNOWN"
        
        # Load fisheye calibration
        self.camera_matrix, self.dist_coeffs = self._load_calibration()
        
        # Dynamic Reconfigure
        self.config = None
        self.srv = Server(TrafficLightConfig, self.reconfigure_callback)
        
        # Publishers
        self.state_pub = rospy.Publisher('/webot/traffic_light/state', String, queue_size=1)
        self.pub_image = rospy.Publisher('/webot/traffic_light/image', Image, queue_size=1)
        self.pub_debug = rospy.Publisher('/webot/traffic_light/debug', Image, queue_size=1)
        
        # Image subscriber
        self.image_sub = rospy.Subscriber(
            '/usb_cam/image_raw/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo("="*50)
        rospy.loginfo("Traffic Light Detection Node initialized")
        rospy.loginfo("Publishing to: /webot/traffic_light/state")
        rospy.loginfo("View debug: rqt_image_view /webot/traffic_light/image")
        rospy.loginfo("="*50)
    
    def _load_calibration(self):
        """Load fisheye camera calibration"""
        try:
            calib_file = rospy.get_param('~calibration_file',
                '/home/wego/catkin_ws/src/usb_cam/calibration/usb_cam.yaml')
            with open(calib_file, 'r') as f:
                calib = yaml.safe_load(f)
            camera_matrix = np.array(calib['camera_matrix']['data']).reshape(3, 3)
            dist_coeffs = np.array(calib['distortion_coefficients']['data'])
            rospy.loginfo("[TrafficLightDetector] Calibration loaded from %s", calib_file)
            return camera_matrix, dist_coeffs
        except Exception as e:
            rospy.logwarn("[TrafficLightDetector] Calibration load failed: %s", str(e))
            return None, None
    
    def undistort(self, img):
        """Apply fisheye undistortion"""
        if self.camera_matrix is None:
            return img
        h, w = img.shape[:2]
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.camera_matrix, self.dist_coeffs, (w, h), np.eye(3), balance=0.0
        )
        return cv2.fisheye.undistortImage(
            img, self.camera_matrix, self.dist_coeffs, Knew=new_K
        )
    
    def reconfigure_callback(self, config, level):
        self.config = config
        rospy.loginfo("[TrafficLightDetector] Config updated")
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
            return False
            
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return False
            
        circularity = 4 * np.pi * area / (perimeter ** 2)
        return circularity > self.config.circularity_thresh
    
    def image_callback(self, msg):
        if self.config is None:
            return
            
        try:
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                return
            
            # Apply fisheye undistortion
            cv_image = self.undistort(cv_image)
            
            # Apply brightness adjustment
            adjusted = cv2.convertScaleAbs(cv_image, alpha=self.config.brightness_factor, beta=0)
            
            # Extract ROI (traffic light is typically at top of image)
            roi = adjusted[self.config.roi_top:self.config.roi_bottom, 
                          self.config.roi_left:self.config.roi_right]
            
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
            red_detected = any(self.check_rectangularity(c) for c in red_contours)
            green_detected = any(self.check_rectangularity(c) for c in green_contours)
            
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
                rospy.loginfo(f"[TrafficLightDetector] State: {prev_state} -> {self.current_state}")
            
            # Publish current state
            self.state_pub.publish(String(self.current_state))
            
            # Publish debug mask image
            if self.pub_debug.get_num_connections() > 0:
                mask_image = cv2.merge([np.zeros_like(red_mask), green_mask, red_mask])
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(mask_image, "bgr8"))
            
            # Visualization image
            if self.pub_image.get_num_connections() > 0:
                debug_image = cv_image.copy()
                
                # Draw ROI
                cv2.rectangle(debug_image, 
                              (self.config.roi_left, self.config.roi_top),
                              (self.config.roi_right, self.config.roi_bottom),
                              (255, 255, 0), 2)
                
                # Draw detections
                offset_x, offset_y = self.config.roi_left, self.config.roi_top
                for cnt in red_contours:
                    if self.check_rectangularity(cnt):
                        x, y, w, h = cv2.boundingRect(cnt)
                        cv2.rectangle(debug_image, (x+offset_x, y+offset_y), 
                                      (x+w+offset_x, y+h+offset_y), (0, 0, 255), 3)
                
                for cnt in green_contours:
                    if self.check_rectangularity(cnt):
                        x, y, w, h = cv2.boundingRect(cnt)
                        cv2.rectangle(debug_image, (x+offset_x, y+offset_y), 
                                      (x+w+offset_x, y+h+offset_y), (0, 255, 0), 3)
                
                # State text
                color = (0, 0, 255) if self.current_state == "RED" else \
                        (0, 255, 0) if self.current_state == "GREEN" else (128, 128, 128)
                cv2.putText(debug_image, f"State: {self.current_state}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                
                self.pub_image.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))
                
        except Exception as e:
            rospy.logerr("[TrafficLightDetector] Error: %s", str(e))
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        detector = TrafficLightDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
