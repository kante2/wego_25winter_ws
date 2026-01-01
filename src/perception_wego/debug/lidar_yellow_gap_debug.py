#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Yellow Cone Based Gap Finding Visualization
- Detects yellow cone in camera image
- Finds gap direction based on yellow cone position
- Shows visualization with cone position and gap
- Real-time LiDAR + camera fusion
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge
import math


class YellowConeGapFinder:
    def __init__(self):
        rospy.init_node('yellow_cone_gap_finder', anonymous=True)
        
        # Parameters
        self.scan_angle = rospy.get_param('~scan_angle', 30.0)
        self.safe_distance = rospy.get_param('~safe_distance', 0.5)
        self.num_sectors = rospy.get_param('~num_sectors', 12)
        self.max_range = rospy.get_param('~max_range', 3.0)
        
        # HSV for yellow cone detection
        self.yellow_lower = np.array([15, 80, 80])
        self.yellow_upper = np.array([35, 255, 255])
        
        # Visualization
        self.canvas_size = 800
        self.center = self.canvas_size // 2
        self.scale = (self.canvas_size // 2) / self.max_range
        
        # State
        self.ranges = None
        self.angle_increment = 0
        self.sector_distances = [10.0] * self.num_sectors
        self.sector_angles = []
        
        # Yellow cone info
        self.cone_center_x = None
        self.cone_detected = False
        self.cone_image = None
        
        # LiDAR data
        self.last_scan = None
        
        # Bridge for image processing
        self.bridge = CvBridge()
        
        # Subscribers
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.sub_image = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage,
                                         self.image_callback, queue_size=1)
        
        # Publishers
        self.pub_cone_gap_angle = rospy.Publisher('/webot/yellow_cone/gap_angle', Float32, queue_size=1)
        self.pub_cone_gap_width = rospy.Publisher('/webot/yellow_cone/gap_width', Int32, queue_size=1)
        
        rospy.loginfo("="*60)
        rospy.loginfo("Yellow Cone Gap Finder Started")
        rospy.loginfo("="*60)
        rospy.loginfo("Fusion mode: Camera (Yellow Detection) + LiDAR (Gap Finding)")
        rospy.loginfo("  - Detects yellow cone in camera image")
        rospy.loginfo("  - Finds gap in LiDAR based on cone position")
        rospy.loginfo("  - Publishes: /webot/yellow_cone/gap_angle")
        rospy.loginfo("  - Publishes: /webot/yellow_cone/gap_width")
        rospy.loginfo("Press 'q' to quit")
        rospy.loginfo("="*60)
        
        self._init_sectors()
    
    def _init_sectors(self):
        """Initialize sector center angles"""
        total_angle = self.scan_angle * 2
        sector_size = total_angle / self.num_sectors
        
        self.sector_angles = []
        for i in range(self.num_sectors):
            angle = self.scan_angle - sector_size * (i + 0.5)
            self.sector_angles.append(angle)
        
        self.sector_distances = [10.0] * self.num_sectors
    
    def image_callback(self, msg):
        """Process camera image to detect yellow cone"""
        try:
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img is None:
                self.cone_detected = False
                return
            
            # Convert to HSV
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            # Create yellow mask
            mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
            
            # Morphology
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.dilate(mask, kernel, iterations=1)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Filter by area and find largest cone
            valid_contours = [c for c in contours if cv2.contourArea(c) > 500]
            
            if len(valid_contours) > 0:
                # Get largest contour
                largest = max(valid_contours, key=cv2.contourArea)
                M = cv2.moments(largest)
                
                if M["m00"] != 0:
                    # Cone center in image
                    self.cone_center_x = int(M["m10"] / M["m00"])
                    self.cone_detected = True
                    self.cone_image = img.copy()
                else:
                    self.cone_detected = False
            else:
                self.cone_detected = False
                
        except Exception as e:
            rospy.logwarn(f"Image processing error: {e}")
            self.cone_detected = False
    
    def scan_callback(self, msg):
        """Process LiDAR scan data"""
        self.ranges = np.array(msg.ranges)
        self.angle_increment = msg.angle_increment
        self.last_scan = msg
        self._calculate_sector_distances()
        self._find_gap_from_cone()
        self._visualize()
    
    def _calculate_sector_distances(self):
        """Calculate minimum distance for each sector"""
        if self.ranges is None or self.angle_increment == 0:
            return
        
        total_points = len(self.ranges)
        center_idx = 0
        points_per_degree = 1.0 / np.degrees(self.angle_increment)
        sector_size_deg = (self.scan_angle * 2) / self.num_sectors
        
        for i in range(self.num_sectors):
            start_angle = -self.scan_angle + sector_size_deg * i
            end_angle = start_angle + sector_size_deg
            
            start_idx = int(center_idx + start_angle * points_per_degree)
            end_idx = int(center_idx + end_angle * points_per_degree)
            
            start_idx = max(0, min(start_idx, total_points - 1))
            end_idx = max(0, min(end_idx, total_points - 1))
            
            if start_idx > end_idx:
                start_idx, end_idx = end_idx, start_idx
            
            sector_ranges = self.ranges[start_idx:end_idx + 1]
            valid = sector_ranges[(sector_ranges > 0.01) & (sector_ranges < 10.0)]
            
            self.sector_distances[i] = np.min(valid) if len(valid) > 0 else 10.0
    
    def _find_gap_from_cone(self):
        """Find gap based on yellow cone position"""
        if not self.cone_detected or self.cone_image is None:
            rospy.loginfo_throttle(1.0, "[YellowConeGap] Cone not detected - no gap")
            self.pub_cone_gap_angle.publish(Float32(0.0))
            self.pub_cone_gap_width.publish(Int32(0))
            return
        
        # Map image x-coordinate to camera angle
        # Assume 640 pixel width = 60 degree FOV (typical USB camera)
        img_width = self.cone_image.shape[1]
        pixel_per_degree = img_width / 60.0
        
        # Cone angle relative to image center
        cone_angle_img = (self.cone_center_x - img_width / 2.0) / pixel_per_degree
        
        # Limit to scan range
        cone_angle = np.clip(cone_angle_img, -self.scan_angle, self.scan_angle)
        
        rospy.loginfo_throttle(1.0, 
            f"[YellowConeGap] Cone detected at pixel {self.cone_center_x}, angle {cone_angle:.1f}°")
        
        # Find safe gap around cone
        gap_angle, gap_width = self._find_best_gap_around_cone(cone_angle)
        
        # Publish results
        self.pub_cone_gap_angle.publish(Float32(gap_angle))
        self.pub_cone_gap_width.publish(Int32(gap_width))
        
        rospy.loginfo_throttle(1.0,
            f"[YellowConeGap] Best gap: angle={gap_angle:.1f}°, width={gap_width} sectors")
    
    def _find_best_gap_around_cone(self, cone_angle):
        """Find best gap around detected cone"""
        # Find which sector cone is in
        cone_sector_idx = None
        min_angle_diff = float('inf')
        
        for i, sector_angle in enumerate(self.sector_angles):
            angle_diff = abs(sector_angle - cone_angle)
            if angle_diff < min_angle_diff:
                min_angle_diff = angle_diff
                cone_sector_idx = i
        
        if cone_sector_idx is None:
            return 0.0, 0
        
        # Mark dangerous sectors around cone
        cone_range = self.sector_distances[cone_sector_idx]
        dangerous_threshold = self.safe_distance + 0.2  # slightly larger buffer
        
        is_open = []
        for i, dist in enumerate(self.sector_distances):
            # Sector is open if distance > safe_distance AND far from cone
            far_from_cone = abs(i - cone_sector_idx) >= 2  # at least 2 sectors away
            is_open.append((dist >= self.safe_distance) and far_from_cone)
        
        # Find largest gap
        gaps = []
        gap_start = None
        
        for i, open_sector in enumerate(is_open):
            if open_sector and gap_start is None:
                gap_start = i
            elif not open_sector and gap_start is not None:
                gaps.append((gap_start, i - 1))
                gap_start = None
        
        if gap_start is not None:
            gaps.append((gap_start, len(is_open) - 1))
        
        if not gaps:
            # No gap found, return safest direction (away from cone)
            if cone_sector_idx < self.num_sectors // 2:
                # Cone on right, go left
                return self.sector_angles[-1], 1
            else:
                # Cone on left, go right
                return self.sector_angles[0], 1
        
        # Find best (widest) gap
        best_gap = max(gaps, key=lambda g: g[1] - g[0])
        gap_start_idx, gap_end_idx = best_gap
        gap_width = gap_end_idx - gap_start_idx + 1
        
        # Get center angle of gap
        center_idx = (gap_start_idx + gap_end_idx) / 2.0
        if center_idx == int(center_idx):
            gap_angle = self.sector_angles[int(center_idx)]
        else:
            low_idx = int(center_idx)
            high_idx = min(low_idx + 1, len(self.sector_angles) - 1)
            ratio = center_idx - low_idx
            gap_angle = (self.sector_angles[low_idx] * (1 - ratio) + 
                        self.sector_angles[high_idx] * ratio)
        
        return gap_angle, gap_width
    
    def _visualize(self):
        """Draw combined visualization"""
        # Create LiDAR canvas
        canvas = np.ones((self.canvas_size, self.canvas_size, 3), dtype=np.uint8) * 255
        
        # Draw grid
        cv2.circle(canvas, (self.center, self.center), 5, (0, 0, 0), -1)
        
        for dist in [0.5, 1.0, 1.5, 2.0]:
            radius = int(dist * self.scale)
            cv2.circle(canvas, (self.center, self.center), radius, (200, 200, 200), 1)
            cv2.putText(canvas, f"{dist}m", (self.center + radius + 5, self.center - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
        
        # Draw front
        cv2.line(canvas, (self.center, self.center), 
                (self.center, self.center - int(2.5 * self.scale)),
                (0, 0, 0), 2)
        
        # Draw scanning range
        for angle in range(-int(self.scan_angle), int(self.scan_angle) + 1):
            rad = np.radians(angle)
            x = self.center + int(self.max_range * self.scale * np.sin(rad))
            y = self.center - int(self.max_range * self.scale * np.cos(rad))
            cv2.circle(canvas, (x, y), 2, (255, 0, 0), -1)
        
        # Draw sectors
        for i, angle_deg in enumerate(self.sector_angles):
            distance = self.sector_distances[i]
            vis_distance = min(distance, self.max_range)
            
            rad = np.radians(angle_deg)
            x = self.center + int(vis_distance * self.scale * np.sin(rad))
            y = self.center - int(vis_distance * self.scale * np.cos(rad))
            
            if distance < self.safe_distance:
                color = (0, 0, 255)  # Red: dangerous
            else:
                color = (0, 255, 0)  # Green: safe
            
            cv2.circle(canvas, (x, y), 6, color, -1)
        
        # Draw title
        cv2.putText(canvas, "Yellow Cone Based Gap Finding", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        
        # Cone status
        if self.cone_detected:
            status = f"Yellow Cone: DETECTED (x={self.cone_center_x})"
            color = (0, 255, 255)
        else:
            status = "Yellow Cone: NOT DETECTED"
            color = (0, 0, 255)
        
        cv2.putText(canvas, status, (10, 65),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Legend
        cv2.circle(canvas, (10, 100), 5, (255, 0, 0), -1)
        cv2.putText(canvas, "Red: < 0.5m (Danger)", (25, 105),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        cv2.circle(canvas, (10, 130), 5, (0, 255, 0), -1)
        cv2.putText(canvas, "Green: > 0.5m (Safe)", (25, 135),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        cv2.circle(canvas, (10, 160), 5, (0, 255, 255), -1)
        cv2.putText(canvas, "Yellow: Best Gap Direction", (25, 165),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        # Min distance
        if len(self.sector_distances) > 0:
            min_dist = min(self.sector_distances)
            cv2.putText(canvas, f"Min Distance: {min_dist:.2f} m", (10, self.canvas_size - 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        
        # Display
        cv2.imshow("Yellow Cone Gap Finder - LiDAR", canvas)
        
        # Handle key
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown("User quit")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = YellowConeGapFinder()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
