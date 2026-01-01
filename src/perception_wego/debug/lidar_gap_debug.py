#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR Gap Finding Visualization
- Shows front LiDAR scan in polar coordinates
- Displays detected gaps
- Shows best gap for avoidance
- Real-time visualization
"""

import rospy
import numpy as np
import cv2
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32


class LiDARGapVisualizer:
    def __init__(self):
        rospy.init_node('lidar_gap_visualizer', anonymous=True)
        
        # Parameters
        self.scan_angle = rospy.get_param('~scan_angle', 30.0)
        self.safe_distance = rospy.get_param('~safe_distance', 0.5)
        self.num_sectors = rospy.get_param('~num_sectors', 12)
        self.max_range = rospy.get_param('~max_range', 3.0)
        
        # Visualization
        self.canvas_size = 800
        self.center = self.canvas_size // 2
        self.scale = (self.canvas_size // 2) / self.max_range
        
        # State
        self.ranges = None
        self.angle_increment = 0
        self.sector_distances = [10.0] * self.num_sectors
        self.sector_angles = []
        self.gap_angle = 0.0
        self.gap_width = 0
        
        # Subscribers
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.sub_gap_angle = rospy.Subscriber('/webot/obstacle/gap_angle', Float32, 
                                              self.gap_angle_callback, queue_size=1)
        self.sub_gap_width = rospy.Subscriber('/webot/obstacle/gap_width', Int32,
                                             self.gap_width_callback, queue_size=1)
        
        rospy.loginfo("="*50)
        rospy.loginfo("LiDAR Gap Visualizer Started")
        rospy.loginfo("="*50)
        rospy.loginfo("Window: 'LiDAR Scan - Gap Finding'")
        rospy.loginfo("  Blue arc: Scanning range (±30°)")
        rospy.loginfo("  Red: Obstacles (< safe_distance)")
        rospy.loginfo("  Green: Safe gaps (> safe_distance)")
        rospy.loginfo("  Yellow: Best gap for avoidance")
        rospy.loginfo("Press 'q' to quit")
        rospy.loginfo("="*50)
        
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
    
    def scan_callback(self, msg):
        """Process LiDAR scan data"""
        self.ranges = np.array(msg.ranges)
        self.angle_increment = msg.angle_increment
        self._calculate_sector_distances()
        self._visualize()
    
    def gap_angle_callback(self, msg):
        """Receive best gap angle"""
        self.gap_angle = float(msg.data)
    
    def gap_width_callback(self, msg):
        """Receive gap width"""
        self.gap_width = int(msg.data)
    
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
    
    def _visualize(self):
        """Draw visualization"""
        # Create canvas
        canvas = np.ones((self.canvas_size, self.canvas_size, 3), dtype=np.uint8) * 255
        
        # Draw grid and center
        cv2.circle(canvas, (self.center, self.center), 5, (0, 0, 0), -1)
        
        # Draw distance circles (0.5m, 1.0m, 1.5m, 2.0m)
        for dist in [0.5, 1.0, 1.5, 2.0]:
            radius = int(dist * self.scale)
            cv2.circle(canvas, (self.center, self.center), radius, (200, 200, 200), 1)
            cv2.putText(canvas, f"{dist}m", (self.center + radius + 5, self.center - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
        
        # Draw front direction
        cv2.line(canvas, (self.center, self.center), 
                (self.center, self.center - int(2.5 * self.scale)),
                (0, 0, 0), 2)
        
        # Draw scanning range arc (blue)
        for angle in range(-int(self.scan_angle), int(self.scan_angle) + 1):
            rad = np.radians(angle)
            x = self.center + int(self.max_range * self.scale * np.sin(rad))
            y = self.center - int(self.max_range * self.scale * np.cos(rad))
            cv2.circle(canvas, (x, y), 2, (255, 0, 0), -1)
        
        # Draw sectors and obstacles
        for i, angle_deg in enumerate(self.sector_angles):
            distance = self.sector_distances[i]
            
            # Clamp distance for visualization
            vis_distance = min(distance, self.max_range)
            
            rad = np.radians(angle_deg)
            x = self.center + int(vis_distance * self.scale * np.sin(rad))
            y = self.center - int(vis_distance * self.scale * np.cos(rad))
            
            # Color based on distance
            if distance < self.safe_distance:
                color = (0, 0, 255)  # Red: dangerous
            else:
                color = (0, 255, 0)  # Green: safe
            
            cv2.circle(canvas, (x, y), 6, color, -1)
        
        # Draw best gap angle (yellow)
        if self.gap_width > 0:
            gap_rad = np.radians(self.gap_angle)
            # Draw direction to gap
            gap_dist_vis = min(2.0, self.max_range)
            gap_x = self.center + int(gap_dist_vis * self.scale * np.sin(gap_rad))
            gap_y = self.center - int(gap_dist_vis * self.scale * np.cos(gap_rad))
            
            cv2.line(canvas, (self.center, self.center), (gap_x, gap_y), (0, 255, 255), 3)
            cv2.circle(canvas, (gap_x, gap_y), 8, (0, 255, 255), -1)
        
        # Draw info text
        cv2.putText(canvas, "LiDAR Scan - Gap Finding", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        
        cv2.putText(canvas, f"Gap Angle: {self.gap_angle:.1f} deg", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(canvas, f"Gap Width: {self.gap_width} sectors", (10, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Legend
        cv2.circle(canvas, (10, 130), 5, (255, 0, 0), -1)
        cv2.putText(canvas, "Red: < 0.5m (Danger)", (25, 135),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        cv2.circle(canvas, (10, 160), 5, (0, 255, 0), -1)
        cv2.putText(canvas, "Green: > 0.5m (Safe)", (25, 165),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        cv2.circle(canvas, (10, 190), 5, (0, 255, 255), -1)
        cv2.putText(canvas, "Yellow: Best Gap", (25, 195),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        # Min distance info
        if len(self.sector_distances) > 0:
            min_dist = min(self.sector_distances)
            cv2.putText(canvas, f"Min Distance: {min_dist:.2f} m", (10, self.canvas_size - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        
        # Display
        cv2.imshow("LiDAR Scan - Gap Finding", canvas)
        
        # Handle key press
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.loginfo("Quitting...")
            rospy.signal_shutdown("User quit")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = LiDARGapVisualizer()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
