#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Obstacle Avoidance Perception Node for WEGO (Gap Finding Algorithm)
- Uses LiDAR to scan front ±30 degrees
- Divides into sectors and finds the best gap
- Publishes: obstacle/gap_angle, obstacle/gap_width, obstacle/min_distance, obstacle/debug
- Does NOT control motor (decision node does that)
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from wego.cfg import ObstacleAvoidConfig


class ObstacleAvoidPerception:
    def __init__(self):
        rospy.init_node('obstacle_avoid_perception')
        
        # Parameters (from yaml/cfg)
        self.scan_angle = rospy.get_param('~scan_angle', 30.0)
        self.safe_distance = rospy.get_param('~safe_distance', 0.5)
        self.stop_distance = rospy.get_param('~stop_distance', 0.2)
        self.num_sectors = rospy.get_param('~num_sectors', 12)
        
        # State
        self.sector_distances = [10.0] * self.num_sectors
        self.sector_angles = []
        
        # LiDAR data
        self.ranges = None
        self.angle_increment = 0
        
        # CV Bridge for visualization
        self.bridge = CvBridge()
        self.vis_size = 400
        
        # Dynamic reconfigure
        self.srv = Server(ObstacleAvoidConfig, self.reconfigure_callback)
        
        # Publishers - perception data only
        self.pub_gap_angle = rospy.Publisher('/webot/obstacle/gap_angle', Float32, queue_size=1)
        self.pub_gap_width = rospy.Publisher('/webot/obstacle/gap_width', Int32, queue_size=1)
        self.pub_min_distance = rospy.Publisher('/webot/obstacle/min_distance', Float32, queue_size=1)
        self.pub_debug_image = rospy.Publisher('/webot/obstacle/debug', Image, queue_size=1)
        
        # 색상 기반 분류 (노란색 콘 vs 검은 차량)
        self.pub_obstacle_type = rospy.Publisher('/webot/obstacle/type', Int32, queue_size=1)  # 0=none, 1=yellow_cone, 2=black_car
        self.pub_yellow_count = rospy.Publisher('/webot/obstacle/yellow_count', Int32, queue_size=1)  # 노란색 HSV 픽셀 개수 (30000 이상 = 콘 감지)
        
        # Subscribers
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        
        # 카메라 입력 (색상 기반 장애물 분류용)
        self.sub_image = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, 
                                          self.image_callback, queue_size=1)
        
        self.yellow_count = 0  # 현재 감지된 노란색 개수
        self.obstacle_type = 0  # 0=none, 1=yellow_cone, 2=black_car
        
        # Timers
        self.vis_timer = rospy.Timer(rospy.Duration(0.1), self.visualization_callback)
        
        self._init_sectors()
        
        rospy.loginfo("="*50)
        rospy.loginfo("Obstacle Avoid Perception Node initialized (Gap Finding)")
        rospy.loginfo(f"Scan: ±{self.scan_angle}°, Safe: {self.safe_distance}m")
        rospy.loginfo("Publishing:")
        rospy.loginfo("  - /webot/obstacle/gap_angle")
        rospy.loginfo("  - /webot/obstacle/gap_width")
        rospy.loginfo("  - /webot/obstacle/min_distance")
        rospy.loginfo("  - /webot/obstacle/debug")
        rospy.loginfo("="*50)
    
    def _init_sectors(self):
        """Initialize sector center angles"""
        total_angle = self.scan_angle * 2
        sector_size = total_angle / self.num_sectors
        
        self.sector_angles = []
        for i in range(self.num_sectors):
            angle = self.scan_angle - sector_size * (i + 0.5)
            self.sector_angles.append(angle)
        
        self.sector_distances = [10.0] * self.num_sectors
    
    def reconfigure_callback(self, config, level):
        self.scan_angle = config.scan_angle
        self.safe_distance = config.safe_distance
        self.stop_distance = config.stop_distance
        self.num_sectors = config.num_sectors
        self._init_sectors()
        rospy.loginfo(f"[ObstaclePerception] Config updated: scan=±{self.scan_angle}°, safe={self.safe_distance}m")
        return config
    
    def scan_callback(self, msg):
        """Process LiDAR scan data"""
        self.ranges = np.array(msg.ranges)
        self.angle_increment = msg.angle_increment
        self._calculate_sector_distances()
        self._publish_perception_data()
    
    def image_callback(self, msg):
        """Process camera image to detect yellow cone - using largest cone pixel count"""
        try:
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img is None:
                return
            
            # HSV 변환
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            # 노란색 범위 - 정확한 필터 (실제 노란색 콘만 감지)
            # H: 15-35 (정확한 노란색 범위)
            # S: 80-255 (포화도 높음 - 진한 노란색만)
            # V: 80-255 (명도 높음 - 밝은 노란색만)
            lower_yellow = np.array([15, 80, 80])
            upper_yellow = np.array([35, 255, 255])
            mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            # 형태학 연산 (노이즈 제거 - 더 강한 필터)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
            mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)
            
            # Dilate로 콘 영역 확대
            mask_yellow = cv2.dilate(mask_yellow, kernel, iterations=1)
            
            # ===== 콘별 픽셀 개수 계산 =====
            contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            cone_pixels = []  # 각 콘의 픽셀 개수
            for contour in contours:
                # 최소 면적 필터 (노이즈 제거)
                area = cv2.contourArea(contour)
                if area > 100:  # 최소 면적
                    cone_pixels.append(area)
            
            # 가장 큰 콘의 픽셀 개수 추출
            max_cone_pixel = max(cone_pixels) if cone_pixels else 0
            cone_count = len(cone_pixels)
            
            # 저장 (발행용)
            self.yellow_count = max_cone_pixel
            
            # ===== 장애물 타입 결정 =====
            # 우선순위: 가장 큰 콘이 7000px 이상 → type=1 (노란색 콘)
            #         콘 미감지 + min_distance < safe_distance → type=2 (검은 차)
            #         장애물 없음 → type=0
            if max_cone_pixel >= 7000:  # ✅ 가장 큰 콘이 7000px 이상 = 콘 감지!
                self.obstacle_type = 1  # 노란색 콘 감지됨
            elif self._get_min_front_distance() < self.safe_distance:
                self.obstacle_type = 2  # 노란색 없지만 LiDAR로 장애물 감지 = 검은 차
            else:
                self.obstacle_type = 0  # 장애물 없음
            
            rospy.loginfo_throttle(1.0, f"[ObstaclePerception] cones={cone_count} max_pixel={max_cone_pixel:.0f}px (threshold=7000) type={self.obstacle_type}")
            
        except Exception as e:
            rospy.logwarn_throttle(5, f"[ObstaclePerception] Image processing error: {e}")
    
    def _calculate_sector_distances(self):
        """Calculate minimum distance for each sector"""
        if self.ranges is None or self.angle_increment == 0:
            return
        
        total_points = len(self.ranges)
        # laser_link가 180도 회전되어 있으므로 인덱스 0이 앞쪽
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
    
    def _find_best_gap(self):
        """Find the best gap (widest open space)"""
        is_open = [d >= self.safe_distance for d in self.sector_distances]
        
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
            max_idx = np.argmax(self.sector_distances)
            return self.sector_angles[max_idx], 1
        
        best_gap = max(gaps, key=lambda g: g[1] - g[0])
        gap_start_idx, gap_end_idx = best_gap
        gap_width = gap_end_idx - gap_start_idx + 1
        
        center_idx = (gap_start_idx + gap_end_idx) / 2.0
        if center_idx == int(center_idx):
            gap_center_angle = self.sector_angles[int(center_idx)]
        else:
            low_idx = int(center_idx)
            high_idx = min(low_idx + 1, len(self.sector_angles) - 1)
            ratio = center_idx - low_idx
            gap_center_angle = (self.sector_angles[low_idx] * (1 - ratio) + 
                               self.sector_angles[high_idx] * ratio)
        
        return gap_center_angle, gap_width
    
    def _has_obstacle_in_front(self):
        """Check if there's an obstacle in front"""
        center_sectors = self.num_sectors // 3
        start = (self.num_sectors - center_sectors) // 2
        end = start + center_sectors
        front_distances = self.sector_distances[start:end]
        return min(front_distances) < self.safe_distance if front_distances else False
    
    def _get_min_front_distance(self):
        """Get minimum distance in front"""
        center_sectors = self.num_sectors // 3
        start = (self.num_sectors - center_sectors) // 2
        end = start + center_sectors
        front_distances = self.sector_distances[start:end]
        return min(front_distances) if front_distances else 10.0
    
    def _publish_perception_data(self):
        """Publish perception results"""
        gap_angle, gap_width = self._find_best_gap()
        min_dist = self._get_min_front_distance()
        
        self.pub_gap_angle.publish(Float32(gap_angle))
        self.pub_gap_width.publish(Int32(gap_width))
        self.pub_min_distance.publish(Float32(min_dist))
        
        # 색상 기반 분류 발행
        self.pub_obstacle_type.publish(Int32(self.obstacle_type))
        self.pub_yellow_count.publish(Int32(self.yellow_count))
    
    def publish_debug_image(self):
        """LiDAR sector visualization for rqt_image_view"""
        if self.pub_debug_image.get_num_connections() == 0:
            return
            
        size = self.vis_size
        img = np.zeros((size, size, 3), dtype=np.uint8)
        
        cx, cy = size // 2, size - 50
        
        # Grid circles
        for r_meters in [0.5, 1.0, 1.5, 2.0]:
            r_pixels = int(r_meters * 100)
            cv2.circle(img, (cx, cy), r_pixels, (40, 40, 40), 1)
            cv2.putText(img, f"{r_meters}m", (cx + r_pixels - 20, cy - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (80, 80, 80), 1)
        
        # Safe distance circle
        safe_r = int(self.safe_distance * 100)
        cv2.circle(img, (cx, cy), safe_r, (0, 100, 100), 2)
        
        # Stop distance circle
        stop_r = int(self.stop_distance * 100)
        cv2.circle(img, (cx, cy), stop_r, (0, 0, 100), 2)
        
        # Sectors
        for i, (angle, dist) in enumerate(zip(self.sector_angles, self.sector_distances)):
            rad = np.radians(90 - angle)
            display_dist = min(dist, 2.0)
            r = int(display_dist * 100)
            
            ex = int(cx + r * np.cos(rad))
            ey = int(cy - r * np.sin(rad))
            
            if dist >= self.safe_distance:
                color = (0, 255, 0)  # Green - open
            elif dist >= self.stop_distance:
                color = (0, 165, 255)  # Orange - caution
            else:
                color = (0, 0, 255)  # Red - stop
            
            cv2.line(img, (cx, cy), (ex, ey), color, 2)
            if dist < 2.0:
                cv2.circle(img, (ex, ey), 4, color, -1)
        
        # Best gap arrow
        gap_angle, gap_width = self._find_best_gap()
        gap_rad = np.radians(90 - gap_angle)
        gap_len = 150
        gap_x = int(cx + gap_len * np.cos(gap_rad))
        gap_y = int(cy - gap_len * np.sin(gap_rad))
        cv2.arrowedLine(img, (cx, cy), (gap_x, gap_y), (255, 255, 0), 3, tipLength=0.2)
        
        # Robot marker
        cv2.circle(img, (cx, cy), 8, (255, 255, 255), -1)
        cv2.putText(img, "R", (cx - 4, cy + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)
        
        # Info panel
        cv2.rectangle(img, (0, 0), (size, 70), (30, 30, 30), -1)
        
        cv2.putText(img, "OBSTACLE PERCEPTION", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, f"Gap Angle: {gap_angle:.1f}°", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        cv2.putText(img, f"Gap Width: {gap_width}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        min_dist = self._get_min_front_distance()
        cv2.putText(img, f"Min Front: {min_dist:.2f}m", (size - 150, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        try:
            self.pub_debug_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except Exception as e:
            rospy.logwarn_throttle(5, f"Debug image error: {e}")
    
    def visualization_callback(self, event):
        """Timer callback for visualization"""
        if self.ranges is not None:
            self.publish_debug_image()
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ObstacleAvoidPerception()
        node.run()
    except rospy.ROSInterruptException:
        pass
