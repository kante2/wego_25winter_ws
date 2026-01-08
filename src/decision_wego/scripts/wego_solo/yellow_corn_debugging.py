#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Yellow Lane + Obstacle Detection Debugging Tool
노란색 차선 + 장애물 감지 및 LiDAR 갭 찾기 디버깅 도구
"""

import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage, LaserScan
from cv_bridge import CvBridge
import math

class YellowObstacleDebugger:
    def __init__(self):
        rospy.init_node('yellow_obstacle_debugger', anonymous=True)
        
        self.cv_bridge = CvBridge()
        self.bgr = None
        self.ranges = None
        self.angle_increment = 0.0
        
        # Yellow detection parameters (넓은 범위로 수정)
        self.yellow_lower = np.array([15, 50, 80])   # H: 15~40, S: 50~255, V: 80~255
        self.yellow_upper = np.array([40, 255, 255])
        self.yellow_threshold = 0.08  # 8% (노란 콘 감지)
        self.yellow_detected = False
        
        # LiDAR parameters
        self.lidar_angle_range = 45.0  # ±45도 범위 (넓게 설정)
        self.obstacle_distance_threshold = 1.0  # 1.0m 이내 장애물
        self.gap_threshold = 0.3  # 갭으로 인정할 최소 거리 차이
        self.obstacle_detected = False
        self.best_gap_angle = 0.0
        self.best_gap_distance = 0.0
        
        # Subscribers
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, 
                        self.camera_callback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        
        rospy.loginfo("[YellowObstacleDebug] Node started")
        rospy.loginfo("[YellowObstacleDebug] Yellow HSV: [15,50,80] ~ [40,255,255]")
        rospy.loginfo("[YellowObstacleDebug] Yellow threshold: 8%")
        rospy.loginfo("[YellowObstacleDebug] LiDAR range: ±45°")
        rospy.loginfo("[YellowObstacleDebug] Obstacle threshold: 1.0m")
        rospy.loginfo("[YellowObstacleDebug] Gap vector shown ONLY when Yellow >= 8% AND Obstacle <= 1.0m")
    
    def camera_callback(self, msg):
        """Camera image callback"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv.imdecode(np_arr, cv.IMREAD_COLOR)
            if cv_image is None:
                return
            self.bgr = cv_image
        except Exception as e:
            rospy.logerr(f"[YellowObstacleDebug] Camera Error: {e}")
    
    def scan_callback(self, msg):
        """LiDAR scan callback"""
        self.ranges = np.array(msg.ranges)
        self.angle_increment = msg.angle_increment
        
        # Analyze LiDAR data for obstacles and gaps
        self.analyze_lidar_gap()
    
    def detect_yellow(self, img):
        """
        Detect yellow color in image
        Returns: (yellow_mask, yellow_ratio, yellow_detected)
        """
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        
        # Yellow mask
        yellow_mask = cv.inRange(hsv, self.yellow_lower, self.yellow_upper)
        
        # Calculate yellow pixel ratio
        yellow_pixel_count = cv.countNonZero(yellow_mask)
        total_pixels = yellow_mask.shape[0] * yellow_mask.shape[1]
        yellow_ratio = yellow_pixel_count / total_pixels if total_pixels > 0 else 0.0
        
        # Yellow detection
        yellow_detected = (yellow_ratio > self.yellow_threshold)
        
        return yellow_mask, yellow_ratio, yellow_detected
    
    def analyze_lidar_gap(self):
        """
        Analyze LiDAR data to find obstacles and best gap
        obstacle_avoid_node.py와 동일한 로직 사용
        """
        if self.ranges is None or len(self.ranges) == 0 or self.angle_increment == 0:
            self.obstacle_detected = False
            return
        
        total_points = len(self.ranges)
        # laser_link가 180도 회전되어 있으므로 인덱스 0이 앞쪽 (정면)
        center_idx = 0
        points_per_degree = 1.0 / math.degrees(self.angle_increment)
        
        # 섹터 개수 (12개로 나눔)
        num_sectors = 12
        sector_size_deg = (self.lidar_angle_range * 2) / num_sectors
        
        # 각 섹터의 최소 거리 계산
        sector_distances = []
        sector_angles = []
        
        for i in range(num_sectors):
            # 왼쪽(-45°)부터 오른쪽(+45°)으로
            start_angle = -self.lidar_angle_range + sector_size_deg * i
            end_angle = start_angle + sector_size_deg
            
            start_idx = int(center_idx + start_angle * points_per_degree)
            end_idx = int(center_idx + end_angle * points_per_degree)
            
            start_idx = max(0, min(start_idx, total_points - 1))
            end_idx = max(0, min(end_idx, total_points - 1))
            
            if start_idx > end_idx:
                start_idx, end_idx = end_idx, start_idx
            
            sector_ranges = self.ranges[start_idx:end_idx + 1]
            valid = sector_ranges[(sector_ranges > 0.01) & (sector_ranges < 10.0)]
            
            min_distance = np.min(valid) if len(valid) > 0 else 10.0
            sector_distances.append(min_distance)
            
            # 섹터 중심 각도
            center_angle = (start_angle + end_angle) / 2.0
            sector_angles.append(center_angle)
        
        # 장애물 판단 (safe_distance 이내)
        self.obstacle_detected = any(d < self.obstacle_distance_threshold for d in sector_distances)
        
        if not self.obstacle_detected:
            self.best_gap_angle = 0.0
            self.best_gap_distance = 10.0
            return
        
        # 베스트 갭 찾기 (연속된 open 섹터 중 가장 넓은 구간)
        is_open = [d >= self.obstacle_distance_threshold for d in sector_distances]
        
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
            # 갭이 없으면 가장 먼 섹터 선택
            max_idx = np.argmax(sector_distances)
            self.best_gap_angle = sector_angles[max_idx]
            self.best_gap_distance = sector_distances[max_idx]
        else:
            # 가장 넓은 갭 선택
            best_gap = max(gaps, key=lambda g: g[1] - g[0])
            gap_start_idx, gap_end_idx = best_gap
            
            # 갭 중심 각도 계산
            center_idx_float = (gap_start_idx + gap_end_idx) / 2.0
            if center_idx_float == int(center_idx_float):
                gap_center_angle = sector_angles[int(center_idx_float)]
            else:
                low_idx = int(center_idx_float)
                high_idx = min(low_idx + 1, len(sector_angles) - 1)
                ratio = center_idx_float - low_idx
                gap_center_angle = (sector_angles[low_idx] * (1 - ratio) + 
                                   sector_angles[high_idx] * ratio)
            
            self.best_gap_angle = gap_center_angle
            # 갭 구간의 평균 거리
            gap_distances = sector_distances[gap_start_idx:gap_end_idx + 1]
            self.best_gap_distance = np.mean(gap_distances)
        
        rospy.loginfo_throttle(0.5, 
            f"[LiDAR] Obstacle: {self.obstacle_detected} | "
            f"Best Gap: {self.best_gap_angle:.1f}° @ {self.best_gap_distance:.2f}m")
    
    def draw_lidar_visualization(self, img, yellow_detected=False):
        """
        Draw LiDAR visualization on image
        LiDAR 데이터를 이미지 위에 시각화
        갭 벡터는 yellow_detected=True AND obstacle_detected=True 일 때만 표시
        """
        if self.ranges is None:
            return img
        
        h, w = img.shape[:2]
        vis_img = img.copy()
        
        # LiDAR visualization parameters
        lidar_center_x = w // 2
        lidar_center_y = h - 50
        scale = 100  # pixels per meter
        
        num_points = len(self.ranges)
        center_idx = num_points // 2
        angle_range_rad = math.radians(self.lidar_angle_range)
        half_range_indices = int(angle_range_rad / self.angle_increment)
        
        start_idx = max(0, center_idx - half_range_indices)
        end_idx = min(num_points, center_idx + half_range_indices)
        
        # Draw LiDAR points
        for i in range(start_idx, end_idx):
            distance = self.ranges[i]
            if np.isinf(distance) or np.isnan(distance) or distance > 3.0:
                continue
            
            angle = (i - center_idx) * self.angle_increment
            x = int(lidar_center_x + distance * scale * math.sin(angle))
            y = int(lidar_center_y - distance * scale * math.cos(angle))
            
            # Color based on distance
            if distance < self.obstacle_distance_threshold:
                color = (0, 0, 255)  # Red for obstacles
                radius = 4
            else:
                color = (0, 255, 0)  # Green for clear
                radius = 2
            
            if 0 <= x < w and 0 <= y < h:
                cv.circle(vis_img, (x, y), radius, color, -1)
        
        # Draw car position
        cv.circle(vis_img, (lidar_center_x, lidar_center_y), 10, (255, 255, 0), -1)
        cv.circle(vis_img, (lidar_center_x, lidar_center_y), 10, (0, 0, 0), 2)
        cv.putText(vis_img, "CAR", (lidar_center_x - 20, lidar_center_y + 30),
                  cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Draw best gap direction ONLY when BOTH yellow detected AND obstacle detected
        if self.obstacle_detected and self.best_gap_distance < 3.0 and yellow_detected:
            gap_angle_rad = math.radians(self.best_gap_angle)
            gap_length = min(self.best_gap_distance * scale, 250)
            gap_end_x = int(lidar_center_x + gap_length * math.sin(gap_angle_rad))
            gap_end_y = int(lidar_center_y - gap_length * math.cos(gap_angle_rad))
            
            # Draw VERY THICK arrow pointing to best gap (마젠타 - 매우 굵게!)
            cv.arrowedLine(vis_img, (lidar_center_x, lidar_center_y), 
                          (gap_end_x, gap_end_y), (255, 0, 255), 8, tipLength=0.3)
            
            # Draw second layer for glow effect (흰색 외곽선)
            cv.arrowedLine(vis_img, (lidar_center_x, lidar_center_y), 
                          (gap_end_x, gap_end_y), (255, 255, 255), 12, tipLength=0.3)
            cv.arrowedLine(vis_img, (lidar_center_x, lidar_center_y), 
                          (gap_end_x, gap_end_y), (255, 0, 255), 8, tipLength=0.3)
            
            # Draw gap angle arc (각도 표시)
            arc_radius = 80
            start_angle = -90  # 정면 (위쪽)
            end_angle = -90 + self.best_gap_angle
            cv.ellipse(vis_img, (lidar_center_x, lidar_center_y), 
                      (arc_radius, arc_radius), 0, start_angle, end_angle, 
                      (255, 0, 255), 4)
            
            # Draw gap circle at end point (목표 지점 - 더 크게!)
            cv.circle(vis_img, (gap_end_x, gap_end_y), 20, (255, 255, 255), -1)
            cv.circle(vis_img, (gap_end_x, gap_end_y), 15, (255, 0, 255), -1)
            cv.circle(vis_img, (gap_end_x, gap_end_y), 20, (0, 0, 0), 3)
            
            # Draw gap info with background
            gap_text = f"GAP: {self.best_gap_angle:.1f}deg"
            gap_dist_text = f"{self.best_gap_distance:.2f}m"
            
            # Text background (더 크게)
            text_x = gap_end_x - 70
            text_y = gap_end_y - 40
            cv.rectangle(vis_img, (text_x - 10, text_y - 30), 
                        (text_x + 140, text_y + 20), (0, 0, 0), -1)
            cv.rectangle(vis_img, (text_x - 10, text_y - 30), 
                        (text_x + 140, text_y + 20), (255, 0, 255), 3)
            
            # Text (더 크게)
            cv.putText(vis_img, gap_text, (text_x, text_y - 5),
                      cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
            cv.putText(vis_img, gap_dist_text, (text_x, text_y + 15),
                      cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 화면 상단에 큰 메시지 표시
            cv.putText(vis_img, ">>> BEST GAP ACTIVE <<<", (w//2 - 200, 50),
                      cv.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 255), 3)
        else:
            # No gap vector (조건 불만족)
            if self.obstacle_detected and not yellow_detected:
                # 장애물만 있고 노란색 없음
                cv.putText(vis_img, "OBSTACLE ONLY (No Yellow)", 
                          (lidar_center_x - 100, lidar_center_y - 100),
                          cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 0), 2)
            elif yellow_detected and not self.obstacle_detected:
                # 노란색만 있고 장애물 없음
                cv.putText(vis_img, "YELLOW ONLY (No Obstacle)", 
                          (lidar_center_x - 100, lidar_center_y - 100),
                          cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            elif not self.obstacle_detected:
                # 둘 다 없음 - 전방 화살표
                forward_length = 150
                forward_end_x = lidar_center_x
                forward_end_y = lidar_center_y - forward_length
                cv.arrowedLine(vis_img, (lidar_center_x, lidar_center_y),
                              (forward_end_x, forward_end_y), (0, 255, 0), 4, tipLength=0.2)
                cv.putText(vis_img, "CLEAR", (forward_end_x - 30, forward_end_y - 10),
                          cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Draw ±45° FOV lines
        fov_angle_rad = math.radians(45)
        fov_length = 200
        
        # Left boundary (-45°)
        left_x = int(lidar_center_x - fov_length * math.sin(fov_angle_rad))
        left_y = int(lidar_center_y - fov_length * math.cos(fov_angle_rad))
        cv.line(vis_img, (lidar_center_x, lidar_center_y), (left_x, left_y), 
                (255, 255, 0), 2)
        
        # Right boundary (+45°)
        right_x = int(lidar_center_x + fov_length * math.sin(fov_angle_rad))
        right_y = int(lidar_center_y - fov_length * math.cos(fov_angle_rad))
        cv.line(vis_img, (lidar_center_x, lidar_center_y), (right_x, right_y), 
                (255, 255, 0), 2)
        
        return vis_img
    
    def draw_debug_info(self, img, yellow_mask, yellow_ratio):
        """Draw all debug information"""
        h, w = img.shape[:2]
        debug_img = img.copy()
        
        # Yellow mask overlay
        yellow_overlay = cv.cvtColor(yellow_mask, cv.COLOR_GRAY2BGR)
        yellow_overlay[:, :, 0] = 0  # B
        yellow_overlay[:, :, 2] = 0  # R
        # G channel keeps mask values
        
        # Alpha blending for yellow
        cv.addWeighted(debug_img, 0.7, yellow_overlay, 0.3, 0, debug_img)
        
        # Add LiDAR visualization (yellow_detected 전달)
        debug_img = self.draw_lidar_visualization(debug_img, self.yellow_detected)
        
        # Info panel background
        panel_height = 200
        cv.rectangle(debug_img, (10, 10), (w - 10, panel_height), (0, 0, 0), -1)
        cv.rectangle(debug_img, (10, 10), (w - 10, panel_height), (255, 255, 255), 2)
        
        # Yellow detection info
        yellow_text = f"Yellow Ratio: {yellow_ratio*100:.2f}%"
        yellow_status = "DETECTED!" if self.yellow_detected else "Clear"
        yellow_color = (0, 255, 255) if self.yellow_detected else (100, 100, 100)
        
        cv.putText(debug_img, yellow_text, (20, 40),
                  cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv.putText(debug_img, f"Status: {yellow_status}", (20, 70),
                  cv.FONT_HERSHEY_SIMPLEX, 0.7, yellow_color, 2)
        
        # Obstacle detection info
        obstacle_status = "OBSTACLE!" if self.obstacle_detected else "Clear"
        obstacle_color = (0, 0, 255) if self.obstacle_detected else (0, 255, 0)
        
        cv.putText(debug_img, f"Obstacle: {obstacle_status}", (20, 100),
                  cv.FONT_HERSHEY_SIMPLEX, 0.7, obstacle_color, 2)
        
        # Best Gap Vector info (두 조건 모두 만족할 때만)
        if self.obstacle_detected and self.yellow_detected:
            gap_text = f"Best Gap: {self.best_gap_angle:.1f}deg @ {self.best_gap_distance:.2f}m"
            cv.putText(debug_img, gap_text, (20, 130),
                      cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            
            # Gap vector 강조 표시
            vector_text = f">>> GAP VECTOR: {self.best_gap_angle:+.1f}deg <<<"
            cv.rectangle(debug_img, (15, 145), (w - 15, 175), (255, 0, 255), 2)
            cv.putText(debug_img, vector_text, (25, 168),
                      cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
        elif self.obstacle_detected:
            # 장애물만 있고 노란색 없음
            no_gap_text = "Best Gap: Waiting for Yellow >= 8%"
            cv.putText(debug_img, no_gap_text, (20, 130),
                      cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 0), 2)
        else:
            # 장애물 없을 때
            no_gap_text = "Best Gap: None (No obstacles)"
            cv.putText(debug_img, no_gap_text, (20, 130),
                      cv.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
        
        # Combined status (YELLOW + OBSTACLE) - 회피 모드 활성화!
        if self.yellow_detected and self.obstacle_detected:
            cv.rectangle(debug_img, (10, panel_height - 50), (w - 10, panel_height - 10),
                        (0, 0, 255), -1)
            cv.putText(debug_img, "AVOIDANCE MODE ACTIVE!", (20, panel_height - 20),
                      cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        return debug_img
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(30)  # 30 Hz
        
        rospy.loginfo("[YellowObstacleDebug] Waiting for camera and LiDAR data...")
        rospy.loginfo("[YellowObstacleDebug] Topics: /usb_cam/image_raw/compressed, /scan")
        rospy.loginfo("[YellowObstacleDebug] Press 'q' in window to quit")
        
        data_received = False
        
        while not rospy.is_shutdown():
            if self.bgr is not None:
                if not data_received:
                    rospy.loginfo("[YellowObstacleDebug] Camera data received! Window opening...")
                    data_received = True
                
                # Detect yellow
                yellow_mask, yellow_ratio, yellow_detected = self.detect_yellow(self.bgr)
                self.yellow_detected = yellow_detected
                
                # Draw debug visualization
                debug_img = self.draw_debug_info(self.bgr, yellow_mask, yellow_ratio)
                
                # Display windows
                cv.imshow("Yellow + Obstacle Debug", debug_img)
                cv.imshow("Yellow Mask", yellow_mask)
                
                # Key handling
                key = cv.waitKey(1) & 0xFF
                if key == ord('q'):
                    rospy.loginfo("[YellowObstacleDebug] Quit requested")
                    break
            else:
                # Show waiting message every 2 seconds
                rospy.loginfo_throttle(2.0, "[YellowObstacleDebug] Still waiting for camera data...")
                cv.waitKey(1)  # Keep OpenCV responsive
            
            rate.sleep()
        
        cv.destroyAllWindows()

if __name__ == '__main__':
    try:
        debugger = YellowObstacleDebugger()
        debugger.run()
    except rospy.ROSInterruptException:
        pass
