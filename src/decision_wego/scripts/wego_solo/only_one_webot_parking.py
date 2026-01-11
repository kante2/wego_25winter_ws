#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lane Detection Node for WEGO
- HSV-based white lane detection
- Siding_window-based lane center finding
- Stanley control for steering
- Publishes steering/speed for other nodes to use
- Only controls motor when publish_cmd_vel=True
- Only publish debug topics when debug_view=True
"""

import rospy
import cv2 as cv
import numpy as np
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Float32, Int32, Bool, String, Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped
from dynamic_reconfigure.server import Server
from wego_cfg.cfg import LaneDetectConfig
import math
import time


class LaneFollow_2:
    def __init__(self):
        rospy.init_node('lanefollow')
        self.config = None
        self.cv_bridge = CvBridge()

        # Parameters from launch file
        self.publish_cmd_vel = rospy.get_param("~publish_cmd_vel", True)
        self.debug_view = rospy.get_param("~debug_view", True) 

        # cmd_vel publisher (only when publish_cmd_vel is True)
        if self.publish_cmd_vel:
            self.cmd_vel_pub = rospy.Publisher('/low_level/ackermann_cmd_mux/input/navigation', 
                                                AckermannDriveStamped, queue_size=1)
            # Subscribe to stop flag from traffic light
            self.sub_stop = rospy.Subscriber('/webot/traffic_stop', Bool, self.stop_callback, queue_size=1)
 
        # Subscriber - wego uses usb_cam
        self.image_sub = rospy.Subscriber(
            '/usb_cam/image_raw/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        # LiDAR obstacle detection
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.obstacle_safe_distance = rospy.get_param('~obstacle_safe_distance', 0.5)
        self.obstacle_stop_distance = rospy.get_param('~obstacle_stop_distance', 0.2)
        self.lidar_roi_angle = rospy.get_param('~lidar_roi_angle', 30.0)

        self.debug_publisher1 = rospy.Publisher('/binary_LaneFollow',Image,queue_size = 10)
        self.debug_publisher2 = rospy.Publisher('/sliding_window_debug',Image,queue_size = 10)
        self.debug_publisher3 = rospy.Publisher('/lane_follow_debug',Image,queue_size = 10)

        self.white_lower = np.array(rospy.get_param('~white_lower', [0, 0, 180]), dtype=np.uint8)
        self.white_upper = np.array(rospy.get_param('~white_upper', [180, 40, 255]), dtype=np.uint8)

        self.yellow_lower = np.array(rospy.get_param('~yellow_lower', [20, 40, 100]), dtype=np.uint8)
        self.yellow_upper = np.array(rospy.get_param('~yellow_upper',[38, 110, 255]), dtype=np.uint8)   

        # Stop flag (from traffic light or other nodes)
        self.stop_flag = False
        
        # 시작 대기 시간 (8초)
        self.start_time = rospy.Time.now()
        self.startup_delay = rospy.get_param('~startup_delay', 8.0)  # 8초 대기
        self.is_ready_to_drive = False
        
        # LiDAR data for obstacle detection
        self.ranges = None
        self.angle_increment = 0
        self.min_front_distance = 10.0  # 장애물 거리 저장용
        
        # Obstacle avoidance parameters
        self.avoidance_gain = rospy.get_param('~avoidance_gain', 0.6)
        self.num_sectors = rospy.get_param('~num_sectors', 9)
        self.yellow_detected = False
        self.obstacle_in_roi = False
        self.avoidance_angle = 0.0
        self.sector_angles = []
        self.sector_densities = []
        
        # ===== RED TRAFFIC LIGHT DETECTION =====
        self.red_detected = False
        self.red_stop_triggered = False  # 빨간불 정지 1회만 수행했는지 플래그
        # 빨간색 HSV 범위 확장 (채도, 명도 임계값 낮춤)
        self.red_lower1 = np.array([0, 70, 70], dtype=np.uint8)      # 빨간색 HSV (0-10) - 더 어두운 빨강도 포함
        self.red_upper1 = np.array([10, 255, 255], dtype=np.uint8)
        self.red_lower2 = np.array([165, 70, 70], dtype=np.uint8)    # 빨간색 HSV (165-180) - 범위 확장
        self.red_upper2 = np.array([180, 255, 255], dtype=np.uint8)
        self.red_threshold = rospy.get_param('~red_threshold', 0.005)  # 0.5%로 낮춤 (더 민감하게)
        
        # ===== PARKING PARAMETERS (from mission_parking.py) =====
        self.parking_active = False
        self.parking_state = "IDLE"
        self.parking_phase_start_time = None
        self.parking_triggered = False
        
        # Parking sequence parameters (속도 증가 - VESC 최소 동작 속도 고려)
        self.speed_slow = rospy.get_param('~parking_speed_slow', 0.35)  # 0.15 -> 0.35
        self.speed_reverse = rospy.get_param('~parking_speed_reverse', -0.35)  # -0.13 -> -0.35
        self.steering_angle_parking = rospy.get_param('~parking_steering_angle', 0.5)
        
        self.forward_time = rospy.get_param('~parking_forward_time', 1.5)
        self.reverse_right_time = rospy.get_param('~parking_reverse_right_time', 3.0)
        self.reverse_left_time = rospy.get_param('~parking_reverse_left_time', 3.0)
        self.final_forward_time = rospy.get_param('~parking_final_forward_time', 1.0)
        
        self.stop_before_sec = rospy.get_param('~parking_stop_before_sec', 1.0)
        self.stop_align_sec = rospy.get_param('~parking_stop_align_sec', 1.0)
        
        self.trigger_distance = rospy.get_param('~parking_trigger_distance', 0.5)
        self.target_marker_id = rospy.get_param('~parking_target_marker_id', 0)
        
        # ===== TURN MANEUVER PARAMETERS (좌회전/우회전 하드코딩) =====
        self.turn_active = False
        self.turn_state = "IDLE"
        self.turn_phase_start_time = None
        self.turn_triggered = False
        self.turn_direction = None  # "LEFT" or "RIGHT"
        
        self.turn_speed = rospy.get_param('~turn_speed', 0.35)
        self.turn_steering_angle = rospy.get_param('~turn_steering_angle', 0.5)
        self.turn_duration = rospy.get_param('~turn_duration', 2.0)  # 첫 번째 방향 2초 (기본값)
        self.turn_right_first_duration = rospy.get_param('~turn_right_first_duration', 1.8)  # ID 4 첫 번째(우회전) 1.8초
        self.turn_duration_second = rospy.get_param('~turn_duration_second', 2.0)  # 두 번째 방향 2초 (기본값)
        self.turn_right_second_duration = rospy.get_param('~turn_right_second_duration', 3.7)  # ID 4 두 번째(좌회전) 3.0초
        self.turn_left_marker_id = rospy.get_param('~turn_left_marker_id', 1)
        self.turn_right_marker_id = rospy.get_param('~turn_right_marker_id', 4)
        
        # Subscribe to ArUco marker info
        aruco_topic = rospy.get_param('~aruco_marker_info_topic', '/webot/aruco/marker_info')
        self.sub_aruco = rospy.Subscriber(aruco_topic, Float32MultiArray, self._aruco_callback, queue_size=1)

        # Load fisheye calibration
        self.camera_matrix, self.dist_coeffs = self._load_calibration()
 
        # Dynamic Reconfigure
        self.srv = Server(LaneDetectConfig, self.reconfigure_callback)

        # Image dimensions (will be updated from first image)
        self.img_width = 640
        self.img_height = 480

        # Publishers - steering/speed for other nodes
        self.pub_steering = rospy.Publisher('/webot/steering_offset', Float32, queue_size=1)
        self.pub_speed = rospy.Publisher('/webot/lane_speed', Float32, queue_size=1)
        self.pub_center_x = rospy.Publisher('/webot/lane_center_x', Int32, queue_size=1)

        # Image publishers
        self.pub_image = rospy.Publisher('/webot/lane_detect/image', Image, queue_size=1)
        self.pub_mask = rospy.Publisher('/webot/lane_detect/mask', Image, queue_size=1)

        self.src_points= np.float32([
            [0, 310],
            [640, 310],
            [0, 480],
            [640, 480]
        ])
        self.dst_points= np.float32([
            [0,   310],
            [640,   310],
            [225 , 480],
            [415, 480]
        ])

        self.warp_mat = cv.getPerspectiveTransform(self.src_points,self.dst_points)
        self.inv_warp_mat = cv.getPerspectiveTransform(self.dst_points,self.src_points)
        
        
        self.bgr = None
        self.warp_img_ori = None
        self.warp_img = None
        self.white_img = None
        self.filtered_img = None
        self.gaussian_sigma = 1
        self.gear = 3 # 3.이 default
        self.yaw = 0
        self.error = 0
        self.steer = 0
        
        
        rospy.loginfo("="*50)
        rospy.loginfo("lanefollow_parking node initialized")
        rospy.loginfo(f"publish_cmd_vel: {self.publish_cmd_vel}")
        rospy.loginfo("Steering topic: /webot/steering_offset")
        rospy.loginfo("Speed topic: /webot/lane_speed")
        rospy.loginfo(f"LiDAR obstacle detection: safe_distance={self.obstacle_safe_distance}m")
        rospy.loginfo(f"Parking: target_marker_id={self.target_marker_id}, trigger_dist={self.trigger_distance}m")
        rospy.loginfo(f"Turn Left: marker_id={self.turn_left_marker_id}, trigger_dist={self.trigger_distance}m, duration={self.turn_duration}s + {self.turn_duration_second}s")
        rospy.loginfo(f"Turn Right: marker_id={self.turn_right_marker_id}, trigger_dist={self.trigger_distance}m, duration={self.turn_duration}s + {self.turn_right_second_duration}s")
        rospy.loginfo("View: rqt_image_view /webot/lane_detect/image")
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
            rospy.loginfo("[LaneDetect] Calibration loaded from %s", calib_file)
            return camera_matrix, dist_coeffs
        except Exception as e:
            rospy.logwarn("[LaneDetect] Calibration load failed: %s", str(e))
            return None, None
    
    def undistort(self, img):
        """Apply fisheye undistortion"""
        if self.camera_matrix is None:
            return img
        h, w = img.shape[:2]
        new_K = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.camera_matrix, self.dist_coeffs, (w, h), np.eye(3), balance=0.0
        )
        return cv.fisheye.undistortImage(
            img, self.camera_matrix, self.dist_coeffs, Knew=new_K
        )
    
    def warpping(self,img):
        h,w = img.shape[:2]
        warp_img = cv.warpPerspective(img,self.warp_mat,(w,h))
        return warp_img
    
    def Gaussian_filter(self,img):
        filtered_img = cv.GaussianBlur(img,(0,0),self.gaussian_sigma)
        return filtered_img
    
    def white_color_filter_hsv(self,img):
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        white_hsv = cv.inRange(hsv,self.white_lower,self.white_upper)
        return white_hsv
    
    def yellow_color_filter_hsv(self, img):
        """Yellow lane detection using HSV color space"""
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self.yellow_lower, self.yellow_upper)
        
        # 노란색 픽셀 비율 계산
        yellow_pixel_count = cv.countNonZero(mask)
        total_pixels = mask.shape[0] * mask.shape[1]
        yellow_ratio = yellow_pixel_count / total_pixels if total_pixels > 0 else 0.0
        
        # 노란색이 일정 비율 이상이면 감지
        self.yellow_detected = (yellow_ratio > 0.01)  # 1% 이상
        
        # **디버그: 0.5초마다 상세 정보 출력**
        rospy.loginfo_throttle(0.5, 
            f"[YELLOW_DEBUG] Ratio: {yellow_ratio*100:.2f}% | "
            f"Detected: {self.yellow_detected} | "
            f"stop_flag: {self.stop_flag}")
        
        return mask

    def red_color_filter_hsv(self, img):
        """
        Red traffic light detection (lower region: 70%-100% vertical, left 60% horizontal)
        좌측 하단부(세로 70%~100%, 가로 좌측 60%)에서 빨간색 신호등 감지
        """
        h, w = img.shape[:2]
        # ROI 하단으로 이동: 세로 70%~100% (하단 30%), 가로 0~60% (좌측 60%)
        roi_y_start = int(h * 0.7)  # 70%부터 시작 (더 아래로)
        roi_y_end = h  # 100%까지
        roi_x_start = 0  # 좌측
        roi_x_end = int(w * 0.6)  # 60%까지
        
        lower_left_roi = img[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        
        # HSV 변환
        hsv = cv.cvtColor(lower_left_roi, cv.COLOR_BGR2HSV)
        
        # 빨간색은 HSV에서 0-10, 165-180 두 범위로 나뉨
        mask1 = cv.inRange(hsv, self.red_lower1, self.red_upper1)
        mask2 = cv.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv.bitwise_or(mask1, mask2)
        
        # 빨간색 픽셀 비율 계산
        red_pixel_count = cv.countNonZero(red_mask)
        total_pixels = red_mask.shape[0] * red_mask.shape[1]
        red_ratio = red_pixel_count / total_pixels if total_pixels > 0 else 0.0
        
        # 빨간색 감지 (임계값 이상)
        self.red_detected = (red_ratio > self.red_threshold)
        
        # 디버그 로그 - 더 자주 출력
        if self.red_detected:
            rospy.logwarn_throttle(0.3, 
                f"[RED TRAFFIC LIGHT] Ratio: {red_ratio*100:.3f}% | STOPPING!")
        else:
            rospy.loginfo_throttle(1.0, 
                f"[Red Check] Ratio: {red_ratio*100:.3f}% | ROI: {roi_y_start}:{roi_y_end}, {roi_x_start}:{roi_x_end}")
        
        return red_mask

    def roi_set(self,img):
        roi_img = img[310:480,0:640]
        return roi_img   

# callback 
    def reconfigure_callback(self, config, level):
        self.config = config
        rospy.loginfo(f"[LaneFollow] Config updated: speed={config.base_speed}, k={config.k}, yaw_k={config.yaw_k}")
        return config

    def stop_callback(self, msg):
        """Callback for stop flag from traffic light"""
        self.stop_flag = msg.data
    
    def _aruco_callback(self, msg):
        """
        Callback for ArUco marker detection
        msg.data = [id1, dist1, id2, dist2, ...]
        ID 0: 주차
        ID 1: 좌회전 (좌->우 시퀀스)
        ID 4: 우회전 (우->좌 시퀀스)
        """
        # 이미 다른 미션이 진행 중이면 무시
        if self.parking_active or self.turn_active:
            return
        
        data = list(msg.data)
        if len(data) < 2:
            return
        
        # 디버그: 감지된 모든 마커 정보 출력
        detected_markers = []
        for i in range(0, len(data) - 1, 2):
            marker_id = int(data[i])
            dist = float(data[i + 1])
            detected_markers.append(f"ID{marker_id}:{dist:.2f}m")
        
        if detected_markers:
            rospy.loginfo_throttle(2.0, f"[ArUco Debug] Detected: {', '.join(detected_markers)}")
        
        for i in range(0, len(data) - 1, 2):
            marker_id = int(data[i])
            dist = float(data[i + 1])
            
            # 주차 미션 (ID 0)
            if marker_id == self.target_marker_id and self.parking_state == "IDLE":
                rospy.loginfo_throttle(0.5,
                    f"[Parking] ArUco ID:{marker_id} dist={dist:.3f}m (trigger<={self.trigger_distance}m)")
                
                if dist <= self.trigger_distance:
                    rospy.logwarn("="*60)
                    rospy.logwarn(f"[PARKING TRIGGERED] ArUco ID {marker_id} detected at {dist:.3f}m!")
                    rospy.logwarn("Starting hardcoded parking sequence...")
                    rospy.logwarn("="*60)
                    self.parking_triggered = True
                    self.parking_active = True
                    self._set_parking_state("STOP_BEFORE")
                return
            
            # 좌회전 미션 (ID 1)
            elif marker_id == self.turn_left_marker_id and self.turn_state == "IDLE":
                # 디버그: 마커 1 감지 정보
                if dist > self.trigger_distance:
                    rospy.loginfo_throttle(1.0,
                        f"[Turn Debug] ID 1 detected at {dist:.3f}m (waiting for {self.trigger_distance}m)")
                else:
                    rospy.loginfo_throttle(0.5,
                        f"[Turn Left] ArUco ID:{marker_id} dist={dist:.3f}m (trigger<={self.trigger_distance}m)")
                
                if dist <= self.trigger_distance:
                    rospy.logwarn("="*60)
                    rospy.logwarn(f"[LEFT TURN TRIGGERED] ArUco ID {marker_id} detected at {dist:.3f}m!")
                    rospy.logwarn("Starting LEFT turn sequence (Left 2s -> Right 2s)...")
                    rospy.logwarn("="*60)
                    self.turn_triggered = True
                    self.turn_active = True
                    self.turn_direction = "LEFT"
                    self._set_turn_state("TURN_FIRST")
                return
            
            # 우회전 미션 (ID 4)
            elif marker_id == self.turn_right_marker_id and self.turn_state == "IDLE":
                # 디버그: 마커 4 감지 정보
                if dist > self.trigger_distance:
                    rospy.loginfo_throttle(1.0,
                        f"[Turn Debug] ID 4 detected at {dist:.3f}m (waiting for {self.trigger_distance}m)")
                else:
                    rospy.loginfo_throttle(0.5,
                        f"[Turn Right] ArUco ID:{marker_id} dist={dist:.3f}m (trigger<={self.trigger_distance}m)")
                
                if dist <= self.trigger_distance:
                    rospy.logwarn("="*60)
                    rospy.logwarn(f"[RIGHT TURN TRIGGERED] ArUco ID {marker_id} detected at {dist:.3f}m!")
                    rospy.logwarn(f"Starting RIGHT turn sequence (Right 1.5s -> Left 2s)...")
                    rospy.logwarn("="*60)
                    self.turn_triggered = True
                    self.turn_active = True
                    self.turn_direction = "RIGHT"
                    self._set_turn_state("TURN_FIRST")
                return
    
    def _set_parking_state(self, new_state):
        """Helper to transition parking states"""
        rospy.loginfo(f"[Parking] State: {self.parking_state} -> {new_state}")
        self.parking_state = new_state
        self.parking_phase_start_time = rospy.Time.now()
    
    def _set_turn_state(self, new_state):
        """Helper to transition turn maneuver states"""
        rospy.logwarn(f"[Turn {self.turn_direction}] State: {self.turn_state} -> {new_state}")
        self.turn_state = new_state
        self.turn_phase_start_time = rospy.Time.now()
    
    def _parking_elapsed(self):
        """Get elapsed time in current parking phase"""
        if self.parking_phase_start_time is None:
            return 0.0
        return (rospy.Time.now() - self.parking_phase_start_time).to_sec()
    
    def _turn_elapsed(self):
        """Get elapsed time in current turn phase"""
        if self.turn_phase_start_time is None:
            return 0.0
        return (rospy.Time.now() - self.turn_phase_start_time).to_sec()
    
    def execute_parking_sequence(self):
        """
        Execute hardcoded parking maneuver (from mission_parking.py)
        Returns: (speed, steering, is_done)
        """
        if not self.parking_active:
            return 0.0, 0.0, False
        
        elapsed = self._parking_elapsed()
        
        # STOP_BEFORE: Initial stop
        if self.parking_state == "STOP_BEFORE":
            if elapsed > self.stop_before_sec:
                self._set_parking_state("FORWARD_PASS")
            rospy.loginfo_throttle(0.5, f"[Parking] STOP_BEFORE {elapsed:.1f}/{self.stop_before_sec}s")
            return 0.0, 0.0, False
        
        # FORWARD_PASS: Move forward
        elif self.parking_state == "FORWARD_PASS":
            if elapsed > self.forward_time:
                self._set_parking_state("STOP_ALIGN")
            rospy.loginfo_throttle(0.5, f"[Parking] FORWARD_PASS {elapsed:.1f}/{self.forward_time}s")
            return self.speed_slow, 0.0, False
        
        # STOP_ALIGN: Stop to align
        elif self.parking_state == "STOP_ALIGN":
            if elapsed > self.stop_align_sec:
                self._set_parking_state("REVERSE_RIGHT")
            rospy.loginfo_throttle(0.5, f"[Parking] STOP_ALIGN {elapsed:.1f}/{self.stop_align_sec}s")
            return 0.0, 0.0, False
        
        # REVERSE_RIGHT: Reverse with right steering
        elif self.parking_state == "REVERSE_RIGHT":
            if elapsed >= self.reverse_right_time:
                self._set_parking_state("REVERSE_LEFT")
            rospy.loginfo_throttle(0.5, f"[Parking] REVERSE_RIGHT {elapsed:.1f}/{self.reverse_right_time}s")
            return self.speed_reverse, +self.steering_angle_parking, False
        
        # REVERSE_LEFT: Reverse with left steering
        elif self.parking_state == "REVERSE_LEFT":
            if elapsed >= self.reverse_left_time:
                self._set_parking_state("FORWARD_STRAIGHT")
            rospy.loginfo_throttle(0.5, f"[Parking] REVERSE_LEFT {elapsed:.1f}/{self.reverse_left_time}s")
            return self.speed_reverse, -self.steering_angle_parking, False
        
        # FORWARD_STRAIGHT: Final forward adjustment
        elif self.parking_state == "FORWARD_STRAIGHT":
            if elapsed > self.final_forward_time:
                self._set_parking_state("DONE")
            rospy.loginfo_throttle(0.5, f"[Parking] FORWARD_STRAIGHT {elapsed:.1f}/{self.final_forward_time}s")
            return self.speed_slow, 0.0, False
        
        # DONE: Parking complete
        elif self.parking_state == "DONE":
            rospy.logwarn("="*60)
            rospy.logwarn("[PARKING COMPLETE] Vehicle parked successfully!")
            rospy.logwarn("[SHUTDOWN] Terminating node in 1 second...")
            rospy.logwarn("="*60)
            
            # 정지 명령 발행
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'Parking_Done'
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            if self.publish_cmd_vel:
                self.cmd_vel_pub.publish(msg)
            
            # 1초 대기 후 노드 종료
            rospy.sleep(1.0)
            rospy.signal_shutdown("Parking completed successfully")
            
            return 0.0, 0.0, True
        
        # Fallback
        return 0.0, 0.0, False
    
    def execute_turn_sequence(self):
        """
        Execute hardcoded turn maneuver
        LEFT: 좌회전(2s) -> 우회전(2s)
        RIGHT: 우회전(1.5s) -> 좌회전(2s)
        Returns: (speed, steering, is_done)
        """
        if not self.turn_active:
            return 0.0, 0.0, False
        
        elapsed = self._turn_elapsed()
        
        # TURN_FIRST: 첫 번째 방향 회전
        if self.turn_state == "TURN_FIRST":
            # ID 4 (우회전)의 경우 첫 번째는 1.5초
            if self.turn_direction == "RIGHT":
                duration = self.turn_right_first_duration  # 1.5초
            else:
                duration = self.turn_duration  # 2.0초 (ID 1의 경우)
            
            if elapsed > duration:
                self._set_turn_state("TURN_SECOND")
            
            # LEFT 시퀀스: 첫 번째는 좌회전
            if self.turn_direction == "LEFT":
                steering = -self.turn_steering_angle  # 좌회전
                rospy.loginfo_throttle(0.5, 
                    f"[Turn LEFT] TURN_FIRST (Left) {elapsed:.1f}/{duration}s")
            # RIGHT 시퀀스: 첫 번째는 우회전
            else:
                steering = +self.turn_steering_angle  # 우회전
                rospy.loginfo_throttle(0.5, 
                    f"[Turn RIGHT] TURN_FIRST (Right) {elapsed:.1f}/{duration}s")
            
            return self.turn_speed, steering, False
        
        # TURN_SECOND: 두 번째 방향 회전
        elif self.turn_state == "TURN_SECOND":
            # ID 4 (우회전)의 경우 두 번째(좌회전)는 3.0초
            if self.turn_direction == "RIGHT":
                duration = self.turn_right_second_duration  # 3.0초
            else:
                duration = self.turn_duration_second  # 2.0초 (ID 1의 경우)
            
            if elapsed > duration:
                self._set_turn_state("DONE")
            
            # LEFT 시퀀스: 두 번째는 우회전
            if self.turn_direction == "LEFT":
                steering = +self.turn_steering_angle  # 우회전
                rospy.loginfo_throttle(0.5, 
                    f"[Turn LEFT] TURN_SECOND (Right) {elapsed:.1f}/{duration}s")
            # RIGHT 시퀀스: 두 번째는 좌회전
            else:
                steering = -self.turn_steering_angle  # 좌회전
                rospy.loginfo_throttle(0.5, 
                    f"[Turn RIGHT] TURN_SECOND (Left) {elapsed:.1f}/{duration}s")
            
            return self.turn_speed, steering, False
        
        # DONE: 회전 완료
        elif self.turn_state == "DONE":
            rospy.logwarn("="*60)
            rospy.logwarn(f"[TURN {self.turn_direction} COMPLETE] Resuming normal driving...")
            rospy.logwarn("="*60)
            
            # 상태 초기화 (노드는 종료하지 않음)
            self.turn_active = False
            self.turn_triggered = False
            self.turn_state = "IDLE"
            self.turn_direction = None
            
            return 0.0, 0.0, True
        
        # Fallback
        return 0.0, 0.0, False

    def scan_callback(self, msg):
        """Callback for LiDAR scan data - obstacle detection with sector analysis"""
        self.ranges = np.array(msg.ranges)
        self.angle_increment = msg.angle_increment
        
        # 기본 전방 장애물 거리 체크
        self.min_front_distance = self._get_min_front_distance()
        
        # 섹터 분석 수행
        self.sector_angles, self.sector_densities, self.avoidance_angle = \
            self._analyze_lidar_sectors(num_sectors=self.num_sectors)
        
        # 정지 판단: 노란색 감지 시에는 회피 시도, 아니면 정지
        if self.min_front_distance < self.obstacle_stop_distance:
            # 매우 가까운 경우에만 무조건 정지
            rospy.logwarn_throttle(1.0, 
                f"[Emergency Stop] Obstacle too close! Distance: {self.min_front_distance:.3f}m")
            self.stop_flag = True
        elif self.min_front_distance < self.obstacle_safe_distance:
            # 노란색이 감지되면 회피 시도, 아니면 정지
            if self.yellow_detected:
                rospy.loginfo_throttle(1.0, 
                    f"[Avoidance Mode] Yellow: YES | Obstacle: YES ({self.min_front_distance:.2f}m) | Avoiding...")
                self.stop_flag = False  # 회피 가능
            else:
                rospy.logwarn_throttle(1.0, 
                    f"[Stop Mode] Yellow: NO | Obstacle: YES ({self.min_front_distance:.2f}m) | Stopping...")
                self.stop_flag = True
        else:
            self.stop_flag = False
            # 장애물 없을 때도 5초마다 상태 출력
            if self.yellow_detected:
                rospy.loginfo_throttle(5.0, f"[Normal] Yellow: YES | Obstacle: NO | Normal driving")
            else:
                rospy.loginfo_throttle(5.0, f"[Normal] Yellow: NO | Obstacle: NO | Normal driving")
        
        # 디버그 로그
        if self.obstacle_in_roi and len(self.sector_densities) > 0:
            rospy.loginfo_throttle(2.0, 
                f"[AvoidanceVector] Best angle: {np.degrees(self.avoidance_angle):.1f}° "
                f"(Density: {self.sector_densities[np.argmin(self.sector_densities)]:.2f})")
    
    def _get_min_front_distance(self):
        """Get minimum distance in front (±lidar_roi_angle degrees)"""
        if self.ranges is None or self.angle_increment == 0:
            return 10.0
        
        scan_angle = self.lidar_roi_angle  # Use parameter from launch file
        total_points = len(self.ranges)
        center_idx = 0
        points_per_degree = 1.0 / np.degrees(self.angle_increment)
        
        # Calculate front sector (±30 degrees)
        start_idx = int(center_idx - scan_angle * points_per_degree)
        end_idx = int(center_idx + scan_angle * points_per_degree)
        
        start_idx = max(0, min(start_idx, total_points - 1))
        end_idx = max(0, min(end_idx, total_points - 1))
        
        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx
        
        sector_ranges = self.ranges[start_idx:end_idx + 1]
        
        # Filter out invalid ranges (0 or too far)
        valid = sector_ranges[(sector_ranges > 0.01) & (sector_ranges < 10.0)]
        
        return np.min(valid) if len(valid) > 0 else 10.0
    
    def _analyze_lidar_sectors(self, num_sectors=9):
        """
        LiDAR ROI를 여러 섹터로 나누어 장애물 밀도 분석
        Returns: (sector_angles, sector_densities, best_angle)
        """
        if self.ranges is None or self.angle_increment == 0:
            return [], [], 0.0
        
        # ROI 범위 계산
        total_points = len(self.ranges)
        center_index = 0  # RPLiDAR는 일반적으로 0번이 정면
        points_per_degree = 1.0 / np.degrees(self.angle_increment)
        half_roi_points = int(self.lidar_roi_angle * points_per_degree)
        
        start_idx = max(0, center_index - half_roi_points)
        end_idx = min(total_points, center_index + half_roi_points)
        
        roi_ranges = self.ranges[start_idx:end_idx]
        
        # 섹터 분할
        sector_size = len(roi_ranges) // num_sectors
        if sector_size == 0:
            return [], [], 0.0
        
        sector_angles = []
        sector_densities = []
        
        for i in range(num_sectors):
            sector_start = i * sector_size
            sector_end = (i + 1) * sector_size if i < num_sectors - 1 else len(roi_ranges)
            sector_data = roi_ranges[sector_start:sector_end]
            
            # 섹터 중심 각도 계산 (라디안)
            sector_center_idx = start_idx + (sector_start + sector_end) // 2
            sector_angle = (sector_center_idx - center_index) * self.angle_increment
            sector_angles.append(sector_angle)
            
            # 장애물 밀도 계산 (가까운 장애물일수록 높은 가중치)
            valid = sector_data[(sector_data > 0.01) & (sector_data < 10.0)]
            
            if len(valid) > 0:
                # 거리 역수로 밀도 계산 (가까울수록 큰 값)
                density = np.sum(1.0 / (valid + 0.1))  # +0.1은 0 division 방지
            else:
                density = 0.0
            
            sector_densities.append(density)
        
        # 장애물이 가장 적은(밀도가 낮은) 섹터 찾기
        if len(sector_densities) > 0:
            best_sector_idx = np.argmin(sector_densities)
            best_angle = sector_angles[best_sector_idx]
            
            # 장애물 존재 여부 판단
            min_distance = np.min(roi_ranges[(roi_ranges > 0.01) & (roi_ranges < 10.0)]) \
                           if np.any((roi_ranges > 0.01) & (roi_ranges < 10.0)) else 10.0
            self.obstacle_in_roi = (min_distance < self.obstacle_safe_distance)
        else:
            best_angle = 0.0
            self.obstacle_in_roi = False
        
        return sector_angles, sector_densities, best_angle

    def image_callback(self,msg):
        if self.config is None:
            return
        
        try:
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
            if cv_image is None:
                return
            
            self.bgr = self.undistort(cv_image)

        except Exception as e:
            rospy.logerr("[LaneFollow] Error: %s", str(e))

 
# sliding window 함수    
    def sliding_window(self,img,n_windows=15,margin = 12,minpix = 3):
        y = img.shape[0]
        x = img.shape[1]
        
        hist_area = np.copy(img[y // 2:, :])
        
        center_x = x // 2
        #기본 30px로 설정
        mask_width = self.config.masked_pixel if self.config else 30

        start_col = center_x - (mask_width // 2) 
        end_col = center_x + (mask_width // 2) + (mask_width % 2) 
        
        hist_area[:, start_col:end_col] = 0 
        
        histogram = np.sum(hist_area, axis=0)
        midpoint = int(histogram.shape[0]/2)
        leftx_current = np.argmax(histogram[:midpoint])
        
        #Fallback 오른쪽 차선 검출 안된 경우
        if sum(histogram[midpoint:]) < 15:
            rightx_current = midpoint*2
        else:
            rightx_current = np.argmax(histogram[midpoint:]) + midpoint
        
        window_height = int(y/n_windows)
        nz = img.nonzero()

        left_lane_inds = []
        right_lane_inds = []
    
        lx, ly, rx, ry = [], [], [], []

        out_img = np.dstack((img,img,img))*255

        for window in range(n_windows):
                
            win_yl = y - (window+1)*window_height
            win_yh = y - window*window_height

            win_xll = leftx_current - margin  
            win_xlh = leftx_current + margin
            win_xrl = rightx_current - margin
            win_xrh = rightx_current + margin

            cv.rectangle(out_img,(win_xll,win_yl),(win_xlh,win_yh),(0,255,0), 2) 
            cv.rectangle(out_img,(win_xrl,win_yl),(win_xrh,win_yh),(0,255,0), 2) 

            # 슬라이딩 윈도우 박스(녹색박스) 하나 안에 있는 흰색 픽셀의 x좌표를 모두 모은다.
            good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
            good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # 구한 x좌표 리스트에서 흰색점이 5개 이상인 경우에 한해 x 좌표의 평균값을 구함. -> 이 값을 슬라이딩 윈도우의 중심점으로 사용
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nz[1][good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = int(np.mean(nz[1][good_right_inds]))
            

            lx.append(leftx_current)
            ly.append((win_yl + win_yh)/2)

            rx.append(rightx_current)
            ry.append((win_yl + win_yh)/2)

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        lfit = np.polyfit(np.array(ly),np.array(lx),1)
        rfit = np.polyfit(np.array(ry),np.array(rx),1)

        out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
        out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]

        #cv.imshow("viewer", out_img)

        if self.debug_view:
            self.debug_publisher2.publish(self.cv_bridge.cv2_to_imgmsg(out_img))
        
        return lfit, rfit

    def sliding_window_right(self,img,n_windows=10,margin = 12,minpix = 5):
        y = img.shape[0]
        x =img.shape[1]
        # 1. 히스토그램을 계산할 이미지 영역 (아래쪽 절반) 복사
        hist_area = np.copy(img[y // 2:, :])
        
        # 2. 가운데 15px 영역 정의
        center_x = x // 2
        mask_width = 30
        
        # 제외할 영역의 시작과 끝 인덱스 계산
        # 정수 나눗셈 // 을 사용하여 계산
        start_col = center_x - (mask_width // 2) 
        end_col = center_x + (mask_width // 2) + (mask_width % 2) 
        
        # 3. 해당 영역의 픽셀 값을 0으로 설정 (마스킹)
        # 이미지 아래쪽 절반 (hist_area)에 적용
        hist_area[:, start_col:end_col] = 0 
        
        # 4. 마스킹된 이미지로 히스토그램 계산
        histogram = np.sum(hist_area, axis=0)
        midpoint = int(histogram.shape[0]/2)
        leftx_current = np.argmax(histogram[:midpoint])
        
        if sum(histogram[midpoint:]) < 15:
            rightx_current = midpoint*2
        else:
            rightx_current = np.argmax(histogram[midpoint:]) + midpoint
        
        window_height = int(y/n_windows)
        nz = img.nonzero()

        right_lane_inds = []
        rx, ry = [], []

        out_img = np.dstack((img,img,img))*255

        for window in range(n_windows):
            win_yl = y - (window+1)*window_height
            win_yh = y - window*window_height

            win_xrl = rightx_current - margin
            win_xrh = rightx_current + margin

            cv.rectangle(out_img,(win_xrl,win_yl),(win_xrh,win_yh),(0,255,0), 2) 

            good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&
                               (nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

            right_lane_inds.append(good_right_inds)

            if len(good_right_inds) > minpix:        
                rightx_current = int(np.mean(nz[1][good_right_inds]))

            rx.append(rightx_current)
            ry.append((win_yl + win_yh)/2)

        right_lane_inds = np.concatenate(right_lane_inds)

        rfit = np.polyfit(np.array(ry),np.array(rx),1)
        
        out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]
        cv.imshow("right_viewer", out_img)

        return rfit


# center line 계산 함수

    def cal_center_line(self, lfit, rfit):
        """
        lfit, rfit : np.polyfit으로 구한 왼쪽/오른쪽 차선의 1차 다항식 계수
                     x = a*y + b 형태 (len == 2)
        반환값:
            yaw   : 중앙 차선의 진행 방향 각도 (라디안)
            error : 차량(이미지 중앙) 기준, 차선 중앙의 x 오프셋(px)
        """

        cfit = (lfit + rfit) / 2.0  # [a, b]

        if self.filtered_img is not None:
            h, w = self.filtered_img.shape[:2]
        else:
            h, w = 160, 640

        y_eval = h * 0.75  # 이미지 높이의 3/4 지점을 계산에 사용

        
        a, b = cfit
        x_center = a * (y_eval) + b 

        #기울기 계산: a
        dx_dy = a
        yaw = np.arctan(dx_dy)  # 전방(y 방향) 기준 x의 변화량에 대한 각도

        #차량을 이미지 가로 중앙에 있다고 가정하고, 중앙선과의 오프셋 계산
        img_center_x = w / 2.0
        error =  - x_center + img_center_x  

        self.pub_center_x.publish(Int32(x_center))

        return yaw, error,x_center
    
    def cal_steering(self,yaw,error,gear=3,k=0.005,yaw_k=1.0,obstacle_distance=10.0): #각도들은 라디안, 거리는 px값, 속도는 0~1사이 스케일값 m/s
        base_speed = self.config.base_speed
        k = self.config.k
        yaw_k = self.config.yaw_k

        # 기본 Stanley 제어
        stanley_angle = yaw_k*yaw + np.arctan2(k*error,base_speed)
        
        # 노란색 + 장애물 감지 시 회피 로직 활성화
        if self.yellow_detected and self.obstacle_in_roi:
            # 회피 각도를 조향각에 반영 (가중치 적용)
            steering = (1 - self.avoidance_gain) * stanley_angle + self.avoidance_gain * self.avoidance_angle
            
            rospy.loginfo_throttle(1.0, 
                f"[Avoidance Active] Stanley: {np.degrees(stanley_angle):.1f}°, "
                f"Avoidance: {np.degrees(self.avoidance_angle):.1f}°, "
                f"Final: {np.degrees(steering):.1f}°")
        else:
            # 일반 차선 추종
            steering = stanley_angle
        
        # 조향각 제한
        steering = np.clip(steering, -0.34, 0.34)
        
        self.steer = steering

        self.pub_steering.publish(Float32(self.steer))
        self.pub_speed.publish(Float32(base_speed))       

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'LaneFollow'

        if self.stop_flag:
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
        else:
            # 장애물 거리에 따른 속도 조정
            if obstacle_distance < self.obstacle_safe_distance:
                # 0.2m ~ 0.5m 사이: 선형 감속 (0.4 -> 0.15)
                speed_ratio = (obstacle_distance - self.obstacle_stop_distance) / \
                             (self.obstacle_safe_distance - self.obstacle_stop_distance)
                speed_ratio = np.clip(speed_ratio, 0.0, 1.0)
                reduced_speed = 0.15 + (0.25 * speed_ratio)  # 0.15 ~ 0.4 m/s
                msg.drive.speed = reduced_speed
                rospy.loginfo_throttle(0.5, 
                    f"[Slow Down] Obstacle: {obstacle_distance:.2f}m | Speed: {reduced_speed:.2f} m/s")
            else:
                # 정상 속도
                msg.drive.speed = 0.4
            
            msg.drive.steering_angle = steering


        if self.publish_cmd_vel:
            self.cmd_vel_pub.publish(msg)

    def draw_lane(self,image, warp_roi,warp_ori,inv_mat, left_fit, right_fit):
            """
            image    : 원본 BGR 이미지
            warp_roi : ROI만 잘라낸 warp 이미지 (self.warp_img)
            inv_mat  : self.inv_warp_mat
            left_fit, right_fit : ROI 좌표계 기준 polyfit 결과
            """


            full_h, full_w = warp_ori.shape[:2]
            roi_h, roi_w   = warp_roi.shape[:2]

            roi_offset_y = 310

            yMax = roi_h
            ploty = np.linspace(0, yMax - 1, yMax)

            # ROI 기준 x좌표
            left_fitx  = left_fit[0] * ploty + left_fit[1] 
            right_fitx = right_fit[0] * ploty + right_fit[1] 

            ploty_full = ploty + 310  

            pts_left  = np.array([np.transpose(np.vstack([left_fitx,  ploty_full]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty_full])))])

            pts = np.hstack((pts_left, pts_right))

            # 전체 warp 크기의 빈 컬러 이미지 만들고 lane area 채우기
            color_warp = np.zeros_like(warp_ori).astype(np.uint8)  # full_h x full_w

            cv.fillPoly(color_warp, np.int32([pts]), (0, 255, 0))

            # 역원근변환으로 원본 이미지 좌표계로 되돌리고 오버레이
            newwarp = cv.warpPerspective(color_warp, inv_mat, (image.shape[1], image.shape[0]))
            result = cv.addWeighted(image, 1, newwarp, 0.3, 0)
            
            text1 = f"yaw: {self.yaw:.3f} rad ({self.steer:.1f} rad)"
            text2 = f"err: {self.error:.1f} px"

            #디버깅 텍스트 추가
            cv.putText(result, text1, (30, 40),
                   cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv.LINE_AA)
            cv.putText(result, text2, (30, 110),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv.LINE_AA)

            return result


    def main(self):
        # ===== 회전 시퀀스가 활성화되면 최우선 실행 (주차보다 우선) =====
        if self.turn_active:
            speed, steering, is_done = self.execute_turn_sequence()
            
            # 회전 명령 발행
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = f'Turn_{self.turn_direction}'
            msg.drive.speed = speed
            msg.drive.steering_angle = steering
            
            # 디버그: 회전 명령 상세 출력
            rospy.loginfo_throttle(0.3,
                f"[Turn {self.turn_direction}] State={self.turn_state} | "
                f"Speed={speed:.2f} | Steer={steering:.3f} | "
                f"Elapsed={self._turn_elapsed():.1f}s")
            
            if self.publish_cmd_vel:
                self.cmd_vel_pub.publish(msg)
            
            # 회전 완료 시 플래그 해제 (이미 execute_turn_sequence에서 처리됨)
            if is_done:
                rospy.logwarn("[Main] Turn complete, resuming normal operation...")
            
            return  # 회전 중에는 다른 제어 무시
        
        # ===== 주차 시퀀스가 활성화되면 두 번째 우선순위 =====
        if self.parking_active:
            speed, steering, is_done = self.execute_parking_sequence()
            
            # **디버그: 실제 명령값 출력**
            rospy.loginfo_throttle(0.3, 
                f"[Parking CMD] speed={speed:.3f}, steering={steering:.3f}")
            
            # 주차 명령 발행
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'Parking'
            msg.drive.speed = speed
            msg.drive.steering_angle = steering
            
            if self.publish_cmd_vel:
                self.cmd_vel_pub.publish(msg)
            
            # 주차 완료 시 플래그 해제
            if is_done:
                self.parking_active = False
                rospy.loginfo("[Main] Parking complete, resuming normal operation...")
            
            return  # 주차 중에는 차선 추종 무시
        
        # ===== 일반 차선 추종 로직 =====
        if self.bgr is None:
            return
        
        # ===== 시작 8초 대기 =====
        if not self.is_ready_to_drive:
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            if elapsed < self.startup_delay:
                rospy.loginfo_throttle(1.0, f"[Startup] Waiting... {elapsed:.1f}/{self.startup_delay}s")
                # 정지 명령 발행
                msg = AckermannDriveStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'Startup_Wait'
                msg.drive.speed = 0.0
                msg.drive.steering_angle = 0.0
                if self.publish_cmd_vel:
                    self.cmd_vel_pub.publish(msg)
                return
            else:
                self.is_ready_to_drive = True
                rospy.logwarn("="*60)
                rospy.logwarn("[STARTUP] 8 seconds delay completed! Starting to drive...")
                rospy.logwarn("="*60)
        
        self.warp_img_ori = self.warpping(self.bgr)
        self.warp_img = self.roi_set(self.warp_img_ori)

        g_filltered = self.Gaussian_filter(self.warp_img)
        
        # 흰색 차선 감지
        self.white_image = self.white_color_filter_hsv(g_filltered)
        
        # 노란색 차선 감지 추가
        yellow_image = self.yellow_color_filter_hsv(g_filltered)
        
        # 두 마스크 결합 (OR 연산)
        combined_mask = cv.bitwise_or(self.white_image, yellow_image)

        if self.debug_view:
            self.debug_publisher1.publish(self.cv_bridge.cv2_to_imgmsg(combined_mask))

        # lfit,rfit = self.sliding_window(combined_mask)
        # self.yaw,self.error,x_center = self.cal_center_line(lfit,rfit)

        rfit = self.sliding_window_right(combined_mask)
        self.yaw,self.error= self.cal_center_line_right(rfit)

        self.cal_steering(yaw=self.yaw, error=self.error, obstacle_distance=self.min_front_distance)

        #debug_img = self.draw_lane(self.bgr,self.warp_img,self.warp_img_ori,self.inv_warp_mat,lfit,rfit)
        
        if self.debug_view:
            self.debug_publisher1.publish(self.cv_bridge.cv2_to_imgmsg(self.white_image, encoding="mono8"))
            #self.debug_publisher3.publish(self.cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))
    
    def cal_center_line_right(self, rfit):
        a,b = rfit
        cfit = [a,b-120]  # 오른쪽 차선에서 중앙선으로 보정

        h, w = 170, 640

        y_eval = h * 0.75

        a, b = cfit
        x_center = a * (y_eval) + b 

        dx_dy = a
        yaw = np.arctan(dx_dy)

        img_center_x = w / 2.0
        error =  - x_center + img_center_x

        return yaw, error
    
if __name__ == '__main__':
    try:
        lf = LaneFollow_2()
        rate = rospy.Rate(30)  # 30Hz

        while not rospy.is_shutdown():
            lf.main()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass