#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32, Int32


class ObstacleAvoidMission:
    """
    - 입력:
      /webot/obstacle/gap_angle, /gap_width, /min_distance
      + lane fallback: /webot/steering_offset, /webot/lane_speed
    - 출력: step()에서 (speed, steer, debug) 반환
    """

    def __init__(self):
        # params
        self.safe_distance = 0.5      # 회피 시작 거리 (50cm)
        self.stop_distance = 0.05     # 응급 정지 거리 (5cm) ← 0.2m에서 0.05m로 변경
        self.avoid_speed = 0.3        # 기본 회피 속도 (0.2에서 0.3으로 상향)
        self.max_steering = 0.8       # 최대 조향각 (0.5에서 0.8로 상향)
        self.steering_gain = 0.05     # 기본 조향 게인 (0.02에서 0.05로 상향)
        self.clear_threshold = 20     # 안전 판정 사이클

        # perception
        self.gap_angle = 0.0
        self.gap_width = 1
        self.min_distance = 10.0
        
        # 색상 기반 분류
        self.obstacle_type = 0  # 0=none, 1=yellow_cone, 2=black_car
        self.yellow_count = 0   # 노란색 개수

        # lane fallback
        self.lane_steering = 0.0
        self.lane_speed = 0.3

        # state
        self.avoiding = False
        self.clear_count = 0

    def init_from_params(self, ns="~obstacle"):
        self.safe_distance = float(rospy.get_param(f"{ns}/safe_distance", 0.5))
        self.stop_distance = float(rospy.get_param(f"{ns}/stop_distance", 0.2))
        self.avoid_speed = float(rospy.get_param(f"{ns}/avoid_speed", 0.2))
        self.max_steering = float(rospy.get_param(f"{ns}/max_steering", 0.5))
        self.steering_gain = float(rospy.get_param(f"{ns}/steering_gain", 0.02))
        self.clear_threshold = int(rospy.get_param(f"{ns}/clear_threshold", 20))

        # topics
        t_gap_angle = rospy.get_param(f"{ns}/gap_angle_topic", "/webot/obstacle/gap_angle")
        t_gap_width = rospy.get_param(f"{ns}/gap_width_topic", "/webot/obstacle/gap_width")
        t_min_dist = rospy.get_param(f"{ns}/min_distance_topic", "/webot/obstacle/min_distance")

        t_lane_steer = rospy.get_param(f"{ns}/lane_steering_topic", "/webot/steering_offset")
        t_lane_speed = rospy.get_param(f"{ns}/lane_speed_topic", "/webot/lane_speed")
        
        # 색상 기반 분류 토픽
        t_obstacle_type = rospy.get_param(f"{ns}/obstacle_type_topic", "/webot/obstacle/type")
        t_yellow_count = rospy.get_param(f"{ns}/yellow_count_topic", "/webot/obstacle/yellow_count")

        rospy.Subscriber(t_gap_angle, Float32, self._cb_gap_angle, queue_size=1)
        rospy.Subscriber(t_gap_width, Int32, self._cb_gap_width, queue_size=1)
        rospy.Subscriber(t_min_dist, Float32, self._cb_min_dist, queue_size=1)
        rospy.Subscriber(t_lane_steer, Float32, self._cb_lane_steer, queue_size=1)
        rospy.Subscriber(t_lane_speed, Float32, self._cb_lane_speed, queue_size=1)
        rospy.Subscriber(t_obstacle_type, Int32, self._cb_obstacle_type, queue_size=1)
        rospy.Subscriber(t_yellow_count, Int32, self._cb_yellow_count, queue_size=1)

        rospy.loginfo("[mission_obstacle] sub: %s %s %s type=%s", t_gap_angle, t_gap_width, t_min_dist, t_obstacle_type)

    def _cb_gap_angle(self, msg: Float32):
        self.gap_angle = float(msg.data)

    def _cb_gap_width(self, msg: Int32):
        self.gap_width = int(msg.data)

    def _cb_min_dist(self, msg: Float32):
        self.min_distance = float(msg.data)

    def _cb_lane_steer(self, msg: Float32):
        self.lane_steering = float(msg.data)

    def _cb_lane_speed(self, msg: Float32):
        self.lane_speed = float(msg.data)
    
    def _cb_obstacle_type(self, msg: Int32):
        self.obstacle_type = int(msg.data)
    
    def _cb_yellow_count(self, msg: Int32):
        self.yellow_count = int(msg.data)

    def on_enter(self):
        rospy.loginfo("[mission_obstacle] enter")

    def on_exit(self):
        rospy.loginfo("[mission_obstacle] exit")
        self.avoiding = False
        self.clear_count = 0

    def step(self):
        state = "IDLE"

        # ========================================
        # 검은 차량 (gap_angle 사용 X)
        # ========================================
        if self.obstacle_type == 2:  # 검은 차량 감지
            
            # 상황 1: 5cm 이내 → 완전 정지
            if self.min_distance < self.stop_distance:
                self.avoiding = True
                self.clear_count = 0
                return 0.0, 0.0, f"BLACK_CAR_STOP min={self.min_distance:.3f}m"
            
            # 상황 2: 5cm~50cm → 저속 직진만 (회피 X)
            if self.min_distance < self.safe_distance:
                self.avoiding = True
                self.clear_count = 0
                return 0.05, 0.0, f"BLACK_CAR_APPROACH min={self.min_distance:.3f}m"
            
            # 상황 3: 50cm 이상 & 회피 중 → 복귀 준비
            if self.avoiding:
                self.clear_count += 1
                if self.clear_count >= self.clear_threshold:
                    self.avoiding = False
                    self.clear_count = 0
                    return self.lane_speed, self.lane_steering, "BLACK_CAR_CLEAR_RETURN"
                else:
                    return 0.05, 0.0, f"BLACK_CAR_CLEAR_WAIT {self.clear_count}/{self.clear_threshold}"
        
        # ========================================
        # 노란색 콘 (gap_angle 사용 O)
        # ========================================
        elif self.obstacle_type == 1:  # 노란색 콘 감지
            
            # 상황 1: 5cm 이내 → 완전 정지
            if self.min_distance < self.stop_distance:
                self.avoiding = True
                self.clear_count = 0
                return 0.0, 0.0, f"YELLOW_CONE_EMERGENCY min={self.min_distance:.3f}m"
            
            # 상황 2: 5cm~50cm → 하드하게 회피
            if self.min_distance < self.safe_distance:
                self.avoiding = True
                self.clear_count = 0

                proximity_ratio = 1.0 - (self.min_distance / self.safe_distance)
                dynamic_gain = self.steering_gain * (1.0 + 2.0 * proximity_ratio)
                
                # ✅ FIX: gap_angle이 0이면 콘의 위치를 yellow_count 기반으로 추정
                if abs(self.gap_angle) < 3.0 and self.yellow_count > 0:
                    # 콘이 보이지만 중앙에 있음 → 우측으로 회피
                    effective_gap_angle = 25.0
                else:
                    effective_gap_angle = self.gap_angle
                
                steering = -dynamic_gain * effective_gap_angle
                steering = float(np.clip(steering, -self.max_steering, self.max_steering))
                
                dynamic_speed = self.avoid_speed * (0.5 + 1.5 * proximity_ratio)
                
                state = f"YELLOW_AVOID dist={self.min_distance:.3f}m cones={self.yellow_count} gap={self.gap_angle:.1f}° steer={steering:.3f}"
                return dynamic_speed, steering, state
            
            # 상황 3: 50cm 이상 & 회피 중 → 지연 복귀
            if self.avoiding:
                steering = -self.steering_gain * self.gap_angle
                steering = float(np.clip(steering, -self.max_steering, self.max_steering))

                self.clear_count += 1
                if self.clear_count >= self.clear_threshold:
                    self.avoiding = False
                    self.clear_count = 0
                    return self.lane_speed, self.lane_steering, "YELLOW_RETURN_TO_LANE"
                else:
                    return self.avoid_speed, steering, f"YELLOW_AVOIDING_CLEAR {self.clear_count}/{self.clear_threshold}"
        
        # ========================================
        # 정상 모드 (장애물 없음)
        # ========================================
        return self.lane_speed, self.lane_steering, "LANE_NORMAL"
