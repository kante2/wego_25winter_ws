#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32


class LaneMission:
    """
    Lane Control (Ver1 - Simple & Direct)
    - 입력: /webot/steering_offset (dx in pixels from BEV), /webot/lane_speed
    - 출력: (speed, steer) 를 step()에서 반환 (publish는 main_node가 함)
    - 처리: 직접 비례 제어 (dx -> steering_angle)
    """

    def __init__(self):
        self.steering_offset = 0.0  # dx in pixels (from BEV perception)
        self.lane_speed = 0.3
        self.last_update = None

        # Control parameters
        self.steer_limit = 0.6
        self.speed_limit = 1.0
        self.timeout_sec = 0.5
        
        # Steering gain: dx (pixels) -> steering_angle (-0.6 ~ 0.6)
        # dx range: -320 ~ +320 pixels (image width = 640)
        # steering_angle range: -0.35 ~ 0.35 rad (≈ ±20 degrees)
        self.steer_gain = 0.001  # steer = dx * gain (0.001 rad/pixel)
        
        self.sub_steer = None
        self.sub_speed = None

    def init_from_params(self, ns="~lane"):
        """Load parameters from ROS parameter server"""
        steer_topic = rospy.get_param(f"{ns}/steering_offset_topic", "/webot/steering_offset")
        speed_topic = rospy.get_param(f"{ns}/lane_speed_topic", "/webot/lane_speed")

        self.steer_limit = float(rospy.get_param(f"{ns}/steer_limit", 0.6))
        self.speed_limit = float(rospy.get_param(f"{ns}/speed_limit", 1.0))
        self.timeout_sec = float(rospy.get_param(f"{ns}/timeout_sec", 0.5))
        self.steer_gain = float(rospy.get_param(f"{ns}/steer_gain", 0.001))

        self.sub_steer = rospy.Subscriber(steer_topic, Float32, self._cb_steer, queue_size=1)
        self.sub_speed = rospy.Subscriber(speed_topic, Float32, self._cb_speed, queue_size=1)

        rospy.loginfo("[mission_lane] steer_gain=%.4f (rad/px)", self.steer_gain)
        rospy.loginfo("[mission_lane] sub: %s, %s", steer_topic, speed_topic)

    def _cb_steer(self, msg: Float32):
        """Callback for steering_offset (dx in pixels)"""
        self.steering_offset = float(msg.data)
        self.last_update = rospy.Time.now()

    def _cb_speed(self, msg: Float32):
        """Callback for lane_speed"""
        self.lane_speed = float(msg.data)
        self.last_update = rospy.Time.now()

    def on_enter(self):
        rospy.loginfo("[mission_lane] enter")

    def on_exit(self):
        rospy.loginfo("[mission_lane] exit")

    def step(self):
        """
        Simple direct proportional steering control
        Returns: (speed, steer, debug_str)
        """
        # timeout 확인
        if self.last_update is None:
            return 0.0, 0.0, "WAIT_LANE_TOPIC"
        
        dt = (rospy.Time.now() - self.last_update).to_sec()
        if dt > self.timeout_sec:
            return 0.0, 0.0, f"LANE_TIMEOUT({dt:.2f}s)"

        # ===== Direct Proportional Control =====
        # dx: pixels offset from BEV center (negative = left, positive = right)
        dx = self.steering_offset
        
        # Convert dx (pixels) to steering angle (radians)
        steer_raw = dx * self.steer_gain
        
        # Clamp to limits
        steer = max(-self.steer_limit, min(steer_raw, self.steer_limit))
        
        # Speed from lane_speed topic
        speed = max(0.0, min(self.lane_speed, self.speed_limit))
        
        debug_str = f"LANE v={speed:.2f} steer={steer:.3f} (dx={dx:.1f}px)"
        
        return speed, steer, debug_str
