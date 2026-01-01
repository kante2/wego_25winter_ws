#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
import math


class LaneMission:
    """
    Lane Control with PID (Ver2 - BEV 기반)
    - 입력: /webot/steering_offset (dx in pixels from BEV), /webot/lane_speed
    - 출력: (speed, steer) 를 step()에서 반환 (publish는 main_node가 함)
    """

    def __init__(self):
        self.steering_offset = 0.0  # dx in pixels (BEV-based)
        self.lane_speed = 0.3
        self.last_update = None

        # PID parameters
        self.pid_kp = 0.01      # P gain
        self.pid_ki = 0.001     # I gain
        self.pid_kd = 0.0005    # D gain
        self.max_integral = 0.1  # Anti-windup
        
        # PID state
        self.integral = 0.0
        self.prev_error = 0.0
        
        # Control limits
        self.steer_limit = 0.6
        self.speed_limit = 1.0
        self.timeout_sec = 0.5
        self.dx_tolerance = 3.0  # pixels (dead zone)
        self.max_abs_dx = 83.0   # max error range (pixels)

        self.sub_steer = None
        self.sub_speed = None

    def init_from_params(self, ns="~lane"):
        # ns는 main_node에서 "~lane"처럼 넘겨주는 용도
        steer_topic = rospy.get_param(f"{ns}/steering_offset_topic", "/webot/steering_offset")
        speed_topic = rospy.get_param(f"{ns}/lane_speed_topic", "/webot/lane_speed")

        self.steer_limit = float(rospy.get_param(f"{ns}/steer_limit", 0.6))
        self.speed_limit = float(rospy.get_param(f"{ns}/speed_limit", 1.0))
        self.timeout_sec = float(rospy.get_param(f"{ns}/timeout_sec", 0.5))
        
        # PID parameters
        self.pid_kp = float(rospy.get_param(f"{ns}/pid_kp", 0.01))
        self.pid_ki = float(rospy.get_param(f"{ns}/pid_ki", 0.001))
        self.pid_kd = float(rospy.get_param(f"{ns}/pid_kd", 0.0005))
        self.max_integral = float(rospy.get_param(f"{ns}/max_integral", 0.1))
        
        self.dx_tolerance = float(rospy.get_param(f"{ns}/dx_tolerance", 3.0))
        self.max_abs_dx = float(rospy.get_param(f"{ns}/max_abs_dx", 83.0))

        self.sub_steer = rospy.Subscriber(steer_topic, Float32, self._cb_steer, queue_size=1)
        self.sub_speed = rospy.Subscriber(speed_topic, Float32, self._cb_speed, queue_size=1)

        rospy.loginfo("[mission_lane_pid] PID Params: kp=%.4f, ki=%.4f, kd=%.4f", 
                     self.pid_kp, self.pid_ki, self.pid_kd)
        rospy.loginfo("[mission_lane_pid] sub: %s, %s", steer_topic, speed_topic)

    def _cb_steer(self, msg: Float32):
        """Callback for steering_offset (dx in pixels)"""
        self.steering_offset = float(msg.data)
        self.last_update = rospy.Time.now()

    def _cb_speed(self, msg: Float32):
        """Callback for lane_speed"""
        self.lane_speed = float(msg.data)
        self.last_update = rospy.Time.now()

    def on_enter(self):
        rospy.loginfo("[mission_lane_pid] enter")
        self.integral = 0.0
        self.prev_error = 0.0

    def on_exit(self):
        rospy.loginfo("[mission_lane_pid] exit")

    def step(self):
        """
        PID-based steering control
        Returns: (speed, steer, debug_str)
        """
        # timeout 확인
        if self.last_update is None:
            return 0.0, 0.0, "WAIT_LANE_TOPIC"
        
        dt = (rospy.Time.now() - self.last_update).to_sec()
        if dt > self.timeout_sec:
            return 0.0, 0.0, f"LANE_TIMEOUT({dt:.2f}s)"

        # ===== PID Control =====
        dx = self.steering_offset  # pixels (from BEV)
        
        # Dead zone
        if abs(dx) < self.dx_tolerance:
            error = 0.0
        else:
            # Normalize error to [-1, 1]
            error = max(-1.0, min(1.0, dx / self.max_abs_dx))
        
        # P term
        p_term = self.pid_kp * error
        
        # I term (anti-windup)
        self.integral += error * (dt if dt > 0 else 0.01)
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        i_term = self.pid_ki * self.integral
        
        # D term
        if dt > 0:
            d_term = self.pid_kd * (error - self.prev_error) / dt
        else:
            d_term = 0.0
        self.prev_error = error
        
        # PID output
        steer_raw = p_term + i_term + d_term
        steer = max(-self.steer_limit, min(steer_raw, self.steer_limit))
        
        # Speed from lane_speed topic
        speed = max(0.0, min(self.lane_speed, self.speed_limit))
        
        debug_str = f"LANE v={speed:.2f} steer={steer:.3f} [pid: p={p_term:.3f} i={i_term:.3f} d={d_term:.3f}]"
        
        return speed, steer, debug_str
