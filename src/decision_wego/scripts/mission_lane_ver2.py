#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mission Lane Control (Ver 2 - BEV + Curvature)
- Based on C++ mission_lane.cpp
- PID + Non-linear gain control
- Curvature-based dynamic steering gain
- Steering and speed command generation
"""

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
import math


class MissionLaneV2:
    """
    Lane control module for trajectory following
    - Input: center_point_px (BEV), curvature
    - Output: Ackermann drive commands
    """
    
    def __init__(self):
        # ========== Topic Names ==========
        self.topic_center_point = "/webot/lane_center_px"
        self.topic_curvature = "/webot/lane_curvature"
        self.cmd_topic = "/low_level/ackermann_cmd_mux/input/navigation"
        
        # ========== BEV Parameters ==========
        self.bev_center_x_px = 320.0  # BEV 이미지 중심 x좌표
        
        # ========== Servo Scaling ==========
        self.servo_center = 0.5
        self.servo_min = 0.0
        self.servo_max = 1.0
        self.steer_sign = -1.0
        
        # ========== Control Parameters ==========
        self.max_abs_dx_px = 83.0      # 최대 오차 (픽셀)
        self.dx_tolerance = 3.0        # 무시 범위
        
        # Steering gain (기본 + 곡률 기반)
        self.steer_gain_base = 1.5
        self.steer_gain_min = 0.8      # 직선 부근
        self.steer_gain_max = 2.0      # 급커브
        self.steer_gain = 1.5          # 현재 gain (매 프레임 갱신)
        
        # EMA smoothing
        self.alpha_ema = 0.2           # 평활 계수
        self.max_delta = 0.08          # 최대 변화율
        
        # Speed planning
        self.base_speed_mps = 0.3
        self.min_speed_mps = 0.1
        self.speed_drop_gain = 0.5
        
        # ========== PID Parameters ==========
        self.pid_kp = 0.02
        self.pid_ki = 0.0
        self.pid_kd = 0.001
        self.pid_weight = 0.3          # PID 가중치 (0.0 = 비선형만, 1.0 = PID만)
        
        # PID state
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_prev_time = None
        
        # ========== Curvature Parameters ==========
        self.min_curvature = 3e-6
        self.max_curvature = 1e-3
        
        # ========== Internal State ==========
        self.prev_steer = 0.0
        self.latest_steer_cmd = 0.0
        self.latest_speed_cmd = 0.0
        
        # Dx spike filter
        self.prev_dx = 0.0
        self.has_prev_dx = False
        self.spike_threshold = 80.0    # 급점프 기준 (픽셀)
        
        # Timeout
        self.dx_timeout_sec = 1.0
        self.last_cb_time = None
        
        # ========== Publishers ==========
        self.pub_cmd = rospy.Publisher(
            self.cmd_topic,
            AckermannDriveStamped,
            queue_size=1
        )
        
        # ========== Subscribers ==========
        self.sub_center = rospy.Subscriber(
            self.topic_center_point,
            PointStamped,
            self.center_callback,
            queue_size=10
        )
        
        self.sub_curvature = rospy.Subscriber(
            self.topic_curvature,
            Float32,
            self.curvature_callback,
            queue_size=10
        )
        
        rospy.loginfo("="*60)
        rospy.loginfo("Mission Lane Ver2 initialized")
        rospy.loginfo(f"Subscribe: {self.topic_center_point}")
        rospy.loginfo(f"Subscribe: {self.topic_curvature}")
        rospy.loginfo(f"Publish: {self.cmd_topic}")
        rospy.loginfo(f"Base speed: {self.base_speed_mps} m/s")
        rospy.loginfo("="*60)
    
    def load_params(self, pnh):
        """Load parameters from ROS parameter server"""
        rospy.loginfo("[lane_v2] Loading parameters...")
        
        self.bev_center_x_px = pnh.get_param('bev_center_x_px', 320.0)
        
        self.servo_center = pnh.get_param('servo_center', 0.5)
        self.servo_min = pnh.get_param('servo_min', 0.0)
        self.servo_max = pnh.get_param('servo_max', 1.0)
        self.steer_sign = pnh.get_param('steer_sign', -1.0)
        
        self.max_abs_dx_px = pnh.get_param('max_abs_dx_px', 83.0)
        self.dx_tolerance = pnh.get_param('dx_tolerance', 3.0)
        self.steer_gain_base = pnh.get_param('steer_gain_base', 1.5)
        self.steer_gain_min = pnh.get_param('steer_gain_min', 0.8)
        self.steer_gain_max = pnh.get_param('steer_gain_max', 2.0)
        self.steer_gain = self.steer_gain_base
        
        self.alpha_ema = pnh.get_param('alpha_ema', 0.2)
        self.max_delta = pnh.get_param('max_delta', 0.08)
        
        self.base_speed_mps = pnh.get_param('base_speed_mps', 0.3)
        self.min_speed_mps = pnh.get_param('min_speed_mps', 0.1)
        self.speed_drop_gain = pnh.get_param('speed_drop_gain', 0.5)
        
        self.pid_kp = pnh.get_param('pid_kp', 0.02)
        self.pid_ki = pnh.get_param('pid_ki', 0.0)
        self.pid_kd = pnh.get_param('pid_kd', 0.001)
        self.pid_weight = pnh.get_param('pid_weight', 0.3)
        
        self.min_curvature = pnh.get_param('min_curvature', 3e-6)
        self.max_curvature = pnh.get_param('max_curvature', 1e-3)
        
        self.dx_timeout_sec = pnh.get_param('dx_timeout_sec', 1.0)
        
        rospy.loginfo("[lane_v2] Parameters loaded")
    
    def clamp(self, x, lo, hi):
        """Clamp value to range [lo, hi]"""
        return max(lo, min(hi, x))
    
    def run_pid(self, error):
        """
        Run one step of PID control
        Returns: pid output (normalized)
        """
        now = rospy.Time.now()
        
        if self.pid_prev_time is None:
            self.pid_prev_time = now
            dt = 0.0
        else:
            dt = (now - self.pid_prev_time).to_sec()
            self.pid_prev_time = now
        
        # P term
        p_term = self.pid_kp * error
        
        # I term (with anti-windup)
        self.pid_integral += error * dt
        self.pid_integral = self.clamp(self.pid_integral, -1000, 1000)
        i_term = self.pid_ki * self.pid_integral
        
        # D term
        d_term = 0.0
        if dt > 1e-4:
            diff = (error - self.pid_prev_error) / dt
            d_term = self.pid_kd * diff
        
        self.pid_prev_error = error
        
        # Raw output (pixel units)
        return p_term + i_term + d_term
    
    def process_dx(self, dx):
        """
        Process dx error and generate control commands
        """
        abs_dx = abs(dx)
        
        # 1) Error normalization
        if abs_dx <= self.dx_tolerance:
            err_norm = 0.0
        else:
            err_norm = self.clamp(dx / self.max_abs_dx_px, -1.0, 1.0)
        
        # 2) Non-linear gain: weak near center, strong at large errors
        gain_scale = 1.0
        if abs_dx < 15.0:
            gain_scale = 0.5      # Suppress oscillation near center
        elif abs_dx > 35.0:
            gain_scale = 1.35     # More aggressive on large error/curve
        
        steer_raw_base = math.tanh(self.steer_gain * gain_scale * err_norm)
        
        # 3) PID correction
        u_pid = self.run_pid(dx)
        steer_pid = self.clamp(u_pid / self.max_abs_dx_px, -1.0, 1.0)
        
        # Combine base + PID
        w_base = 1.0
        steer_raw = w_base * steer_raw_base + self.pid_weight * steer_pid
        steer_raw = self.clamp(steer_raw, -1.0, 1.0)
        
        # 4) EMA smoothing with adaptive alpha/delta
        local_alpha = self.alpha_ema
        local_delta = self.max_delta
        
        if abs_dx < 15.0:
            local_alpha = 0.15    # Smoother near center
            local_delta = 0.05
        elif abs_dx > 35.0:
            local_alpha = 0.28    # Faster on large error
            local_delta = 0.12
        
        steer_smooth = local_alpha * steer_raw + (1.0 - local_alpha) * self.prev_steer
        
        # Limit step-wise change
        delta = self.clamp(steer_smooth - self.prev_steer, -local_delta, local_delta)
        steer_cmd = self.prev_steer + delta
        self.prev_steer = steer_cmd
        
        # 5) Speed planning
        speed_cmd = self.clamp(
            self.base_speed_mps - self.speed_drop_gain * abs(err_norm),
            self.min_speed_mps,
            self.base_speed_mps
        )
        
        self.latest_steer_cmd = steer_cmd  # -1 ~ +1
        self.latest_speed_cmd = speed_cmd  # m/s
        
        rospy.loginfo_throttle(
            1.0,
            f"[lane_v2][CB] dx={dx:.1f}px |dx|={abs_dx:.1f} errN={err_norm:.3f} "
            f"steer={steer_cmd:.3f} v={speed_cmd:.2f} "
            f"(gain_scale={gain_scale:.2f}, alpha={local_alpha:.2f}, steer_gain={self.steer_gain:.3f})"
        )
    
    def center_callback(self, msg):
        """Callback for center point"""
        self.last_cb_time = rospy.Time.now()
        
        # Extract center x coordinate
        cx = msg.point.x
        dx_raw = cx - self.bev_center_x_px  # Right is +, Left is -
        
        # Spike filter
        dx = dx_raw
        if self.has_prev_dx and abs(dx_raw - self.prev_dx) > self.spike_threshold:
            dx = self.prev_dx  # Ignore spike frame
        
        self.prev_dx = dx
        self.has_prev_dx = True
        
        self.process_dx(dx)
    
    def curvature_callback(self, msg):
        """Callback for curvature"""
        curv = msg.data
        k = abs(curv)  # Use magnitude only
        
        if self.max_curvature <= self.min_curvature:
            rospy.logwarn_throttle(
                1.0,
                "[lane_v2] Invalid curvature range (min >= max)"
            )
            self.steer_gain = self.steer_gain_base
            return
        
        # 1) Clamp curvature
        k_clamped = self.clamp(k, self.min_curvature, self.max_curvature)
        
        # 2) Normalize to [0, 1]
        t = (k_clamped - self.min_curvature) / (self.max_curvature - self.min_curvature)
        t = self.clamp(t, 0.0, 1.0)
        
        # 3) Interpolate steer_gain (computed per frame, not accumulated)
        self.steer_gain = self.steer_gain_min + t * (self.steer_gain_max - self.steer_gain_min)
        
        rospy.loginfo_throttle(
            1.0,
            f"[lane_v2] curv={curv:.4e} |k|={k:.4e} t={t:.3f} "
            f"-> steer_gain={self.steer_gain:.3f} "
            f"(range={self.steer_gain_min:.3f}~{self.steer_gain_max:.3f})"
        )
    
    def step(self):
        """
        Main control loop step (called from main_node)
        Generates Ackermann command and publishes
        """
        now = rospy.Time.now()
        have_dx = False
        
        if self.last_cb_time is not None:
            dt = (now - self.last_cb_time).to_sec()
            have_dx = (dt <= self.dx_timeout_sec)
        
        if have_dx:
            steer_cmd = self.latest_steer_cmd  # -1 ~ +1
            speed_cmd = self.latest_speed_cmd  # m/s
        else:
            steer_cmd = 0.0
            speed_cmd = 0.0
        
        # ---- Convert steering: -1~+1 -> steering angle ----
        steer_norm = self.clamp(steer_cmd, -1.0, 1.0)
        
        # Flip direction
        steer_norm *= (-self.steer_sign)
        
        # Map to steering angle (assuming ±0.35 rad = ±20deg max)
        max_steering_angle = 0.35  # radians
        steering_angle = steer_norm * max_steering_angle
        steering_angle = self.clamp(steering_angle, -max_steering_angle, max_steering_angle)
        
        # ---- Publish Ackermann command ----
        msg = AckermannDriveStamped()
        msg.header.stamp = now
        msg.header.frame_id = "base_link"
        msg.drive.speed = float(speed_cmd)
        msg.drive.steering_angle = float(steering_angle)
        
        self.pub_cmd.publish(msg)
        
        rospy.loginfo_throttle(
            1.0,
            f"[lane_v2][step] have_dx={have_dx} steer_cmd={steer_cmd:.3f} "
            f"steering_angle={steering_angle:.4f} speed={speed_cmd:.2f} m/s"
        )


# =====================================================
# ★ Called from main_node.py ★
# =====================================================

# Global instance
g_mission_lane = None


def mission_lane_init(nh, pnh):
    """Initialize mission lane module"""
    global g_mission_lane
    
    rospy.loginfo("[mission_lane_v2] mission_lane_init()")
    
    g_mission_lane = MissionLaneV2()
    g_mission_lane.load_params(pnh)
    
    rospy.loginfo("[mission_lane_v2] mission_lane_init done")


def mission_lane_step():
    """Step function called from main_node control loop"""
    global g_mission_lane
    
    if g_mission_lane is None:
        rospy.logwarn("[mission_lane_v2] Not initialized!")
        return
    
    g_mission_lane.step()


# For standalone testing
if __name__ == '__main__':
    rospy.init_node('mission_lane_v2_test', anonymous=True)
    
    nh = rospy.NodeHandle()
    pnh = rospy.NodeHandle("~")
    
    mission_lane_init(nh, pnh)
    
    rate = rospy.Rate(20)  # 20 Hz
    while not rospy.is_shutdown():
        mission_lane_step()
        rate.sleep()
