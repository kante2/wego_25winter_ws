#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
mission_traffic_light.py
- Traffic Light Mission for WEGO
- Does NOT publish Ackermann directly.
- Subscribes to traffic light state from perception node
- step() returns (speed, steering, debug_str)

Trigger:
  /webot/traffic_light/state (String) = "RED" | "GREEN" | "UNKNOWN"
"""

import rospy
from std_msgs.msg import String, Float32


class TrafficLightState:
    IDLE = "IDLE"
    WAITING = "WAITING"  # RED light detected, waiting
    PROCEEDING = "PROCEEDING"  # GREEN light or no traffic light system
    DONE = "DONE"


class TrafficLightMission:
    def __init__(self):
        # ===== params =====
        self.speed_slow = 0.3
        self.speed_zero = 0.0
        self.steer_straight = 0.0
        
        # Time to hold stop after RED detected (safety margin)
        self.stop_hold_time = 0.5
        
        # ===== state =====
        self.state = TrafficLightState.IDLE
        self.phase_start_time = None
        self.mission_active = False
        
        # ===== perception data =====
        self.traffic_light_state = "UNKNOWN"  # RED, GREEN, or UNKNOWN
        
        # ===== lane fallback values =====
        self.lane_steering = 0.0
        self.lane_speed = 0.3
        
        self._sub_traffic_light = None
        self._sub_lane_steer = None
        self._sub_lane_speed = None
    
    def init_from_params(self, ns="~traffic_light"):
        """Initialize from ROS parameters"""
        self.speed_slow = float(rospy.get_param(f"{ns}/speed_slow", 0.3))
        self.speed_zero = float(rospy.get_param(f"{ns}/speed_zero", 0.0))
        self.steer_straight = float(rospy.get_param(f"{ns}/steer_straight", 0.0))
        self.stop_hold_time = float(rospy.get_param(f"{ns}/stop_hold_time", 0.5))
        
        # topics
        traffic_light_topic = rospy.get_param(f"{ns}/traffic_light_state_topic", "/webot/traffic_light/state")
        lane_steer_topic = rospy.get_param(f"{ns}/lane_steering_topic", "/webot/steering_offset")
        lane_speed_topic = rospy.get_param(f"{ns}/lane_speed_topic", "/webot/lane_speed")
        
        self._sub_traffic_light = rospy.Subscriber(traffic_light_topic, String, self._cb_traffic_light, queue_size=1)
        self._sub_lane_steer = rospy.Subscriber(lane_steer_topic, Float32, self._cb_lane_steer, queue_size=1)
        self._sub_lane_speed = rospy.Subscriber(lane_speed_topic, Float32, self._cb_lane_speed, queue_size=1)
        
        rospy.loginfo("[mission_traffic_light] initialized, subscribing to '%s'", traffic_light_topic)
    
    # ---------- callbacks ----------
    def _cb_traffic_light(self, msg: String):
        """Update traffic light state from perception"""
        self.traffic_light_state = msg.data
    
    def _cb_lane_steer(self, msg: Float32):
        """Fallback steering from lane detection"""
        self.lane_steering = float(msg.data)
    
    def _cb_lane_speed(self, msg: Float32):
        """Fallback speed from lane detection"""
        self.lane_speed = float(msg.data)
    
    def step(self, dt=0.05):
        """
        Control step for traffic light mission.
        
        Args:
            dt: time delta (default 0.05s for 20Hz control loop)
        
        Returns:
            (speed, steering_angle, debug_str)
            - speed: motor command (-1.0 to 1.0)
            - steering_angle: steering command (-1.0 to 1.0)
            - debug_str: debug message
        """
        
        # State machine: decide next state based on traffic light color
        current_time = rospy.Time.now().to_sec()
        if self.phase_start_time is None:
            self.phase_start_time = current_time
        
        elapsed = current_time - self.phase_start_time
        
        # State transitions
        if self.traffic_light_state == "RED":
            # RED light detected -> STOP
            if self.state != TrafficLightState.WAITING:
                rospy.loginfo("[mission_traffic_light] RED light detected, STOPPING")
                self.state = TrafficLightState.WAITING
                self.phase_start_time = current_time
            
            # Stay stopped
            speed = self.speed_zero
            steering = self.steer_straight
            debug_str = f"[TL_RED] stopped at RED light"
        
        elif self.traffic_light_state == "GREEN":
            # GREEN light -> PROCEED
            if self.state == TrafficLightState.WAITING:
                rospy.loginfo("[mission_traffic_light] GREEN light detected, PROCEEDING")
                self.state = TrafficLightState.PROCEEDING
                self.phase_start_time = current_time
            
            # Use lane control values
            speed = self.lane_speed
            steering = self.lane_steering
            debug_str = f"[TL_GREEN] proceeding with lane control"
        
        else:
            # UNKNOWN (no traffic light system or detection failure)
            # -> proceed normally with lane control
            if self.state == TrafficLightState.IDLE:
                self.state = TrafficLightState.PROCEEDING
            
            speed = self.lane_speed
            steering = self.lane_steering
            debug_str = f"[TL_UNKNOWN] using lane control"
        
        return (speed, steering, debug_str)
    
    def reset(self):
        """Reset mission state"""
        self.state = TrafficLightState.IDLE
        self.phase_start_time = None
        self.mission_active = False
        rospy.loginfo("[mission_traffic_light] reset")
    
    def is_active(self):
        """Check if mission is actively controlling (RED light detected)"""
        return self.traffic_light_state == "RED"
    
    def get_state(self):
        """Return current internal state"""
        return self.state
    
    def get_debug_info(self):
        """Return detailed debug info"""
        return {
            "state": self.state,
            "traffic_light_color": self.traffic_light_state,
            "lane_steering": self.lane_steering,
            "lane_speed": self.lane_speed,
        }


if __name__ == '__main__':
    # Example usage (for debugging)
    rospy.init_node('traffic_light_mission_test')
    mission = TrafficLightMission()
    mission.init_from_params()
    
    rate = rospy.Rate(20)  # 20 Hz
    while not rospy.is_shutdown():
        speed, steer, debug = mission.step()
        rospy.loginfo_throttle(1, f"Speed: {speed:.2f}, Steer: {steer:.3f}, {debug}")
        rate.sleep()
