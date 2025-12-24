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
        self.safe_distance = 0.5
        self.stop_distance = 0.2
        self.avoid_speed = 0.2
        self.max_steering = 0.5
        self.steering_gain = 0.02
        self.clear_threshold = 20

        # perception
        self.gap_angle = 0.0
        self.gap_width = 1
        self.min_distance = 10.0

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

        rospy.Subscriber(t_gap_angle, Float32, self._cb_gap_angle, queue_size=1)
        rospy.Subscriber(t_gap_width, Int32, self._cb_gap_width, queue_size=1)
        rospy.Subscriber(t_min_dist, Float32, self._cb_min_dist, queue_size=1)
        rospy.Subscriber(t_lane_steer, Float32, self._cb_lane_steer, queue_size=1)
        rospy.Subscriber(t_lane_speed, Float32, self._cb_lane_speed, queue_size=1)

        rospy.loginfo("[mission_obstacle] sub: %s %s %s", t_gap_angle, t_gap_width, t_min_dist)

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

    def on_enter(self):
        rospy.loginfo("[mission_obstacle] enter")

    def on_exit(self):
        rospy.loginfo("[mission_obstacle] exit")
        self.avoiding = False
        self.clear_count = 0

    def step(self):
        state = "IDLE"

        # emergency stop
        if self.min_distance < self.stop_distance:
            self.avoiding = True
            self.clear_count = 0
            return 0.0, 0.0, f"TOO_CLOSE min={self.min_distance:.2f}m"

        # obstacle in front
        if self.min_distance < self.safe_distance:
            self.avoiding = True
            self.clear_count = 0

            steering = -self.steering_gain * self.gap_angle
            steering = float(np.clip(steering, -self.max_steering, self.max_steering))
            state = f"AVOID gap={self.gap_angle:.1f}deg width={self.gap_width} min={self.min_distance:.2f}"
            return self.avoid_speed, steering, state

        # avoiding but currently clear -> hold for N cycles
        if self.avoiding:
            steering = -self.steering_gain * self.gap_angle
            steering = float(np.clip(steering, -self.max_steering, self.max_steering))

            self.clear_count += 1
            if self.clear_count >= self.clear_threshold:
                self.avoiding = False
                self.clear_count = 0
                return self.lane_speed, self.lane_steering, "RETURN_TO_LANE"
            else:
                return self.avoid_speed, steering, f"AVOID_CLEAR {self.clear_count}/{self.clear_threshold}"

        # normal fallback
        return self.lane_speed, self.lane_steering, "LANE_FALLBACK"
