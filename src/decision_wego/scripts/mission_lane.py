#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32


class LaneMission:
    """
    - 입력: /webot/steering_offset, /webot/lane_speed
    - 출력: (speed, steer) 를 step()에서 반환 (publish는 main_node가 함)
    """

    def __init__(self):
        self.steering_offset = 0.0
        self.lane_speed = 0.3
        self.last_update = None

        # params
        self.steer_limit = 0.6
        self.speed_limit = 1.0
        self.timeout_sec = 0.5

        self.sub_steer = None
        self.sub_speed = None

    def init_from_params(self, ns="~lane"):
        # ns는 main_node에서 "~lane"처럼 넘겨주는 용도 (그냥 param prefix 느낌)
        steer_topic = rospy.get_param(f"{ns}/steering_offset_topic", "/webot/steering_offset")
        speed_topic = rospy.get_param(f"{ns}/lane_speed_topic", "/webot/lane_speed")

        self.steer_limit = float(rospy.get_param(f"{ns}/steer_limit", 0.6))
        self.speed_limit = float(rospy.get_param(f"{ns}/speed_limit", 1.0))
        self.timeout_sec = float(rospy.get_param(f"{ns}/timeout_sec", 0.5))

        self.sub_steer = rospy.Subscriber(steer_topic, Float32, self._cb_steer, queue_size=1)
        self.sub_speed = rospy.Subscriber(speed_topic, Float32, self._cb_speed, queue_size=1)

        rospy.loginfo("[mission_lane] sub: %s, %s", steer_topic, speed_topic)

    def _cb_steer(self, msg: Float32):
        self.steering_offset = float(msg.data)
        self.last_update = rospy.Time.now()

    def _cb_speed(self, msg: Float32):
        self.lane_speed = float(msg.data)
        self.last_update = rospy.Time.now()

    def on_enter(self):
        rospy.loginfo("[mission_lane] enter")

    def on_exit(self):
        rospy.loginfo("[mission_lane] exit")

    def is_active(self):
        """Lane mission is always the fallback (always active)"""
        return True

    def step(self):
        # timeout이면 정지
        if self.last_update is None:
            return 0.0, 0.0, "WAIT_LANE_TOPIC"
        dt = (rospy.Time.now() - self.last_update).to_sec()
        if dt > self.timeout_sec:
            return 0.0, 0.0, f"LANE_TIMEOUT({dt:.2f}s)"

        steer = max(-self.steer_limit, min(self.steering_offset, self.steer_limit))
        speed = max(0.0, min(self.lane_speed, self.speed_limit))
        return speed, steer, f"LANE v={speed:.2f} steer={steer:.3f}"
