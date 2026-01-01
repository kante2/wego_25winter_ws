#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
mission_parking.py
- Parking Mission (reverse parallel parking) for WEGO
- Does NOT publish Ackermann directly.
- Subscribes to ArUco marker_info and (optionally) lane topics for reference.
- step() returns (speed, steering, debug_str)

Trigger:
  /webot/aruco/marker_info (Float32MultiArray) = [id, dist, id, dist, ...]
"""

import rospy
from std_msgs.msg import Float32, Float32MultiArray


class ParkingState:
    IDLE = "IDLE"
    STOP_BEFORE = "STOP_BEFORE"
    FORWARD_PASS = "FORWARD_PASS"
    STOP_ALIGN = "STOP_ALIGN"
    REVERSE_RIGHT = "REVERSE_RIGHT"
    REVERSE_LEFT = "REVERSE_LEFT"
    FORWARD_STRAIGHT = "FORWARD_STRAIGHT"
    DONE = "DONE"


class ParkingMission:
    def __init__(self):
        # ===== params (default values are same as your parking_node) =====
        self.speed_slow = 0.15
        self.speed_reverse = -0.13
        self.steering_angle = 0.5

        self.forward_time = 1.5
        self.reverse_right_time = 3.0
        self.reverse_left_time = 3.0
        self.final_forward_time = 1.0

        self.stop_before_sec = 1.0
        self.stop_align_sec = 1.0

        self.trigger_distance = 0.5
        self.target_marker_id = 0

        # ===== state =====
        self.state = ParkingState.IDLE
        self.phase_start_time = None
        self.parking_triggered = False  # latch once
        self.done = False

        # ===== last seen marker info (debug) =====
        self.last_marker_dist = None

        # ===== lane values (optional reference) =====
        self.lane_steering = 0.0
        self.lane_speed = 0.3

        self._sub_aruco = None
        self._sub_lane_steer = None
        self._sub_lane_speed = None

    def init_from_params(self, ns="~parking"):
        # params
        self.speed_slow = float(rospy.get_param(f"{ns}/speed_slow", 0.15))
        self.speed_reverse = float(rospy.get_param(f"{ns}/speed_reverse", -0.13))
        self.steering_angle = float(rospy.get_param(f"{ns}/steering_angle", 0.5))

        self.forward_time = float(rospy.get_param(f"{ns}/forward_time", 1.5))
        self.reverse_right_time = float(rospy.get_param(f"{ns}/reverse_right_time", 3.0))
        self.reverse_left_time = float(rospy.get_param(f"{ns}/reverse_left_time", 3.0))
        self.final_forward_time = float(rospy.get_param(f"{ns}/final_forward_time", 1.0))

        self.stop_before_sec = float(rospy.get_param(f"{ns}/stop_before_sec", 1.0))
        self.stop_align_sec = float(rospy.get_param(f"{ns}/stop_align_sec", 1.0))

        self.trigger_distance = float(rospy.get_param(f"{ns}/trigger_distance", 0.5))
        self.target_marker_id = int(rospy.get_param(f"{ns}/target_marker_id", 0))

        # topics
        aruco_topic = rospy.get_param(f"{ns}/aruco_marker_info_topic", "/webot/aruco/marker_info")
        lane_steer_topic = rospy.get_param(f"{ns}/lane_steering_topic", "/webot/steering_offset")
        lane_speed_topic = rospy.get_param(f"{ns}/lane_speed_topic", "/webot/lane_speed")

        self._sub_aruco = rospy.Subscriber(aruco_topic, Float32MultiArray, self._cb_aruco, queue_size=1)
        # lane topics are optional references (not required for parking control itself)
        self._sub_lane_steer = rospy.Subscriber(lane_steer_topic, Float32, self._cb_lane_steer, queue_size=1)
        self._sub_lane_speed = rospy.Subscriber(lane_speed_topic, Float32, self._cb_lane_speed, queue_size=1)

        rospy.loginfo("[mission_parking] sub aruco='%s' (target_id=%d, trigger=%.2fm)",
                      aruco_topic, self.target_marker_id, self.trigger_distance)

    # ---------- callbacks ----------
    def _cb_lane_steer(self, msg: Float32):
        self.lane_steering = float(msg.data)

    def _cb_lane_speed(self, msg: Float32):
        self.lane_speed = float(msg.data)

    def _cb_aruco(self, msg: Float32MultiArray):
        """
        msg.data = [id1, dist1, id2, dist2, ...]
        Trigger only when state==IDLE and not triggered.
        """
        if self.done or self.parking_triggered or self.state != ParkingState.IDLE:
            return

        data = list(msg.data)
        if len(data) < 2:
            return

        for i in range(0, len(data) - 1, 2):
            marker_id = int(data[i])
            dist = float(data[i + 1])

            if marker_id == self.target_marker_id:
                self.last_marker_dist = dist
                rospy.loginfo_throttle(0.5,
                    "[mission_parking] ArUco ID:%d dist=%.3fm (trigger<=%.2f)",
                    marker_id, dist, self.trigger_distance
                )

                if dist <= self.trigger_distance:
                    rospy.loginfo("[mission_parking] Triggered by ArUco ID %d (dist=%.3fm)",
                                  marker_id, dist)
                    self.parking_triggered = True
                    self._set_state(ParkingState.STOP_BEFORE)
                return

    # ---------- helpers ----------
    def _set_state(self, new_state: str):
        rospy.loginfo("[mission_parking] Phase: %s -> %s", self.state, new_state)
        self.state = new_state
        self.phase_start_time = rospy.Time.now()

    def _elapsed(self) -> float:
        if self.phase_start_time is None:
            return 0.0
        return (rospy.Time.now() - self.phase_start_time).to_sec()

    def is_active(self) -> bool:
        """main_node에서 '주차가 제어권을 가져야 하는지' 판단용"""
        return self.state != ParkingState.IDLE

    # ---------- step ----------
    def step(self):
        """
        Returns: (speed, steering, debug_str)
        Note: In IDLE state, parking mission does NOT take control (main_node should use lane/obstacle).
        """
        # IDLE: 제어권 없음
        if self.state == ParkingState.IDLE:
            dbg = "PARKING_IDLE"
            if self.last_marker_dist is not None:
                dbg += f" marker_dist={self.last_marker_dist:.2f}"
            return 0.0, 0.0, dbg

        # DONE: 계속 정지 유지
        if self.state == ParkingState.DONE:
            self.done = True
            return 0.0, 0.0, "PARKING_DONE"

        # STOP_BEFORE
        if self.state == ParkingState.STOP_BEFORE:
            if self._elapsed() > self.stop_before_sec:
                self._set_state(ParkingState.FORWARD_PASS)
            return 0.0, 0.0, f"STOP_BEFORE {self._elapsed():.1f}/{self.stop_before_sec}"

        # FORWARD_PASS
        if self.state == ParkingState.FORWARD_PASS:
            if self._elapsed() > self.forward_time:
                self._set_state(ParkingState.STOP_ALIGN)
            return self.speed_slow, 0.0, f"FORWARD_PASS {self._elapsed():.1f}/{self.forward_time}"

        # STOP_ALIGN
        if self.state == ParkingState.STOP_ALIGN:
            if self._elapsed() > self.stop_align_sec:
                self._set_state(ParkingState.REVERSE_RIGHT)
            return 0.0, 0.0, f"STOP_ALIGN {self._elapsed():.1f}/{self.stop_align_sec}"

        # REVERSE_RIGHT
        if self.state == ParkingState.REVERSE_RIGHT:
            if self._elapsed() >= self.reverse_right_time:
                self._set_state(ParkingState.REVERSE_LEFT)
            return self.speed_reverse, +self.steering_angle, \
                   f"REVERSE_RIGHT {self._elapsed():.1f}/{self.reverse_right_time}"

        # REVERSE_LEFT
        if self.state == ParkingState.REVERSE_LEFT:
            if self._elapsed() >= self.reverse_left_time:
                self._set_state(ParkingState.FORWARD_STRAIGHT)
            return self.speed_reverse, -self.steering_angle, \
                   f"REVERSE_LEFT {self._elapsed():.1f}/{self.reverse_left_time}"

        # FORWARD_STRAIGHT
        if self.state == ParkingState.FORWARD_STRAIGHT:
            if self._elapsed() > self.final_forward_time:
                self._set_state(ParkingState.DONE)
            return self.speed_slow, 0.0, f"FORWARD_STRAIGHT {self._elapsed():.1f}/{self.final_forward_time}"

        # fallback
        return 0.0, 0.0, f"PARKING_UNKNOWN_STATE({self.state})"
