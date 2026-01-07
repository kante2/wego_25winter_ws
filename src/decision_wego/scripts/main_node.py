#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os

# Add scripts directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

import rospy
from enum import IntEnum
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDriveStamped

import mission_lane
import mission_obstacle
import mission_parking
import mission_crosswalk
import mission_traffic_light


class MissionState(IntEnum):
    LANE = 0
    OBSTACLE = 1
    PARKING = 2
    TRAFFIC_LIGHT = 3
    CROSSWALK = 4


def make_ackermann(speed: float, steer: float) -> AckermannDriveStamped:
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"
    msg.drive.speed = float(speed)
    msg.drive.steering_angle = float(steer)
    return msg


class MainDecisionNode:
    def __init__(self):
        rospy.init_node("wego_main_decision", anonymous=False)
        rospy.loginfo("[main_node] started")

        # ===== output =====
        self.cmd_topic = rospy.get_param("~cmd_topic",
                                         "/low_level/ackermann_cmd_mux/input/navigation")
        self.pub_cmd = rospy.Publisher(self.cmd_topic, AckermannDriveStamped, queue_size=1)
        self.pub_state = rospy.Publisher("~state", String, queue_size=1)

        # parking debug topics (원래 parking_node가 내던 토픽 유지)
        self.pub_parking_state = rospy.Publisher("/webot/parking/state", String, queue_size=1)
        self.pub_parking_done = rospy.Publisher("/webot/parking/done", Bool, queue_size=1)

        # (optional) traffic stop override
        self.stop_flag = False
        stop_topic = rospy.get_param("~traffic_stop_topic", "/webot/traffic_stop")
        self.sub_stop = rospy.Subscriber(stop_topic, Bool, self._cb_stop, queue_size=1)

        # ===== missions =====
        self.m_lane = mission_lane.LaneMission()
        self.m_obstacle = mission_obstacle.ObstacleAvoidMission()
        self.m_parking = mission_parking.ParkingMission()
        self.m_traffic_light = mission_traffic_light.TrafficLightMission()
        self.m_crosswalk = mission_crosswalk.CrosswalkMission()

        self.m_lane.init_from_params("~lane")
        self.m_obstacle.init_from_params("~obstacle")
        self.m_parking.init_from_params("~parking")
        self.m_traffic_light.init_from_params("~traffic_light")
        self.m_crosswalk.init_from_params("~crosswalk")

        self.state = MissionState.LANE
        self.prev_state = self.state

        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        rospy.loginfo("[main_node] init done (rate=%.1fHz) cmd=%s", self.rate_hz, self.cmd_topic)

    def _cb_stop(self, msg: Bool):
        self.stop_flag = bool(msg.data)

    def _decide_state(self) -> MissionState:
        """
        Priority: Marker(PARKING) > TRAFFIC_LIGHT(RED) > CROSSWALK > OBSTACLE > LANE
        - PARKING: parking mission is triggered/active
        - TRAFFIC_LIGHT: traffic light is RED (must stop)
        - CROSSWALK: crosswalk mission is active
        - OBSTACLE: obstacle avoidance required
        - LANE: default lane following
        """
        # 1) parking first (highest priority)
        if self.m_parking.is_active():
            return MissionState.PARKING

        # 2) traffic light second (RED light is critical safety)
        if self.m_traffic_light.is_active():
            return MissionState.TRAFFIC_LIGHT

        # 3) crosswalk third
        if self.m_crosswalk.is_active():
            return MissionState.CROSSWALK

        # 4) obstacle third
        # mission_obstacle.py 내부 상태 사용 (이전 답변의 모듈 기준)
        if (self.m_obstacle.min_distance < self.m_obstacle.safe_distance) or self.m_obstacle.avoiding:
            return MissionState.OBSTACLE

        # 5) default lane
        return MissionState.LANE

    def spin(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            # decide state
            new_state = self._decide_state()
            if new_state != self.state:
                rospy.loginfo("[main_node] state: %s -> %s", self.state.name, new_state.name)
                self.state = new_state

            # run one step
            speed, steer, dbg = 0.0, 0.0, "IDLE"

            if self.state == MissionState.PARKING:
                speed, steer, dbg = self.m_parking.step()
                # parking state/done publish
                self.pub_parking_state.publish(String(data=self.m_parking.state))
                self.pub_parking_done.publish(Bool(data=bool(self.m_parking.done)))

            elif self.state == MissionState.TRAFFIC_LIGHT:
                speed, steer, dbg = self.m_traffic_light.step()
                # parking debug topic is idle
                self.pub_parking_state.publish(String(data=mission_parking.ParkingState.IDLE))
                self.pub_parking_done.publish(Bool(data=False))

            elif self.state == MissionState.CROSSWALK:
                speed, steer, dbg = self.m_crosswalk.step()
                # parking debug topic is idle
                self.pub_parking_state.publish(String(data=mission_parking.ParkingState.IDLE))
                self.pub_parking_done.publish(Bool(data=False))

            elif self.state == MissionState.OBSTACLE:
                speed, steer, dbg = self.m_obstacle.step()

                # parking debug topic is idle
                self.pub_parking_state.publish(String(data=mission_parking.ParkingState.IDLE))
                self.pub_parking_done.publish(Bool(data=False))

            else:  # LANE
                speed, steer, dbg = self.m_lane.step()
                self.pub_parking_state.publish(String(data=mission_parking.ParkingState.IDLE))
                self.pub_parking_done.publish(Bool(data=False))

            # traffic stop override (global 최고 우선)
            if self.stop_flag:
                speed = 0.0
                steer = 0.0
                dbg = f"STOP_OVERRIDE | {dbg}"

            # publish cmd
            self.pub_cmd.publish(make_ackermann(speed, steer))
            self.pub_state.publish(String(data=f"{self.state.name} | {dbg}"))

            rospy.loginfo_throttle(
                1.0,
                "[main_node] state=%s stop=%d | v=%.2f steer=%.3f | %s",
                self.state.name, int(self.stop_flag), speed, steer, dbg
            )

            rate.sleep()


if __name__ == "__main__":
    try:
        node = MainDecisionNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
