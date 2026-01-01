#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main_node.py
============
Main Decision Node - Orchestrates all missions based on perception data

- Subscribes to perception topics (각 mission이 구독)
- Decides which mission is active based on priority
- Executes the active mission's step() function
- Publishes motor commands

Priority: PARKING > TRAFFIC_LIGHT > CROSSWALK > OBSTACLE > LANE
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

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
    """Mission Priority States (higher number = higher priority)"""
    LANE = 0
    OBSTACLE = 1
    PARKING = 2
    TRAFFIC_LIGHT = 3
    CROSSWALK = 4


def make_ackermann(speed: float, steer: float) -> AckermannDriveStamped:
    """Make AckermannDriveStamped message"""
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"
    msg.drive.speed = float(speed)
    msg.drive.steering_angle = float(steer)
    return msg


class MainDecisionNode:
    """
    Main Decision Node - Orchestrates all missions
    
    Flow:
    1. 각 mission이 init_from_params()로 perception 토픽 구독
    2. _decide_state()에서 우선순위에 따라 active mission 결정
    3. active mission의 step() 호출 → (speed, steer, debug_str) 반환
    4. make_ackermann()으로 변환 후 모터 명령 발행
    """
    def __init__(self):
        rospy.init_node("wego_main_decision", anonymous=False)
        rospy.loginfo("[main_node] started")

        # ===== Output Topic =====
        self.cmd_topic = rospy.get_param("~cmd_topic",
                                         "/low_level/ackermann_cmd_mux/input/navigation")
        self.pub_cmd = rospy.Publisher(self.cmd_topic, AckermannDriveStamped, queue_size=1)
        self.pub_state = rospy.Publisher("~state", String, queue_size=1)

        # parking debug topics (유지용 - 과거 호환성)
        self.pub_parking_state = rospy.Publisher("/webot/parking/state", String, queue_size=1)
        self.pub_parking_done = rospy.Publisher("/webot/parking/done", Bool, queue_size=1)

        # ===== Global Stop Override =====
        # traffic_stop 토픽으로 모든 모션 중지 가능
        self.stop_flag = False
        stop_topic = rospy.get_param("~traffic_stop_topic", "/webot/traffic_stop")
        self.sub_stop = rospy.Subscriber(stop_topic, Bool, self._cb_stop, queue_size=1)

        # ===== Mission Instances =====
        # 각 mission이 perception 토픽을 구독하고, step()에서 제어값 반환
        self.m_lane = mission_lane.LaneMission()
        self.m_obstacle = mission_obstacle.ObstacleAvoidMission()
        self.m_parking = mission_parking.ParkingMission()
        self.m_traffic_light = mission_traffic_light.TrafficLightMission()
        self.m_crosswalk = mission_crosswalk.CrosswalkMission()

        # 각 mission이 파라미터에서 토픽명과 gain 읽음
        self.m_lane.init_from_params("~lane")
        self.m_obstacle.init_from_params("~obstacle")
        self.m_parking.init_from_params("~parking")
        self.m_traffic_light.init_from_params("~traffic_light")
        self.m_crosswalk.init_from_params("~crosswalk")

        # ===== State Machine =====
        self.state = MissionState.LANE
        self.prev_state = self.state

        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        rospy.loginfo("[main_node] initialized (rate=%.1fHz) cmd_topic=%s", 
                      self.rate_hz, self.cmd_topic)

    def _cb_stop(self, msg: Bool):
        """Global stop override callback"""
        self.stop_flag = bool(msg.data)

    def _decide_state(self) -> MissionState:
        """
        State Machine - 우선순위에 따라 active mission 결정
        
        Priority (높을수록 중요):
        5. PARKING: 주차 표지 감지 시 최우선
        4. TRAFFIC_LIGHT: 빨간 신호등 감지 시 정지
        3. CROSSWALK: 횡단보도 감지
        2. OBSTACLE: 장애물 회피 필요
        1. LANE: 기본 차선 추종
        0. (없음 - 항상 LANE으로 돌아감)
        
        각 mission이 perception 토픽을 구독하고,
        is_active()로 활성화 조건 확인
        """
        # 1️⃣ PARKING (최우선 - 주차 시작)
        if self.m_parking.is_active():
            return MissionState.PARKING

        # 2️⃣ TRAFFIC_LIGHT (안전 - 빨간 신호)
        if self.m_traffic_light.is_active():
            return MissionState.TRAFFIC_LIGHT

        # 3️⃣ CROSSWALK (안전 - 횡단보도)
        if self.m_crosswalk.is_active():
            return MissionState.CROSSWALK

        # 4️⃣ OBSTACLE (회피)
        if self.m_obstacle.is_active():
            return MissionState.OBSTACLE

        # 5️⃣ LANE (기본)
        return MissionState.LANE

        # 4) obstacle third
        # mission_obstacle.py 내부 상태 사용 (이전 답변의 모듈 기준)
        if (self.m_obstacle.min_distance < self.m_obstacle.safe_distance) or self.m_obstacle.avoiding:
            return MissionState.OBSTACLE

        # 5) default lane
        return MissionState.LANE

    def spin(self):
        """
        Main loop - State machine executor
        
        흐름:
        1. _decide_state()로 활성 미션 결정 (우선순위 기반)
        2. 미션 전환 시 on_exit() → on_enter() 호출
        3. active mission의 step() 호출 → (speed, steer, debug) 반환
        4. global stop flag 확인
        5. Ackermann 메시지로 변환 후 발행
        """
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            # ===== Step 1: 현재 활성 미션 결정 =====
            new_state = self._decide_state()
            if new_state != self.state:
                rospy.loginfo("[main_node] STATE CHANGE: %s → %s", 
                             self.state.name, new_state.name)
                self.state = new_state

            # ===== Step 2: 현재 미션 실행 =====
            speed, steer, dbg = 0.0, 0.0, "IDLE"

            if self.state == MissionState.PARKING:
                speed, steer, dbg = self.m_parking.step()
                self.pub_parking_state.publish(String(data=self.m_parking.state))
                self.pub_parking_done.publish(Bool(data=bool(self.m_parking.done)))

            elif self.state == MissionState.TRAFFIC_LIGHT:
                speed, steer, dbg = self.m_traffic_light.step()
                self.pub_parking_state.publish(String(data=mission_parking.ParkingState.IDLE))
                self.pub_parking_done.publish(Bool(data=False))

            elif self.state == MissionState.CROSSWALK:
                speed, steer, dbg = self.m_crosswalk.step()
                self.pub_parking_state.publish(String(data=mission_parking.ParkingState.IDLE))
                self.pub_parking_done.publish(Bool(data=False))

            elif self.state == MissionState.OBSTACLE:
                speed, steer, dbg = self.m_obstacle.step()
                self.pub_parking_state.publish(String(data=mission_parking.ParkingState.IDLE))
                self.pub_parking_done.publish(Bool(data=False))

            else:  # LANE
                speed, steer, dbg = self.m_lane.step()
                self.pub_parking_state.publish(String(data=mission_parking.ParkingState.IDLE))
                self.pub_parking_done.publish(Bool(data=False))

            # ===== Step 3: Global Stop Override (최고 우선순위) =====
            # 신호등이나 수동 정지 신호가 들어오면 모든 미션 중지
            if self.stop_flag:
                speed = 0.0
                steer = 0.0
                dbg = f"🛑 STOP_OVERRIDE | {dbg}"

            # ===== Step 4: 모터 명령 발행 =====
            self.pub_cmd.publish(make_ackermann(speed, steer))
            self.pub_state.publish(String(data=f"{self.state.name} | {dbg}"))

            # ===== 로그 출력 (1초마다) =====
            rospy.loginfo_throttle(1.0, 
                f"[{self.state.name:12s}] v={speed:.2f} steer={steer:.3f} | {dbg}")

            rate.sleep()


if __name__ == "__main__":
    try:
        node = MainDecisionNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
