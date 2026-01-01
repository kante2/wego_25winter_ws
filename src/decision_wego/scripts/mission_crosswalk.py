#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool


class CrosswalkMission:
    """
    스텝 함수 기반 크로스워크 미션
    - 입력: /crosswalk_detected (Bool 토픽)
    - 출력: (speed, steer) 를 step()에서 반환 (publish는 main_node가 함)
    """
    
    IDLE = 0
    STOP = 1
    GO   = 2

    def __init__(self):
        # ---------- params ----------
        self.crosswalk_topic = ""
        self.stop_duration_s = 5.0
        self.go_duration_s = 2.0
        self.go_speed = 0.20  # m/s
        self.require_false_to_rearm = True

        # ---------- state ----------
        self.state = self.IDLE
        self.state_end_time = rospy.Time(0)

        self.last_crosswalk = False
        self.armed = True  # True면 트리거 가능

        # ---------- pub/sub ----------
        self.sub_cw = None

    def init_from_params(self, ns="~crosswalk"):
        """메인 노드에서 호출될 초기화 함수"""
        self.crosswalk_topic = rospy.get_param(f"{ns}/crosswalk_topic", "/crosswalk_detected")
        self.stop_duration_s = float(rospy.get_param(f"{ns}/stop_duration", 5.0))
        self.go_duration_s = float(rospy.get_param(f"{ns}/go_duration", 2.0))
        self.go_speed = float(rospy.get_param(f"{ns}/go_speed", 0.20))
        self.require_false_to_rearm = bool(rospy.get_param(f"{ns}/require_false_to_rearm", True))

        self.sub_cw = rospy.Subscriber(self.crosswalk_topic, Bool, self.crosswalk_cb, queue_size=1)

        rospy.loginfo("[mission_crosswalk] init from params (ns=%s)", ns)
        rospy.loginfo("  stop_duration = %.2f s, go_duration = %.2f s", self.stop_duration_s, self.go_duration_s)
        rospy.loginfo("  go_speed = %.2f m/s", self.go_speed)

    def is_active(self):
        """크로스워크 미션이 활성 상태인지 확인"""
        return self.state != self.IDLE

    def on_enter(self):
        """미션 진입 시 호출"""
        rospy.loginfo("[mission_crosswalk] enter")

    def on_exit(self):
        """미션 종료 시 호출"""
        rospy.loginfo("[mission_crosswalk] exit")

    def crosswalk_cb(self, msg: Bool):
        cw = bool(msg.data)

        # 재무장 로직
        if self.require_false_to_rearm:
            if not cw:
                self.armed = True

        # rising edge 트리거 (False -> True)
        rising = (not self.last_crosswalk) and cw
        self.last_crosswalk = cw

        # 이미 수행 중이면 트리거 무시
        if self.state != self.IDLE:
            return

        # 트리거 가능 상태가 아니면 무시
        if self.require_false_to_rearm and (not self.armed):
            return

        if rising:
            # STOP 시작
            self.state = self.STOP
            self.state_end_time = rospy.Time.now() + rospy.Duration.from_sec(self.stop_duration_s)
            self.armed = False  # (false를 보기 전까지) 재트리거 방지
            rospy.logwarn("[mission_crosswalk] Trigger! STOP for %.2fs", self.stop_duration_s)

    def step(self):
        """메인 루프에서 호출되는 스텝 함수"""
        now = rospy.Time.now()

        if self.state == self.IDLE:
            return 0.0, 0.0, "CROSSWALK_IDLE"

        if self.state == self.STOP:
            if now >= self.state_end_time:
                self.state = self.GO
                self.state_end_time = now + rospy.Duration.from_sec(self.go_duration_s)
                rospy.logwarn("[mission_crosswalk] GO straight for %.2fs", self.go_duration_s)
            return 0.0, 0.0, "CROSSWALK_STOP"

        if self.state == self.GO:
            if now >= self.state_end_time:
                self.state = self.IDLE
                rospy.logwarn("[mission_crosswalk] Done -> IDLE")
                return 0.0, 0.0, "CROSSWALK_DONE"
            return self.go_speed, 0.0, f"CROSSWALK_GO v={self.go_speed:.2f}"

        return 0.0, 0.0, "CROSSWALK_UNKNOWN"
