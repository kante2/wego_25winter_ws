#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Int32, Float32
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from dynamic_reconfigure.server import Server
from wego_cfg.cfg import LaneDetectConfig


class LaneMission:
    """
    Lane Following Mission
    - main_node에서 step() 함수로 호출됨
    - Subscriber로 차선 중앙 정보를 받아 내부 상태 업데이트
    - step()에서 (speed, steer, debug_str)을 리턴
    """
    
    def __init__(self):
        # Config (reconfigure_callback에서 동적 설정됨)
        self.config = None
        
        # State
        self.latest_center_x = None
        self.latest_yaw = 0.0
        self.img_center_x = 320.0  # 이미지 중앙 (640px 기준)
        
        # Subscribers (init_from_params에서 생성됨)
        self.sub_center = None
        
        # Dynamic Reconfigure
        self.srv = Server(LaneDetectConfig, self.reconfigure_callback)
    
    def reconfigure_callback(self, config, level):
        self.config = config
        rospy.loginfo(f"[LaneMission] Config updated: speed={config.base_speed}, k={config.k}, yaw_k={config.yaw_k}")
        return config
        
    def init_from_params(self, ns="~lane"):
        """
        main_node가 호출: subscriber 생성 (cfg는 reconfigure_callback에서 관리)
        """
        center_topic = rospy.get_param(f"{ns}/center_topic", "/webot/lane_center_x")
        
        # Subscriber 생성
        self.sub_center = rospy.Subscriber(
            center_topic, Int32, self._center_callback, queue_size=1
        )
        
        rospy.loginfo(f"[LaneMission] initialized")
        rospy.loginfo(f"[LaneMission] subscribed to {center_topic}")

    def _center_callback(self, msg):
        """차선 중앙 x 좌표 수신"""
        self.latest_center_x = msg.data

    def step(self):
        """
        main_node가 매 루프마다 호출
        Returns:
            (speed: float, steer: float, debug: str)
        """
        # Config가 없으면 기본값 사용
        if self.config is None:
            base_speed = 0.4
            k = 0.005
            yaw_k = 1.0
        else:
            base_speed = self.config.base_speed
            k = self.config.k
            yaw_k = self.config.yaw_k
        
        # 차선 정보가 없으면 정지
        if self.latest_center_x is None:
            return 0.0, 0.0, "NO_LANE_DATA"
        
        # Error 계산 (중앙선 - 이미지 중앙)
        error = self.latest_center_x - self.img_center_x
        
        # ROI 기준 yaw 계산 (간단히 error 기반으로 근사)
        # dh_lanefollow는 polyfit 기울기를 사용하지만, 여기선 error 비율로 근사
        yaw = np.arctan2(error, 170.0 * 0.75)  # ROI 높이 170px의 3/4 지점 기준
        
        # Stanley control
        steering = yaw_k * yaw + np.arctan2(k * error, base_speed)
        
        # 디버깅 문자열
        debug = f"cx={self.latest_center_x} err={error:.1f} steer={steering:.3f}"
        
        return base_speed, steering, debug

    def is_active(self):
        """
        (optional) main_node가 lane mission 활성 여부 확인용
        기본적으로 항상 활성 (fallback mission)
        """
        return True


# ===== 단독 실행용 (테스트/디버깅) =====
if __name__ == '__main__':
    try:
        rospy.init_node('mission_lane_standalone')
        
        mission = LaneMission()
        mission.init_from_params("~")
        
        # Ackermann 메시지 발행 (단독 실행시)
        pub_cmd = rospy.Publisher(
            '/low_level/ackermann_cmd_mux/input/navigation',
            AckermannDriveStamped,
            queue_size=1
        )
        
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            speed, steer, debug = mission.step()
            
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'mission_lane'
            msg.drive.speed = speed
            msg.drive.steering_angle = steer
            
            pub_cmd.publish(msg)
            
            rospy.loginfo_throttle(1.0, f"[LaneMission] {debug}")
            
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
