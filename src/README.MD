# 전체 시스템 (권장)
roslaunch decision_25 complete_25.launch

# 각 노드 개별 테스트
rostopic echo /webot/lane_center              # 차선 감지
rostopic echo /commands/motor/speed            # 모터 명령
rostopic hz /webot/lane_center                # 30Hz 확인