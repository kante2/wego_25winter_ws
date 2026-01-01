WEGO 실행 순서

ssh wego@192.168.1.11
ssh -X wego@192.168.1.11


1단계: 기본 설정 (필수)

roslaunch wego bringup.launch


저수준 드라이버 로드 (카메라, LiDAR, 모터 제어)
기본 하드웨어 초기화


----------------------------

2단계: 인지(Perception) 노드 시작
roslaunch perception_wego perception_all.launch


----------------------------
3단계: 의사결정(Decision) 메인 노드 시작
roslaunch decision_wego decision_all

./home/wego/catkin_ws/src/perception_wego/debug/camera_yellow_filter_debug.py