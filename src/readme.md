WEGO 실행 순서

========================================
🌐 REMOTE SSH 연결 (원격 시스템 접속)
========================================

1️⃣ 기본 SSH 연결 (터미널만)
  ssh wego@192.168.1.11

2️⃣ GUI 지원 SSH 연결 (X11 포워딩 - GUI 실행 가능)
  ssh -X wego@192.168.1.11

3️⃣ 특정 포트 지정
  ssh -p 22 wego@192.168.1.11

💡 팁:
  - ssh-copy-id를 사용하여 공개키 인증 설정 가능
  - ssh wego@192.168.1.11 "명령어" 형태로 원격 명령 실행 가능

========================================
WEGO 시스템 실행 순서
========================================

원격 시스템에서:

1단계: 기본 설정 (필수)

ssh -X wego@192.168.1.11
source ~/catkin_ws/devel/setup.bash
roslaunch wego_cfg bringup.launch

저수준 드라이버 로드 (카메라, LiDAR, 모터 제어, IMU)
기본 하드웨어 초기화

----------------------------

2단계: 인지(Perception) 노드 시작 (Ver2: BEV + Sliding Window)
ssh -X wego@192.168.1.11
source ~/catkin_ws/devel/setup.bash
roslaunch perception_wego perception_all.launch

- lane_detect_perception_ver2.py: BEV 변환 + 슬라이딩 윈도우 차선 감지 (WHITE 차선)
  → /webot/steering_offset (Float32) - 스티어링 오프셋 (픽셀 단위)
  → /webot/lane_speed (Float32) - 차선 추종 속도

----------------------------

3단계: 의사결정(Decision) 메인 노드 시작 (Mission Orchestrator)
ssh -X wego@192.168.1.11
source ~/catkin_ws/devel/setup.bash
roslaunch decision_wego decision_all.launch

- main_node.py: 모든 mission을 우선순위 기반으로 조율
  * 각 mission은 perception 토픽을 구독하고 step()에서 (speed, steer, debug) 반환
  * 우선순위: PARKING > TRAFFIC_LIGHT > CROSSWALK > OBSTACLE > LANE
  → /low_level/ackermann_cmd_mux/input/navigation (AckermannDriveStamped)

========================================
📊 데이터 흐름
========================================

Perception (인지 - Ver2)
  ├── lane_detect_perception_ver2.py (BEV + Sliding Window)
  │   ├── /webot/lane_center_px (PointStamped)
  │   └── /webot/lane_curvature (Float32)
  ├── traffic_light_detect_node.py
  │   └── /webot/traffic_light/state
  └── obstacle_avoid_perception.py
      └── /webot/obstacle/*

         ↓↓↓ main_node.py 구독 ↓↓↓

Decision (의사결정 - Ver2)
  └── main_node.py
      └── mission_lane_ver2.py (PID + 곡률 기반 gain)
          └── /low_level/ackermann_cmd_mux/input/navigation
              → 모터 제어 (speed, steering_angle)

========================================
🔧 DEBUG 노드 사용 방법
========================================

perception_wego/debug 폴더의 스크립트들:

1️⃣ camera_traffic_light_debug.py
   🚦 신호등 감지 디버깅
   - 신호등 색상 감지 (RED/GREEN)
   - ROI 영역 % 표시
   - 마스크 영역 % 표시
   - 감지율 통계
   
   실행:
   rosrun perception_wego camera_traffic_light_debug.py

2️⃣ camera_lane_detect_roi_debug.py
   🛣️ 차선 감지 ROI 디버깅
   - ROI 영역 시각화
   - 차선 히스토그램 표시
   - ROI 위치 조정 시 유용
   
   실행:
   rosrun perception_wego camera_lane_detect_roi_debug.py

3️⃣ camera_yellow_filter_debug.py
   🟡 노란색 필터 디버깅
   - HSV 범위 튜닝
   - 노란색 마스크 실시간 확인
   - rqt_reconfigure와 함께 사용
   
   실행:
   rosrun perception_wego camera_yellow_filter_debug.py

4️⃣ camera_capture_debug.py
   📷 카메라 캡처 테스트
   - 카메라 연결 확인
   - 이미지 품질 확인
   - FPS 측정
   
   실행:
   rosrun perception_wego camera_capture_debug.py

5️⃣ lidar_gap_debug.py / lidar_yellow_gap_debug.py
   🔴 LiDAR 갭 감지 디버깅
   - 장애물 사이 갭 탐지
   - 주행 경로 계산 시각화
   
   실행:
   rosrun perception_wego lidar_gap_debug.py

========================================
🎨 파라미터 튜닝 (rqt_reconfigure)
========================================

실시간으로 파라미터 조정:

# 새 터미널에서 실행
rosrun rqt_reconfigure rqt_reconfigure

그 후:
1. 좌측 패널에서 해당 노드 선택 (lane_detect, traffic_light 등)
2. HSV 범위, ROI, PID 게인 등 조정
3. 우측 카메라 윈도우에서 실시간 변화 확인

========================================
📊 이미지 뷰어로 시각화
========================================

# 차선 감지 결과
rqt_image_view /webot/lane_detect/image

# 차선 마스크
rqt_image_view /webot/lane_detect/mask

# 신호등 감지 결과
rqt_image_view /webot/traffic_light/image

# 신호등 마스크
rqt_image_view /webot/traffic_light/debug

========================================
💡 일반적인 디버깅 워크플로우
========================================

1️⃣ 문제 파악
   rosrun perception_wego camera_[node]_debug.py
   → CV 창에서 문제 원인 확인

2️⃣ 파라미터 튜닝
   rosrun rqt_reconfigure rqt_reconfigure
   → HSV, ROI, 게인 값 조정

3️⃣ 결과 확인
   rqt_image_view /webot/[topic]/image
   → 실시간으로 개선 상황 모니터링

4️⃣ 파라미터 저장
   ~/.ros/dynamic_reconfigure_params/ 에 자동 저장됨


1️⃣ bringup.launch (하드웨어 초기화)
↓
카메라, LiDAR, 모터 제어, IMU, Ackermann MUX 로드

2️⃣ perception_all.launch (인지/감지 - Ver2: BEV + Sliding Window)
↓
- lane_detect_perception_ver2.py: BEV + 슬라이딩 윈도우 → /webot/steering_offset, /webot/lane_speed
- traffic_light_detect_node.py: 신호등 감지 → /webot/traffic_light/state
- obstacle_avoid_perception.py: 장애물 감지
- crosswalk_perception_node.py: 횡단보도 감지
- aruco_detector_node.py: ArUco 마커 감지

3️⃣ decision_all.launch (main_node.py - Mission Orchestrator)
↓
perception 토픽 구독 ← perception_all.launch가 발행한 데이터
↓
main_node.py (State Machine)
├─ mission_lane.py (Simple Proportional Control)
├─ mission_traffic_light.py (Red Light Stop)
├─ mission_crosswalk.py (Crosswalk Handling)
├─ mission_obstacle.py (Obstacle Avoidance)
└─ mission_parking.py (Parking)
↓
우선순위: PARKING > TRAFFIC_LIGHT > CROSSWALK > OBSTACLE > LANE
↓
/low_level/ackermann_cmd_mux/input/navigation 발행 (모터 제어)
↓
모터 제어!