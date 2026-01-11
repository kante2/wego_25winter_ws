# WEGO 자율주행 시스템 (Winter 2025)

## 📋 빠른 시작

### 3단계 실행 순서

```bash

# ** 
# 1단계: 기본 하드웨어 설정 (카메라, LiDAR, 모터 제어)
source devel/setup.bash
roslaunch wego bringup.launch

# 2단계: 인지(Perception) 노드 실행
source devel/setup.bash
roslaunch perception_wego perception_all.launch

# 3단계: 의사결정(Decision) 메인 노드 실행
source devel/setup.bash
roslaunch decision_wego decision_all.launch

# 4
source devel/setup.bash
roslaunch decision_wego only_one_webot.


# 5. ** 
source devel/setup.bash
roslaunch decision_wego parking_with_aruco.launch

# 6. DEBUGGING
source devel/setup.bash
rosrun decision_wego yellow_corn_debugging.py

```

# roslaunch decision_wego only_one_webot.launch

---

## 🏗️ 시스템 구조 (Architecture)

```
┌──────────────────────────────────────────────────────────────┐
│                 WEGO Autonomous Vehicle                       │
│                                                               │
│  Sensors (Camera, LiDAR)                                      │
│         │                                                     │
│         ↓                                                     │
│  ┌──────────────────────┐                                    │
│  │  PERCEPTION_WEGO     │  (Sensor Processing)               │
│  │  - Lane Detection    │                                    │
│  │  - Traffic Light     │                                    │
│  │  - Crosswalk         │                                    │
│  │  - Obstacle Avoid    │                                    │
│  │  - ArUco Markers     │                                    │
│  └──────────────────────┘                                    │
│         │                                                     │
│         ↓ (/webot/*)                                          │
│  ┌──────────────────────┐                                    │
│  │  DECISION_WEGO       │  (Decision & Control)              │
│  │  - Main Orchestrator │                                    │
│  │  - Lane Following    │                                    │
│  │  - Obstacle Avoidance│                                    │
│  │  - Traffic Light     │                                    │
│  │  - Crosswalk         │                                    │
│  │  - Parking           │                                    │
│  └──────────────────────┘                                    │
│         │                                                     │
│         ↓ (/ackermann_cmd_mux/input/navigation)              │
│  Vehicle Control (Motor, Servo)                              │
└──────────────────────────────────────────────────────────────┘
```

---

## 📦 패키지 구조

```
src/
├── decision_wego/          # 의사결정 계층
│   ├── scripts/
│   │   ├── main_node.py           # 메인 오케스트레이터
│   │   ├── dh_lanefollow.py        # 차선 추종 노드
│   │   ├── mission_lane.py         # 차선 미션
│   │   ├── mission_obstacle.py     # 장애물 회피 미션
│   │   ├── mission_traffic_light.py # 신호등 미션
│   │   ├── mission_crosswalk.py    # 횡단보도 미션
│   │   └── mission_parking.py      # 주차 미션
│   └── launch/
│
├── perception_wego/        # 인지 계층
│   ├── scripts/
│   │   ├── lane_detect_perception.py      # 차선 검출
│   │   ├── traffic_light_detect_node.py   # 신호등 검출
│   │   ├── crosswalk_perception_node.py   # 횡단보도 검출
│   │   ├── obstacle_avoid_perception.py   # 장애물 감지
│   │   └── aruco_detector_node.py         # ArUco 마커 검출
│   └── launch/
│
├── wego_cfg/               # 동적 재구성 설정
│   └── cfg/
│       └── LaneDetect.cfg  # Lane detection parameters
│
├── racecar/                # 로봇 플랫폼 패키지
├── usb_cam/                # USB 카메라 드라이버
├── rplidar_ros/            # RPLiDAR 드라이버
├── razor_imu_9dof/         # IMU 센서 드라이버
├── vesc/                   # VESC 모터 제어기
└── inha25-winter-ros/      # 기타 유틸리티
```

---

## 🔍 PERCEPTION_WEGO 패키지

**목적:** 카메라 및 LiDAR을 사용하여 주변 환경 인식

### 5개 인지 노드

#### 1. **lane_detect_perception.py**
- **입력:** Camera `/usb_cam/image_raw/compressed`
- **기능:**
  - HSV 색상 공간 필터링 (흰색 차선)
  - Bird's Eye View (BEV) 원근 변환
  - 슬라이딩 윈도우를 통한 차선 중심선 검출
  - 곡률 계산
- **출력 토픽:**
  - `/webot/lane_center_x` (Int32) - 차선 중심 X픽셀 위치
  - `/webot/lane_detect/image` (Image) - 디버그 이미지
- **동적 재구성:** `masked_pixel`, HSV 임계값
- **주기:** 30 Hz

#### 2. **traffic_light_detect_node.py**
- **입력:** Camera `/usb_cam/image_raw/compressed`
- **기능:**
  - ROI (관심 영역) 정의
  - HSV 색상 필터링 (빨강, 초록)
  - 원형 윤곽 검출
  - 원형도 검증
- **출력 토픽:**
  - `/webot/traffic_light/state` (String) - "RED", "GREEN", "UNKNOWN"
- **주기:** 30 Hz

#### 3. **crosswalk_perception_node.py**
- **입력:** Camera `/usb_cam/image_raw/compressed`
- **기능:**
  - 사다리꼴 ROI 정의
  - 흰색/노란색 줄무늬 검출 (Canny edge + Hough lines)
  - 줄무늬 비율 계산
- **출력 토픽:**
  - `/webot/crosswalk/detected` (Bool) - 횡단보도 감지 여부
  - `/webot/crosswalk/stripe_ratio` (Float32) - 줄무늬 커버율
- **주기:** 30 Hz

#### 4. **obstacle_avoid_perception.py**
- **입력:** LiDAR `/scan`
- **기능:**
  - 극좌표 → 직교좌표 변환
  - 점군 필터링 및 장애물 감지
  - 최적 간격(gap) 계산
- **출력 토픽:**
  - `/webot/obstacle/best_gap` (Float32) - 최적 통과 각도
  - `/webot/obstacle/min_distance` (Float32) - 최단 거리
  - `/webot/obstacle/has_obstacle` (Bool) - 장애물 존재
- **주기:** 20-30 Hz

#### 5. **aruco_detector_node.py**
- **입력:** Camera `/usb_cam/image_raw/compressed`
- **기능:**
  - ArUco 마커 검출
  - 마커 ID 및 포즈 추정
  - 주차 및 위치 결정용
- **출력 토픽:**
  - `/webot/aruco/markers` (MarkerArray)
- **주기:** 30 Hz

---

## 🎮 DECISION_WEGO 패키지

**목적:** 인지 정보를 기반으로 차량 제어 명령 생성

### 메인 노드 + 미션 노드 구조

#### **main_node.py** (메인 오케스트레이터)
- **목적:** 상태 머신 기반 우선순위 관리
- **상태:**
  - `STATE_LANE` (기본) - 차선 추종
  - `STATE_TRAFFIC_LIGHT` - 신호등 대기
  - `STATE_CROSSWALK` - 횡단보도 정지
  - `STATE_OBSTACLE` - 장애물 회피
- **우선순위:** 횡단보도 > 신호등 > 장애물 > 차선
- **제어 루프:** 30 Hz
- **출력:**
  - `/low_level/ackermann_cmd_mux/input/navigation` (AckermannDriveStamped)

#### **dh_lanefollow.py** (차선 추종 제어)
- **입력:**
  - `/usb_cam/image_raw/compressed` - 카메라 이미지
  - `/webot/traffic_stop` (Bool) - 정지 신호
- **기능:**
  - HSV 기반 흰색 차선 검출
  - 슬라이딩 윈도우 알고리즘
  - Stanley 조향 제어
  - 동적 재구성 지원
- **출력:**
  - `/webot/steering_offset` (Float32) - 조향각
  - `/webot/lane_speed` (Float32) - 속도
  - `/webot/lane_center_x` (Int32) - 차선 중심
- **동적 재구성:**
  - `base_speed` - 기본 속도 (0.0-1.0 m/s)
  - `k` - 비례 게인 (0.0-0.05)
  - `yaw_k` - 회전 게인 (0.0-1.0)
  - `masked_pixel` - 마스킹 픽셀 (0-70)
- **주기:** 30 Hz

#### **mission_lane.py** (차선 미션)
- **기능:**
  - 차선 중심 정보 수신
  - Stanley 제어 기반 조향 계산
  - 오류 기반 속도 조정
- **입력:**
  - `/webot/lane_center_x` (Int32)
- **동적 재구성:** `base_speed`, `k`, `yaw_k`
- **출력:** (step() 함수)
  - 속도, 조향각, 디버그 정보

#### **mission_traffic_light.py** (신호등 미션)
- **기능:**
  - 신호등 상태 감시
  - RED → 정지 (speed=0)
  - GREEN → 진행 (lane speed)
- **입력:**
  - `/webot/traffic_light/state` (String)
- **우선순위:** 높음 (규칙 준수)

#### **mission_crosswalk.py** (횡단보도 미션)
- **기능:**
  - 횡단보도 감지시 감속 및 정지
  - 보행자 안전 확보
  - 4-상태 FSM:
    - CLEAR → DETECTED → STOPPED → CROSSING_WAIT
- **입력:**
  - `/webot/crosswalk/detected` (Bool)
  - `/webot/crosswalk/stripe_ratio` (Float32)
- **우선순위:** 최고 (보행자 안전)

#### **mission_obstacle.py** (장애물 회피 미션)
- **기능:**
  - Gap-following 알고리즘
  - 최적 간격으로 조향
  - 회피 중 감속
- **입력:**
  - `/webot/obstacle/best_gap` (Float32)
  - `/webot/obstacle/has_obstacle` (Bool)
- **상태 FSM:**
  - TOO_CLOSE → AVOIDING → CLEAR
- **우선순위:** 중간 (충돌 회피)

#### **mission_parking.py** (주차 미션)
- **기능:**
  - ArUco 마커 기반 위치 결정
  - 자동 주차 제어
- **입력:**
  - `/webot/aruco/markers` (MarkerArray)
- **특수 모드:** 주차 미션 활성시에만 작동

---

## 📡 토픽 네이밍 컨벤션

### Perception 출력 (`/webot/*`)
```
차선:        /webot/lane_center_x, /webot/lane_detect/*
신호등:      /webot/traffic_light/state
횡단보도:    /webot/crosswalk/detected, /webot/crosswalk/stripe_ratio
장애물:      /webot/obstacle/*, /webot/obstacle/best_gap
ArUco:       /webot/aruco/markers
```

### 제어 출력 (`/low_level/ackermann_cmd_mux/input/navigation`)
```
AckermannDriveStamped
  ├── drive.speed (m/s)
  └── drive.steering_angle (rad)
```

### 디버그 토픽
```
/binary_LaneFollow       (Image) - 이진화 이미지
/sliding_window_debug    (Image) - 슬라이딩 윈도우 시각화
/lane_follow_debug       (Image) - 차선 검출 결과
```

---

## ⚙️ 동적 재구성 (Dynamic Reconfigure)

### wego_cfg/LaneDetect.cfg
```yaml
# HSV 필터 (흰색 차선)
hsv_h_low: 0        # Hue 하한
hsv_h_high: 180
hsv_s_low: 0        # Saturation 하한
hsv_s_high: 40
hsv_v_low: 180      # Value 하한
hsv_v_high: 255

# 제어 파라미터
base_speed: 0.3     # m/s (0.0-1.0)
k: 0.005            # 비례 게인
yaw_k: 1.0          # 회전 게인
masked_pixel: 30    # 중앙 마스크 폭
```

### 실행 중 파라미터 변경
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

---

## 🚀 실행 모드

### 1. 완전 자동 모드 (권장)
```bash
roslaunch decision_wego decision_all.launch
# 또는
roslaunch decision_25 complete_25.launch  # 구 구조
```
- 모든 perception + decision 노드 자동 시작
- main_node가 상태 머신 관리

### 2. 개별 미션 테스트
```bash
roslaunch perception_wego perception_all.launch &
roslaunch decision_wego decision_all.launch launch_individual_nodes:=true
```
- main_node 비활성화
- 각 미션 노드 독립 실행
- 토픽 에코로 동작 확인 가능

### 3. Perception만 실행
```bash
roslaunch perception_wego perception_all.launch
```
- 센서 처리만 실행
- `/webot/*` 토픽 발행
- 차량 제어 없음

### 4. 특정 노드만 실행
```bash
rosrun perception_wego lane_detect_perception.py
rosrun decision_wego dh_lanefollow.py
rosrun decision_wego mission_lane.py
```

---

## 🔧 설치 및 빌드

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 의존성 설치
```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 📊 성능 메트릭

| 노드 | 입력 주기 | 처리 시간 | 출력 주기 |
|------|---------|---------|---------|
| lane_detect_perception | 30 Hz | ~15ms | 30 Hz |
| traffic_light_detect | 30 Hz | ~10ms | 30 Hz |
| crosswalk_perception | 30 Hz | ~20ms | 30 Hz |
| obstacle_perception | 20 Hz | ~8ms | 30 Hz |
| aruco_detector | 30 Hz | ~10ms | 30 Hz |
| dh_lanefollow | 30 Hz | ~5ms | 30 Hz |
| mission_lane | 30 Hz | ~2ms | - |
| main_node | 30 Hz | ~3ms | 30 Hz |

**전체 지연 시간:** ~50-80ms (센서 입력 → 차량 제어)

---

## 🐛 디버깅

### 1. 차선 검출 확인
```bash
rosrun rqt_image_view rqt_image_view &
# 토픽: /webot/lane_detect/image, /binary_LaneFollow, /sliding_window_debug
```

### 2. 신호등 상태 확인
```bash
rostopic echo /webot/traffic_light/state
```

### 3. 주행 명령 확인
```bash
rostopic echo /low_level/ackermann_cmd_mux/input/navigation
```

### 4. 로그 기록 및 재생
```bash
# 녹화
rosbag record /webot/* /low_level/ackermann_cmd_mux/input/navigation -o mission_log.bag

# 재생
rosbag play mission_log.bag --clock
```

### 5. 동적 파라미터 변경
```bash
# 기본 속도 변경
rosparam set /lane_detect_perception/base_speed 0.5

# 또는 GUI로
rosrun rqt_reconfigure rqt_reconfigure
```

---

## 📝 소스 코드 위치

| 노드 | 경로 |
|------|------|
| lane_detect_perception | `src/perception_wego/scripts/lane_detect_perception.py` |
| traffic_light_detect | `src/perception_wego/scripts/traffic_light_detect_node.py` |
| crosswalk_perception | `src/perception_wego/scripts/crosswalk_perception_node.py` |
| obstacle_perception | `src/perception_wego/scripts/obstacle_avoid_perception.py` |
| aruco_detector | `src/perception_wego/scripts/aruco_detector_node.py` |
| dh_lanefollow | `src/decision_wego/scripts/dh_lanefollow.py` |
| mission_lane | `src/decision_wego/scripts/mission_lane.py` |
| mission_traffic_light | `src/decision_wego/scripts/mission_traffic_light.py` |
| mission_crosswalk | `src/decision_wego/scripts/mission_crosswalk.py` |
| mission_obstacle | `src/decision_wego/scripts/mission_obstacle.py` |
| mission_parking | `src/decision_wego/scripts/mission_parking.py` |
| main_node | `src/decision_wego/scripts/main_node.py` |

---

## 🎯 주요 기능

### ✅ 구현됨
- [x] 차선 검출 및 추종
- [x] 신호등 인식 및 준수
- [x] 횡단보도 감지 및 정지
- [x] LiDAR 기반 장애물 회피
- [x] ArUco 마커 검출
- [x] 동적 재구성 지원
- [x] 우선순위 기반 상태 관리

### 🔄 계획 중
- [ ] 주차 미션 완성
- [ ] 센서 융합 (Camera + LiDAR)
- [ ] 경로 계획 통합
- [ ] 머신러닝 기반 인식 개선
- [ ] 다중 에이전트 협력

---

## 📚 참고 문서

- [ARCHITECTURE_EXPLANATION.md](ARCHITECTURE_EXPLANATION.md) - 상세 아키텍처
- [TRAFFIC_LIGHT_INTEGRATION.md](TRAFFIC_LIGHT_INTEGRATION.md) - 신호등 통합
- [ROUNDABOUT_IMPLEMENTATION.md](ROUNDABOUT_IMPLEMENTATION.md) - 회전교차로
- [TOPIC_MAPPING.md](TOPIC_MAPPING.md) - 토픽 매핑
- [VALIDATION_CHECKLIST.md](VALIDATION_CHECKLIST.md) - 검증 체크리스트
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - 빠른 참조

---

## 👥 팀 정보

- **프로젝트:** WEGO 자율주행 시스템
- **기간:** Winter 2025
- **라이선스:** MIT

