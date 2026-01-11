# Only One Webot Parking - 통합 자율주행 노드

## 인하대학교 캡스톤2 webot 로봇을 활용한 자율주행 로봇 알고리즘 제작
- 모든 미션 주행 완료
- kante2 1인 개발

## 📋 개요

`only_one_webot_parking.py`는 WEGO 자율주행 차량을 위한 통합 제어 노드입니다. 차선 추종, 장애물 회피, ArUco 마커 기반 주차 및 회전 미션을 하나의 노드로 처리합니다.

**주요 기능:**
- 🚗 HSV 기반 차선 검출 (흰색/노란색)
- 🎯 Stanley 제어 기반 조향
- 🅿️ ArUco 마커 기반 자동 주차
- ↩️ 좌회전/우회전 하드코딩 회피 동작
- 🚧 LiDAR 기반 장애물 회피
- ⏱️ 시작 대기 시간 (8초)

---

## 🔧 시스템 구성

### **입력 토픽**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/usb_cam/image_raw/compressed` | CompressedImage | 카메라 영상 |
| `/scan` | LaserScan | LiDAR 데이터 |
| `/webot/aruco/marker_info` | Float32MultiArray | ArUco 마커 정보 (ID, 거리) |
| `/webot/traffic_stop` | Bool | 외부 정지 신호 |

### **출력 토픽**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/low_level/ackermann_cmd_mux/input/navigation` | AckermannDriveStamped | 속도/조향 제어 명령 |
| `/binary_LaneFollow` | Image | 이진화 이미지 (디버그) |
| `/sliding_window_debug` | Image | 슬라이딩 윈도우 디버그 |
| `/lane_follow_debug` | Image | 차선 추종 디버그 |

---

## 🎯 미션 우선순위

노드는 다음 우선순위로 동작합니다:

```
1️⃣ 시작 대기 (8초)
    ↓
2️⃣ 주차 미션 (ArUco ID 0)
    ↓
3️⃣ 회전 미션 (ArUco ID 1, 4)
    ↓
4️⃣ 장애물 회피
    ↓
5️⃣ 차선 추종
```

---

## 🅿️ ArUco 마커 미션

### **마커 ID 0: 주차**
- **트리거 거리**: 0.5m 이내
- **동작 시퀀스**:
  1. `STOP_BEFORE`: 1초 정지
  2. `FORWARD`: 1.5초 전진
  3. `STOP_ALIGN`: 1초 정지
  4. `REVERSE_RIGHT`: 3초 후진 + 우회전
  5. `REVERSE_LEFT`: 3초 후진 + 좌회전
  6. `FORWARD_STRAIGHT`: 1초 전진
  7. `DONE`: 1초 후 노드 종료

**파라미터**:
```yaml
parking_trigger_distance: 0.5       # 트리거 거리 (m)
parking_speed_slow: 0.35            # 전진 속도 (m/s)
parking_speed_reverse: -0.35        # 후진 속도 (m/s)
parking_steering_angle: 0.5         # 조향각 (rad)
parking_forward_time: 1.5           # 전진 시간 (s)
parking_reverse_right_time: 3.0     # 우회전 후진 시간 (s)
parking_reverse_left_time: 3.0      # 좌회전 후진 시간 (s)
parking_final_forward_time: 1.0     # 마지막 전진 시간 (s)
```

---

### **마커 ID 1: 좌회전 회피**
- **트리거 거리**: 0.5m 이내
- **동작 시퀀스**:
  1. **TURN_FIRST**: 좌회전 2초 (-0.5 rad)
  2. **TURN_SECOND**: 우회전 2초 (+0.5 rad)
  3. **DONE**: 정상 주행 복귀

**타이밍**: 총 4.0초 (대칭)

---

### **마커 ID 4: 우회전 회피**
- **트리거 거리**: 0.5m 이내
- **동작 시퀀스**:
  1. **TURN_FIRST**: 우회전 1.8초 (+0.5 rad)
  2. **TURN_SECOND**: 좌회전 3.7초 (-0.5 rad)
  3. **DONE**: 정상 주행 복귀

**타이밍**: 총 5.5초 (비대칭)

**파라미터**:
```yaml
turn_speed: 0.35                      # 회전 속도 (m/s)
turn_steering_angle: 0.5              # 조향각 (rad, 약 28.6도)
turn_duration: 2.0                    # ID 1 첫 번째 시간 (s)
turn_right_first_duration: 1.8        # ID 4 첫 번째 시간 (s)
turn_duration_second: 2.0             # ID 1 두 번째 시간 (s)
turn_right_second_duration: 3.7       # ID 4 두 번째 시간 (s)
turn_left_marker_id: 1
turn_right_marker_id: 4
```

---

## 🚧 장애물 회피

### **LiDAR 기반 감지**
- **ROI 각도**: ±35° (전방)
- **정지 거리**: 0.2m
- **안전 거리**: 0.5m
- **섹터 분석**: 9개 섹터로 분할

### **회피 전략**
1. **노란색 차선 감지 시**: 섹터 분석 후 회피 시도
2. **노란색 미감지 시**: 0.2m 이내 장애물 시 정지
3. **거리 기반 속도 조절**: 0.2-0.5m 구간에서 속도 점진적 감소 (0.15-0.4 m/s)

**파라미터**:
```yaml
obstacle_stop_distance: 0.2      # 정지 거리 (m)
obstacle_safe_distance: 0.5      # 안전 거리 (m)
lidar_roi_angle: 35.0            # ROI 각도 (degrees)
num_sectors: 9                   # 섹터 개수
avoidance_gain: 0.6              # 회피 게인
```

---

## 🛣️ 차선 추종

### **차선 검출**
- **흰색 차선**: HSV 범위 기반 필터링
- **노란색 차선**: 장애물 회피 판단용
- **ROI**: 화면 하단 310-480px
- **워핑**: 역원근 변환으로 Bird's Eye View 생성

### **제어 방식**
- **Stanley 제어**: 횡방향 오차 + 헤딩 오차
- **슬라이딩 윈도우**: 차선 중심점 탐색
- **속도 제어**: Dynamic Reconfigure로 실시간 조정

**파라미터**:
```yaml
white_lower: [0, 0, 180]        # 흰색 HSV 하한
white_upper: [180, 40, 255]     # 흰색 HSV 상한
yellow_lower: [20, 40, 100]     # 노란색 HSV 하한
yellow_upper: [38, 110, 255]    # 노란색 HSV 상한
```

---

## ⏱️ 시작 대기 기능

노드 시작 후 **8초간 정지** 후 주행 시작:

```python
startup_delay: 8.0  # 대기 시간 (초)
```

**동작**:
1. 0-8초: 정지 상태 (speed=0.0)
2. 8초 후: "Starting to drive..." 로그 출력
3. 정상 주행 시작

---

## 🎛️ Launch 파일 예제

```xml
<launch>
  <node pkg="decision_wego" type="only_one_webot_parking.py" 
        name="only_one_webot_parking" output="screen">
    
    <!-- 기본 설정 -->
    <param name="publish_cmd_vel" value="true"/>
    <param name="debug_view" value="true"/>
    <param name="startup_delay" value="8.0"/>
    
    <!-- 주차 파라미터 -->
    <param name="parking_trigger_distance" value="0.5"/>
    <param name="parking_target_marker_id" value="0"/>
    <param name="parking_speed_slow" value="0.35"/>
    <param name="parking_speed_reverse" value="-0.35"/>
    <param name="parking_steering_angle" value="0.5"/>
    
    <!-- 회전 파라미터 -->
    <param name="turn_left_marker_id" value="1"/>
    <param name="turn_right_marker_id" value="4"/>
    <param name="turn_speed" value="0.35"/>
    <param name="turn_steering_angle" value="0.5"/>
    <param name="turn_right_first_duration" value="1.8"/>
    <param name="turn_right_second_duration" value="3.7"/>
    
    <!-- 장애물 회피 -->
    <param name="obstacle_stop_distance" value="0.2"/>
    <param name="obstacle_safe_distance" value="0.5"/>
    <param name="lidar_roi_angle" value="35.0"/>
    <param name="num_sectors" value="9"/>
    <param name="avoidance_gain" value="0.6"/>
    
  </node>
</launch>
```

---

## 🔍 디버깅

### **로그 메시지**

#### **시작 대기**
```
[Startup] Waiting... 5.0/8.0s
[STARTUP] 8 seconds delay completed! Starting to drive...
```

#### **ArUco 마커 감지**
```
[ArUco Debug] Detected: ID1:0.28m, ID4:1.50m
[PARKING TRIGGERED] ArUco ID 0 detected at 0.48m!
[LEFT TURN TRIGGERED] ArUco ID 1 detected at 0.45m!
[RIGHT TURN TRIGGERED] ArUco ID 4 detected at 0.42m!
```

#### **미션 진행**
```
[Parking] State: IDLE -> STOP_BEFORE
[Parking] FORWARD 1.2/1.5s
[Turn LEFT] TURN_FIRST (Left) 1.5/2.0s
[Turn RIGHT] TURN_SECOND (Left) 2.0/3.7s
```

#### **장애물 감지**
```
[Obstacle] Front: 0.35m | Stop: False | Yellow: True
[EMERGENCY] Very close obstacle (0.15m) - STOP!
```

### **시각화 토픽**
```bash
# 이진화 이미지
rqt_image_view /binary_LaneFollow

# 슬라이딩 윈도우
rqt_image_view /sliding_window_debug

# 차선 추종 전체
rqt_image_view /lane_follow_debug
```

---

## 🚀 실행 방법

```bash
# 1. 노드 실행
roslaunch decision_wego parking_with_aruco.launch

# 2. 파라미터 확인
rosparam list | grep only_one_webot

# 3. 실시간 속도 조정 (Dynamic Reconfigure)
rosrun rqt_reconfigure rqt_reconfigure
```

---

## 📊 주요 상태 변수

| 변수 | 타입 | 설명 |
|------|------|------|
| `parking_active` | bool | 주차 미션 활성화 |
| `parking_state` | string | 주차 단계 (IDLE, STOP_BEFORE, FORWARD, etc.) |
| `turn_active` | bool | 회전 미션 활성화 |
| `turn_direction` | string | 회전 방향 (LEFT, RIGHT) |
| `turn_state` | string | 회전 단계 (IDLE, TURN_FIRST, TURN_SECOND, DONE) |
| `stop_flag` | bool | 정지 플래그 |
| `is_ready_to_drive` | bool | 시작 대기 완료 |
| `yellow_detected` | bool | 노란색 차선 감지 |
| `min_front_distance` | float | 전방 최소 거리 (m) |

---

## ⚙️ 튜닝 가이드

### **주차가 잘 안될 때**
```yaml
# 속도 증가
parking_speed_slow: 0.4
parking_speed_reverse: -0.4

# 후진 시간 조정
parking_reverse_right_time: 3.5
parking_reverse_left_time: 3.5
```

### **회전이 차선을 벗어날 때**
```yaml
# 마커 ID 4 타이밍 조정
turn_right_first_duration: 1.5   # 줄이기
turn_right_second_duration: 3.5  # 줄이기

# 조향각 감소
turn_steering_angle: 0.45
```

### **장애물 회피가 너무 민감할 때**
```yaml
# 정지 거리 감소
obstacle_stop_distance: 0.15

# 회피 게인 감소
avoidance_gain: 0.4
```

---

## 🐛 알려진 이슈

1. **주차 후 노드 종료**: 정상 동작입니다. 주차 완료 시 `rospy.signal_shutdown()` 호출
2. **회전 미션 중 다른 마커 감지**: 미션 중에는 새로운 마커 무시 (`parking_active` or `turn_active` 체크)
3. **시작 대기 중 정지**: 8초 대기는 정상 동작입니다.

---

## 📝 버전 정보

**최종 업데이트**: 2026-01-12
**주요 변경사항**:
- ✅ 시작 8초 대기 기능 추가
- ✅ 빨간색 신호 정지 기능 제거
- ✅ 마커 ID 2 → 4로 변경
- ✅ ID 4 타이밍 비대칭 조정 (1.8s + 3.7s)
- ✅ 장애물 거리 기반 속도 조절 추가

---

## 📞 문의

- **패키지**: `decision_wego`
- **노드**: `only_one_webot_parking.py`
- **위치**: `/home/wego/wego25_winter_ws/src/decision_wego/scripts/wego_solo/`
