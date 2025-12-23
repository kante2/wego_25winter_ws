# Topic Mapping: perception_25 / decision_25 와 inha25-winter-ros 호환성

## 개요
perception_25와 decision_25 패키지의 토픽 이름을 inha25-winter-ros와 호환되도록 통일했습니다.

---

## 1. Lane (차선 인식 & 제어)

### perception_25: lane_perception_node.cpp
**입력 (Subscription):**
- `/usb_cam/image_rect_color` (sensor_msgs/Image): 카메라 영상

**출력 (Publication):**
- `/webot/lane_center` (geometry_msgs/PointStamped): 차선 중심 픽셀 좌표 (x값)
- `/webot/lane_curvature` (std_msgs/Float32): 도로 곡률 추정값
- `/webot/lane_color` (geometry_msgs/PointStamped): 차선 색상 감지 결과

### decision_25: lane_decision_node.cpp
**입력 (Subscription):**
- `/webot/lane_center` (geometry_msgs/PointStamped): 차선 중심
- `/webot/lane_curvature` (std_msgs/Float32): 곡률
- `/webot/lane_color` (geometry_msgs/PointStamped): 색상

**출력 (Publication):**
- `/commands/motor/speed` (std_msgs/Float64): 모터 속도 명령
- `/commands/servo/position` (std_msgs/Float64): 서보 위치 (0.0-1.0)

---

## 2. Obstacle Avoidance (장애물 회피)

### perception_25: lidar_obstacle_perception_node.cpp
**입력 (Subscription):**
- `/scan` (sensor_msgs/LaserScan): LiDAR 스캔

**출력 (Publication):**
- `/webot/obstacle/best_gap` (geometry_msgs/PointStamped): 최적 갭 각도 (x값)
- `/webot/obstacle/min_distance` (std_msgs/Float32): 최소 장애물 거리
- `/webot/obstacle/has_obstacle` (std_msgs/Int32): 장애물 감지 플래그

### decision_25: obstacle_avoid_decision_node.cpp
**입력 (Subscription):**
- `/webot/obstacle/best_gap` (geometry_msgs/PointStamped)
- `/webot/obstacle/min_distance` (std_msgs/Float32)
- `/webot/obstacle/has_obstacle` (std_msgs/Int32)

**출력 (Publication):**
- `/commands/motor/speed` (std_msgs/Float64)
- `/commands/servo/position` (std_msgs/Float64)
- `/webot/obstacle_avoid/state` (std_msgs/String)
- `/webot/obstacle_avoid/is_avoiding` (std_msgs/Bool)

---

## 3. Traffic Light (신호등 제어)

### perception_25: traffic_light_perception_node.cpp
**입력 (Subscription):**
- `/usb_cam/image_rect_color` (sensor_msgs/Image): 카메라 영상

**출력 (Publication):**
- `/webot/traffic_light/state` (std_msgs/String): "RED" / "GREEN" / "UNKNOWN"

### decision_25: traffic_light_decision_node.cpp
**입력 (Subscription):**
- `/webot/traffic_light/state` (std_msgs/String)
- `/webot/steering_offset` (std_msgs/Float32): 차선 조향각
- `/webot/lane_speed` (std_msgs/Float32): 차선 속도

**출력 (Publication):**
- `/commands/motor/speed` (std_msgs/Float64)
- `/commands/servo/position` (std_msgs/Float64)
- `/webot/traffic_stop` (std_msgs/Bool)
- `/webot/traffic_passed` (std_msgs/Bool)

---

## 4. Crosswalk (횡단보도 인식)

### perception_25: crosswalk_perception_node.cpp
**입력 (Subscription):**
- `/usb_cam/image_rect_color` (sensor_msgs/Image)

**출력 (Publication):**
- `/webot/crosswalk/detected` (std_msgs/Bool): 횡단보도 감지 여부
- `/webot/crosswalk/stripe_ratio` (std_msgs/Float32): 줄무늬 비율

### decision_25: crosswalk_decision_node.cpp
**입력 (Subscription):**
- `/webot/crosswalk/detected` (std_msgs/Bool)
- `/webot/crosswalk/stripe_ratio` (std_msgs/Float32)
- `/webot/steering_offset` (std_msgs/Float32)
- `/webot/lane_speed` (std_msgs/Float32)

**출력 (Publication):**
- `/commands/motor/speed` (std_msgs/Float64)
- `/commands/servo/position` (std_msgs/Float64)
- `/webot/crosswalk/stop` (std_msgs/Bool)
- `/webot/crosswalk/state` (std_msgs/String)

---

## 5. inha25-winter-ros와의 호환성

### lane_detect_node.py (Python)
**발행 토픽:**
- `/webot/steering_offset` (Float32) ← **lane_decision_node이 구독**
- `/webot/lane_speed` (Float32) ← **obstacle/traffic/crosswalk decision이 구독**
- `/webot/lane_center_x` (Int32)
- `/webot/traffic_stop` (Bool) ← traffic_light_decision_node이 발행

### obstacle_avoid_node.py (Python)
**구독 토픽:**
- `/webot/steering_offset` (Float32)
- `/webot/lane_speed` (Float32)

---

## 6. 명령 체인 (Command Chain)

```
perception_25 (Sensing)
├── lane_perception_node → /webot/lane_*
├── lidar_obstacle_perception_node → /webot/obstacle/*
├── traffic_light_perception_node → /webot/traffic_light/state
└── crosswalk_perception_node → /webot/crosswalk/*

⬇️

decision_25 (Control)
├── lane_decision_node → /commands/motor/speed, /commands/servo/position
├── obstacle_avoid_decision_node → /commands/motor/speed, /commands/servo/position
├── traffic_light_decision_node → /commands/motor/speed, /commands/servo/position
└── crosswalk_decision_node → /commands/motor/speed, /commands/servo/position

⬇️

Hardware (Motor + Servo)
```

---

## 7. 주의사항

### Topic 이름 규칙
- **perception**: `/webot/` 접두사 (inha25와 통일)
- **decision**: 입출력 모두 동일한 `/webot/*` 사용
- **commands**: `/commands/motor/speed`, `/commands/servo/position` (고정)

### 다중 Decision Node 실행 시
여러 decision 노드가 동시에 `/commands/motor/speed`와 `/commands/servo/position`을 발행하므로, 
**`ackermann_cmd_mux`를 통한 우선순위 관리가 필수**입니다.

### Parameter 파일 설정
각 노드의 `params` 또는 launch 파일에서 다음을 설정할 수 있습니다:
```yaml
lane_decision_node:
  topic_center_point: "/webot/lane_center"
  topic_curvature_center: "/webot/lane_curvature"
  topic_center_color: "/webot/lane_color"
  motor_topic: "/commands/motor/speed"
  servo_topic: "/commands/servo/position"
```

---

## 8. 테스트 명령어

```bash
# 토픽 목록 확인
rostopic list | grep webot

# 특정 토픽 모니터링
rostopic echo /webot/lane_center
rostopic echo /webot/obstacle/best_gap
rostopic echo /webot/traffic_light/state
rostopic echo /webot/crosswalk/detected

# 토픽 발행 빈도 확인
rostopic hz /webot/lane_center
```

---

## 수정 이력

| 파일 | 토픽 변경 | 날짜 |
|------|---------|------|
| lane_perception_node.cpp | `/perception/*` → `/webot/lane_*` | 2025-12-24 |
| lidar_obstacle_perception_node.cpp | `/perception/lidar/*` → `/webot/obstacle/*` | 2025-12-24 |
| traffic_light_perception_node.cpp | `/perception/traffic_light/state` → `/webot/traffic_light/state` | 2025-12-24 |
| crosswalk_perception_node.cpp | `/perception/crosswalk/*` → `/webot/crosswalk/*` | 2025-12-24 |
| lane_decision_node.cpp | 구독 토픽 업데이트 | 2025-12-24 |
| obstacle_avoid_decision_node.cpp | 구독 토픽 업데이트 | 2025-12-24 |
| traffic_light_decision_node.cpp | 구독 토픽 업데이트 | 2025-12-24 |
| crosswalk_decision_node.cpp | 구독 토픽 업데이트 | 2025-12-24 |
