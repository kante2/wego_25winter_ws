
'''
WEGO 실행 순서
1단계: 기본 설정 (필수)

roslaunch wego bringup.launch


저수준 드라이버 로드 (카메라, LiDAR, 모터 제어)
기본 하드웨어 초기화


----------------------------

2단계: 인지(Perception) 노드 시작
roslaunch perception_wego perception_all.launch


----------------------------
3단계: 의사결정(Decision) 메인 노드 시작
roslaunch decision_wego decision_all.launch

'''




# perception_25 & decision_25 Architecture Explanation

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    WEGO25 AUTONOMOUS VEHICLE                     │
│                                                                   │
│  ┌──────────────────────────┐   ┌──────────────────────────┐   │
│  │     PERCEPTION_25        │   │      DECISION_25         │   │
│  │   (Sensor Processing)    │   │   (Control Logic)        │   │
│  └──────────────────────────┘   └──────────────────────────┘   │
│                                                                   │
│  Sensor Input ──→ Perception ──→ /webot/* Topics ──→ Decision   │
│                                                     ──→ /cmd/*   │
└─────────────────────────────────────────────────────────────────┘
```

---

## PERCEPTION_25 Package

**Purpose:** Real-time sensor processing to detect lane, traffic signals, crosswalks, and obstacles

### 4 Perception Nodes

#### 1. **lane_perception_node.cpp**
- **Input:** Camera `/usb_cam/image_rect_color`
- **Processing:**
  - HSV color space filtering for yellow lane detection (H: 18-38, S: 100-255, V: 110-230)
  - Bird's Eye View (BEV) perspective transformation
  - Lane centerline detection via image moments
  - Curvature calculation
- **Output Topics:**
  - `/webot/lane_center` (Float64) - X pixel position of lane center (0-640)
  - `/webot/lane_curvature` (Float64) - Lane curve radius (-10 to 10)
  - `/webot/lane_color` (String) - Color confirmation ("yellow", "none", etc.)
- **Rate:** 30 Hz

#### 2. **traffic_light_perception_node.cpp**
- **Input:** Camera `/usb_cam/image_rect_color`
- **Processing:**
  - ROI (Region of Interest) definition for traffic light area
  - HSV color filtering for RED (H: 0-10, 170-180) and GREEN (H: 35-85)
  - Shape detection (circular contours, min area 500 px)
  - Circularity verification (0.5 threshold)
- **Output Topics:**
  - `/webot/traffic_light/state` (String) - "RED", "GREEN", or "UNKNOWN"
- **Rate:** 30 Hz

#### 3. **crosswalk_perception_node.cpp**
- **Input:** Camera `/usb_cam/image_rect_color`
- **Processing:**
  - Trapezoid ROI (perspective-aware region selection)
  - White/yellow stripe detection via Canny edges + Hough lines
  - Stripe ratio calculation (stripe area / ROI area)
  - Threshold-based detection (stripe_ratio > 0.30)
- **Output Topics:**
  - `/webot/crosswalk/detected` (Bool) - Crosswalk detected (True/False)
  - `/webot/crosswalk/stripe_ratio` (Float32) - Stripe coverage ratio (0.0-1.0)
- **Rate:** 30 Hz

#### 4. **roundabout_perception_node.cpp** ⭐ (NEW)
- **Input:** LiDAR `/scan` (laser_link frame)
- **Processing:**
  - Polar to Cartesian coordinate conversion
  - laser_link 180° rotation correction (heading = angle + π)
  - Point cloud filtering within detection area
    - Forward range: X ∈ [0.2, 0.8]m
    - Lateral range: Y ∈ [-0.3, 0.3]m
  - Threshold detection: ≥3 points = obstacle
- **Output Topics:**
  - `/webot/roundabout/obstacle_detected` (Bool) - Obstacle ahead (True/False)
  - `/webot/roundabout/obstacle_count` (Int32) - Number of detected points
- **Rate:** 30 Hz

---

## DECISION_25 Package

**Purpose:** Decision-making and vehicle control based on perception outputs

### Architecture: Main Orchestrator + 5 Decision Nodes

```
┌──────────────────────────────────────────────────────────────────┐
│  decision_25_main (Orchestrator)                                 │
│  ├─ State Machine (FSM with 4 states)                            │
│  ├─ Priority-based decision logic                                │
│  └─ Hysteresis filtering                                         │
│                                                                   │
│  Calls appropriate step() function based on current state:        │
│  ├─ Lane_step()                                                  │
│  ├─ obstacle_avoid_decision_step()                               │
│  ├─ traffic_light_decision_step()                                │
│  └─ crosswalk_decision_step()                                    │
└──────────────────────────────────────────────────────────────────┘
```

### 5 Decision Nodes

#### 1. **lane_decision_node.cpp** (Primary Control)
- **Input Topics:**
  - `/webot/lane_center` - Lane center position (pixels)
  - `/webot/lane_curvature` - Lane curvature
  - `/webot/lane_color` - Color confirmation
- **Control Logic:**
  - PID steering control for lane following
  - Speed adaptation based on curvature (fast on straights, slow on curves)
  - Cross-track error calculation: `error_px = lane_center - bev_center (320px)`
  - Steering: `angle = atan2(error_px) * steer_gain`
  - Speed: `motor_cmd = base_speed * (1 + curvature_factor)`
- **Output Topics:**
  - `/commands/motor/speed` (Float64) - Throttle command (m/s)
  - `/commands/servo/position` (Float64) - Steering angle (0.0-1.0)
- **Rate:** 30 Hz

#### 2. **obstacle_avoid_decision_node.cpp** (Gap Following)
- **Input Topics:**
  - `/webot/obstacle/best_gap` - Best navigable gap angle (rad)
  - `/webot/obstacle/min_distance` - Closest obstacle distance (m)
  - `/webot/obstacle/has_obstacle` (Bool) - Obstacle presence
- **Control Logic:**
  - 4-state FSM:
    - `TOO_CLOSE`: Stop (obstacle < 0.2m)
    - `AVOIDING_START`: Steer toward best gap, slow speed (0.2 m/s)
    - `AVOIDING`: Continuous gap following
    - `CLEAR`: Resume to lane control (gap > safe distance)
  - Smooth steering transition with gain: `steering = best_gap * steering_gain`
- **Output Topics:**
  - `/commands/motor/speed` - Slow speed (0.2 m/s) during avoidance
  - `/commands/servo/position` - Gap-directed steering

#### 3. **traffic_light_decision_node.cpp** (Signal Compliance)
- **Input Topics:**
  - `/webot/traffic_light/state` - Signal color (RED/GREEN/UNKNOWN)
  - `/webot/steering_offset` - Current steering from lane
  - `/webot/lane_speed` - Current speed from lane
- **Control Logic:**
  - Simple state mapping:
    - `RED` → Stop: motor_cmd = 0.0
    - `GREEN` → Go: motor_cmd = lane_speed
    - `UNKNOWN` → Conservative: motor_cmd = lane_speed * 0.5
  - Maintains lane steering regardless of signal
- **Output Topics:**
  - `/commands/motor/speed` - Speed command based on signal
  - `/commands/servo/position` - Steering (from lane)

#### 4. **crosswalk_decision_node.cpp** (Pedestrian Safety)
- **Input Topics:**
  - `/webot/crosswalk/detected` (Bool) - Crosswalk ahead
  - `/webot/crosswalk/stripe_ratio` (Float32) - Stripe coverage
  - `/webot/steering_offset` - Lane steering
  - `/webot/lane_speed` - Lane speed
- **Control Logic:**
  - 4-state FSM:
    - `CLEAR`: No crosswalk detected, normal control
    - `DETECTED`: Crosswalk found, prepare to stop
    - `STOPPED`: Vehicle fully stopped (speed < 0.1 m/s)
    - `CROSSING_WAIT`: Wait for pedestrians to clear
  - Stripe ratio threshold: 0.30 (30% of ROI is white/yellow)
- **Output Topics:**
  - `/commands/motor/speed` - 0.0 when stopped
  - `/commands/servo/position` - Center steering when stopped

#### 5. **roundabout_decision_node.cpp** ⭐ (NEW - Roundabout Safety)
- **Input Topics:**
  - `/webot/roundabout/obstacle_detected` (Bool) - Forward obstacle
  - `/webot/roundabout/obstacle_count` (Int32) - Obstacle point count
- **Control Logic:**
  - Simple reactive control:
    - `obstacle_detected = true` → STOPPED: Stop immediately (motor=0, servo=center)
    - `obstacle_detected = false` → CLEAR: No output (yield to main_node)
  - Used for sudden collision avoidance in roundabout
- **Output Topics:**
  - `/commands/motor/speed` - 0.0 when obstacle detected
  - `/commands/servo/position` - Center position
  - `/webot/roundabout/state` (String) - "STOPPED" or "CLEAR" for debugging
- **Rate:** 30 Hz
- **Note:** Does NOT interfere with normal control; only publishes when obstacle detected

#### 6. **main_node.cpp** (Orchestrator - Decision FSM)
- **Purpose:** Unified priority management to prevent decision node conflicts
- **State Machine:**
  ```
  STATE_LANE (default)
    ↓
  Crosswalk detected? → STATE_CROSSWALK
    ↓ No
  Traffic light RED? → STATE_TRAFFIC_LIGHT
    ↓ No
  Obstacle nearby? → STATE_OBSTACLE_AVOID
    ↓ No
  STATE_LANE (lane following)
  ```
- **Priority Order** (highest to lowest):
  1. Crosswalk (pedestrian safety)
  2. Traffic light RED (rule compliance)
  3. Obstacle avoidance (collision prevention)
  4. Lane following (default behavior)
- **Hysteresis Filtering** (prevent rapid oscillation):
  - Crosswalk: 0.3s confirm, 0.3s release
  - Obstacle: 0.2s confirm, 0.5s release
- **Input Callbacks:**
  - `CB_CrosswalkDetected()` - Updates crosswalk flag
  - `CB_TrafficLightState()` - Updates traffic light status
  - `CB_ObstacleDetected()` - Updates obstacle flag
- **Control Loop (30 Hz):**
  ```cpp
  while (ros::ok()) {
    ros::spinOnce();  // Process incoming messages
    
    // Determine current state based on priorities
    if (crosswalk_detected && time_since_detect > 0.3s) {
      state = STATE_CROSSWALK;
    } else if (traffic_light_red) {
      state = STATE_TRAFFIC_LIGHT;
    } else if (obstacle_detected && time_since_detect > 0.2s) {
      state = STATE_OBSTACLE_AVOID;
    } else {
      state = STATE_LANE;
    }
    
    // Call appropriate decision step
    switch(state) {
      case STATE_LANE:           Lane_step();                    break;
      case STATE_CROSSWALK:      crosswalk_decision_step();      break;
      case STATE_TRAFFIC_LIGHT:  traffic_light_decision_step();  break;
      case STATE_OBSTACLE_AVOID: obstacle_avoid_decision_step(); break;
    }
    
    loop_rate.sleep();
  }
  ```

---

## Message Flow Diagram

### Normal Lane Following
```
Camera ─→ lane_perception_node ─→ /webot/lane_center
                                     ↓
                              main_node (STATE_LANE)
                                     ↓
                             lane_decision_node
                                     ↓
                         /commands/motor/speed
                        /commands/servo/position
                                     ↓
                            Vehicle Control
```

### With Traffic Light
```
Camera ─→ traffic_light_perception_node ─→ /webot/traffic_light/state
                                                    ↓
                                            main_node (STATE_TRAFFIC_LIGHT)
                                                    ↓
                                        traffic_light_decision_node
                                                    ↓
                            /commands/motor/speed (RED=0, GREEN=lane_speed)
```

### With Obstacle (LiDAR)
```
LiDAR ─→ lidar_obstacle_perception_node ─→ /webot/obstacle/best_gap
                                              ↓
                                      main_node (STATE_OBSTACLE_AVOID)
                                              ↓
                                   obstacle_avoid_decision_node
                                              ↓
                              /commands/motor/speed (slow 0.2 m/s)
                             /commands/servo/position (toward gap)
```

### Roundabout Collision Avoidance
```
LiDAR ─→ roundabout_perception_node ─→ /webot/roundabout/obstacle_detected
                                              ↓
                                   roundabout_decision_node
                                              ↓
                              /commands/motor/speed (0 if obstacle)
                             /commands/servo/position (center if obstacle)
```

---

## Topic Naming Convention

All topics follow the `/webot/*` naming scheme for unified integration:

### Perception Output Topics (`/webot/*`)
```
Lane:         /webot/lane_center, /webot/lane_curvature, /webot/lane_color
Traffic:      /webot/traffic_light/state
Crosswalk:    /webot/crosswalk/detected, /webot/crosswalk/stripe_ratio
Obstacle:     /webot/obstacle/best_gap, /webot/obstacle/min_distance, /webot/obstacle/has_obstacle
Roundabout:   /webot/roundabout/obstacle_detected, /webot/roundabout/obstacle_count
```

### Control Output Topics (`/commands/*`)
```
Motor:        /commands/motor/speed (throttle, m/s)
Servo:        /commands/servo/position (steering, 0.0-1.0)
```

---

## Launch Configuration

### Three-Level Launch System

```bash
# Level 1: Complete System (Recommended)
roslaunch decision_25 complete_25.launch
  ├── Launches perception_25.launch
  │   ├── lane_perception_node (camera)
  │   ├── traffic_light_perception_node (camera)
  │   ├── crosswalk_perception_node (camera)
  │   ├── lidar_obstacle_perception_node (LiDAR)
  │   └── roundabout_perception_node (LiDAR)
  └── Launches decision_25.launch
      └── decision_25_main (orchestrator)

# Level 2: Debugging Individual Missions
roslaunch decision_25 decision_25.launch launch_individual_nodes:=true
  ├── decision_25_main (disabled)
  ├── lane_decision_node
  ├── obstacle_avoid_decision_node
  ├── traffic_light_decision_node
  ├── crosswalk_decision_node
  └── roundabout_decision_node

# Level 3: Only Perception Nodes
roslaunch perception_25 perception_25.launch
  └── All 5 perception nodes (no decision)
```

---

## Parameter Configuration

### Lane Detection HSV
```yaml
# Yellow lane color space bounds
hsv_h_low: 18         # Hue lower bound
hsv_h_high: 38        # Hue upper bound
hsv_s_low: 100        # Saturation lower bound
hsv_s_high: 255       # Saturation upper bound
hsv_v_low: 110        # Value lower bound
hsv_v_high: 230       # Value upper bound
```

### Lane Control Gains
```yaml
servo_center: 0.57           # Center servo position (neutral)
steer_gain_base: 0.8         # Steering proportional gain
motor_gain: 300.0            # Speed motor constant
base_speed_mps: 7.0          # Base lane speed (m/s)
```

### Obstacle Avoidance FSM
```yaml
avoid_speed: 0.2             # Speed during obstacle avoidance (m/s)
steering_gain_obstacle: 0.02 # Steering sensitivity to gap angle
clear_threshold: 20          # Degree threshold to clear obstacle
```

---

## Execution Flow (Step-by-Step)

1. **Initialization (once on startup):**
   ```
   main_node → lane_decision_init()
            → obstacle_avoid_decision_init()
            → traffic_light_decision_init()
            → crosswalk_decision_init()
            
   All perception nodes start publishing on /webot/* topics
   ```

2. **Control Loop (every 30ms, 30 Hz):**
   ```
   Loop:
     - Receive perception messages via callbacks
     - Evaluate state priorities with hysteresis
     - Call appropriate decision node step() function
     - That step() function publishes /commands/* motor/servo
     - Vehicle executes commands
   ```

3. **Example Sequence (Vehicle approaching traffic light):**
   - T=0ms:   Camera detects RED light → `/webot/traffic_light/state` = "RED"
   - T=33ms:  main_node callback updates traffic_light flag
   - T=66ms:  Hysteresis timeout (0.0s already satisfied) → STATE_TRAFFIC_LIGHT
   - T=66ms:  `traffic_light_decision_step()` called
   - T=66ms:  Publishes `/commands/motor/speed` = 0.0 (STOP)
   - T=100ms: Vehicle decelerates due to motor command

---

## Key Design Patterns

### 1. **Single Responsibility**
- Each perception node handles ONE sensor modality
- Each decision node handles ONE mission/scenario
- main_node orchestrates priorities

### 2. **Callback-Driven Architecture**
- Perception nodes publish continuously (30 Hz)
- Decision nodes react to changes via callbacks
- main_node integrates callbacks into state machine

### 3. **Hysteresis Filtering**
- Prevents oscillation between states
- Example: Crosswalk requires 0.3s confirmation before stopping
- Release requires 0.3s clear before resuming

### 4. **Priority-Based Arbitration**
- main_node ensures only ONE decision node controls vehicle
- No conflicts between multiple motor/servo commands
- Clear ordering: Safety > Rules > Efficiency

---

## Testing Strategy

### Unit Testing (Individual Nodes)
```bash
# Test lane detection only
roslaunch perception_25 perception_25.launch &
rostopic echo /webot/lane_center
rostopic hz /webot/lane_center

# Test traffic light detection
rostopic echo /webot/traffic_light/state
```

### Integration Testing (With Decision)
```bash
# Full system with individual nodes
roslaunch decision_25 decision_25.launch launch_individual_nodes:=true &

# Monitor motor/servo commands
rostopic echo /commands/motor/speed
rostopic echo /commands/servo/position

# Monitor decision state
rostopic echo /webot/roundabout/state
```

### System Testing (Complete)
```bash
# Unified orchestrator
roslaunch decision_25 complete_25.launch

# Record all topics
rosbag record /webot/* /commands/* -o mission_log.bag

# Playback for analysis
rosbag play mission_log.bag --clock
```

---

## Performance Metrics

| Node | Input Rate | Compute Time | Output Rate | Latency |
|------|-----------|--------------|------------|---------|
| lane_perception | 30 Hz | ~15ms | 30 Hz | ~20ms |
| traffic_light_perception | 30 Hz | ~10ms | 30 Hz | ~15ms |
| crosswalk_perception | 30 Hz | ~20ms | 30 Hz | ~25ms |
| lidar_obstacle_perception | ~20 Hz (LiDAR) | ~8ms | 30 Hz | ~15ms |
| roundabout_perception | ~20 Hz (LiDAR) | ~5ms | 30 Hz | ~10ms |
| lane_decision | 30 Hz | ~3ms | 30 Hz | ~5ms |
| traffic_light_decision | 30 Hz | ~1ms | 30 Hz | ~2ms |
| crosswalk_decision | 30 Hz | ~2ms | 30 Hz | ~3ms |
| obstacle_avoid_decision | 30 Hz | ~4ms | 30 Hz | ~6ms |
| roundabout_decision | 30 Hz | ~1ms | 30 Hz | ~2ms |
| main_node orchestrator | 30 Hz | ~2ms | 30 Hz | ~3ms |

**Total System Latency:** ~50-70ms from sensor input to vehicle command

---

## Future Enhancements

1. **Sensor Fusion:** Combine camera + LiDAR for robust obstacle detection
2. **Machine Learning:** Use CNN for traffic sign/light recognition
3. **Path Planning:** Integrate global navigation with local obstacle avoidance
4. **Predictive Control:** Anticipate lane curves and obstacles
5. **Multi-Agent:** Coordinate with other vehicles in shared space
6. **Adaptive Gains:** Tune control parameters based on vehicle speed/load

