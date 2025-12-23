# Roundabout Perception & Decision Implementation Summary

## Overview
Implemented LiDAR-based obstacle detection and decision logic for roundabout navigation in C++, following the pattern from `roundabout_node.py`.

## Files Created/Modified

### 1. Perception Node
**File:** [src/perception_25/src/camera/roundabout_perception_node.cpp](src/perception_25/src/camera/roundabout_perception_node.cpp)

**Purpose:** LiDAR-based obstacle detection for forward collision avoidance

**Key Features:**
- Converts polar coordinates (r, θ) from LiDAR to cartesian (x, y)
- Applies laser_link 180-degree rotation correction
- Filters points within detection area:
  - X-axis (forward): 0.2-0.8m
  - Y-axis (lateral): -0.3 to +0.3m
- Threshold-based detection: ≥3 points = obstacle detected

**Published Topics:**
- `/webot/roundabout/obstacle_detected` (std_msgs/Bool) - Binary obstacle flag
- `/webot/roundabout/obstacle_count` (std_msgs/Int32) - Number of detected points

**Parameters (ROS Dynamic Reconfigure):**
```yaml
detect_x_min: 0.2      # Forward detection start (meters)
detect_x_max: 0.8      # Forward detection end (meters)
detect_y_min: -0.3     # Left boundary (meters)
detect_y_max: 0.3      # Right boundary (meters)
obstacle_threshold: 3  # Points required for detection
```

### 2. Decision Node
**File:** [src/decision_25/src/roundabout_decision_node.cpp](src/decision_25/src/roundabout_decision_node.cpp)

**Purpose:** Control logic responding to obstacle detection

**Behavior:**
- **When obstacle detected:** Publish STOP command (motor speed=0, servo center)
- **When clear:** Yield control to main_node.cpp (do not publish)

**Subscribed Topics:**
- `/webot/roundabout/obstacle_detected` (std_msgs/Bool)
- `/webot/roundabout/obstacle_count` (std_msgs/Int32)

**Published Topics:**
- `/commands/motor/speed` (std_msgs/Float64) - Speed command (0 when stopped)
- `/commands/servo/position` (std_msgs/Float64) - Steering command (center when stopped)
- `/webot/roundabout/state` (std_msgs/String) - State for debugging ("STOPPED" or "CLEAR")

**Parameters:**
```yaml
servo_center: 0.57     # Center steering position
steer_sign: -1.0       # Sign correction for servo direction
```

### 3. CMakeLists Updates

**perception_25/CMakeLists.txt:**
```cmake
add_executable(roundabout_perception_node src/camera/roundabout_perception_node.cpp)
target_link_libraries(roundabout_perception_node ${catkin_LIBRARIES})
add_dependencies(roundabout_perception_node ${catkin_EXPORTED_TARGETS})
```

**decision_25/CMakeLists.txt:**
```cmake
add_executable(roundabout_decision_node src/roundabout_decision_node.cpp)
target_link_libraries(roundabout_decision_node ${catkin_LIBRARIES})
add_dependencies(roundabout_decision_node ${catkin_EXPORTED_TARGETS})
```

### 4. Launch File Updates

**perception_25/launch/perception_25.launch:**
- Added `roundabout_perception_node` configuration
- Passes scan_topic and detection parameters

**decision_25/launch/decision_25.launch:**
- Added `roundabout_decision_node` (launches only when `launch_individual_nodes:=true`)
- Main orchestrator (default) handles all decision logic

**decision_25/launch/complete_25.launch:**
- Automatically includes all perception and decision nodes
- Roundabout nodes launched by default

## Integration with Main Orchestrator

The roundabout nodes operate independently from `main_node.cpp`:

1. **Perception flow:**
   - LiDAR `/scan` → roundabout_perception_node → `/webot/roundabout/obstacle_detected`

2. **Decision flow:**
   - `/webot/roundabout/obstacle_detected` → roundabout_decision_node
   - If obstacle: Publish motor/servo STOP commands
   - If clear: No output (main_node controls vehicle)

3. **Priority in main_node:**
   - Roundabout obstacle avoidance acts as separate control path
   - Can be integrated into main_node state machine in future if needed

## Launch Command Examples

```bash
# Complete system with all nodes
roslaunch decision_25 complete_25.launch

# Only perception nodes
roslaunch perception_25 perception_25.launch

# Main decision orchestrator + roundabout perception
roslaunch decision_25 decision_25.launch use_main_orchestrator:=true

# Individual roundabout nodes for debugging
roslaunch decision_25 decision_25.launch use_main_orchestrator:=false launch_individual_nodes:=true
```

## Coordinate System

**LiDAR Data (laser_link frame):**
- Raw angles: -π to +π (laser_link orientation)
- 180° rotation correction applied to get vehicle-relative coordinates

**Cartesian Coordinates (vehicle frame):**
- X-axis: Forward direction (positive = front of vehicle)
- Y-axis: Lateral direction (positive = left side)

**Detection Region:**
```
        y-axis
          |
    +-----|-----+
    |  0.3m    |  Detection Zone
    |     X    |  X marks centroid of obstacles
    |  0.2-0.8m|
    +-----|-----+ (front of vehicle)
          |
```

## Testing Checklist

- [ ] Launch perception_25.launch and verify `/webot/roundabout/*` topics publish
- [ ] Use `rostopic echo /webot/roundabout/obstacle_detected` to test obstacle flag
- [ ] Place obstacle in front of LiDAR, verify detection within region
- [ ] Launch decision_25.launch and verify motor/servo commands publish when obstacle detected
- [ ] Verify control yields to main_node when no obstacle
- [ ] Test hysteresis: obstacle enters/exits detection region smoothly

## Future Enhancements

1. **Integrate into main_node:** Add roundabout state to FSM
2. **Hysteresis filtering:** Add debouncing (0.2-0.3s) for state transitions
3. **Multi-sensor fusion:** Combine with camera-based obstacle detection
4. **Yield behavior:** Gradually release instead of instant stop
5. **Curve detection:** Integrate roundabout_node.py's circle fitting logic

## Reference

Original Python implementation: [src/inha25-winter-ros/wego/scripts/roundabout_node.py](src/inha25-winter-ros/wego/scripts/roundabout_node.py)

