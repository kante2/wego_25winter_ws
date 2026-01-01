# Traffic Light Integration - Validation Checklist

## Refactoring Complete ✓

All components for traffic light mission-based control have been implemented and integrated into the WEGO autonomous vehicle decision system.

---

## Created Files

| File | Type | Purpose |
|------|------|---------|
| [traffic_light_detect_node.py](src/perception_wego/scripts/traffic_light_detect_node.py) | Perception | Detects RED/GREEN traffic light colors from camera |
| [mission_traffic_light.py](src/decision_wego/scripts/mission_traffic_light.py) | Decision | Step-based mission controller for traffic light behavior |

## Modified Files

| File | Changes |
|------|---------|
| [main_node.py](src/decision_wego/scripts/main_node.py) | Added TRAFFIC_LIGHT mission state (priority 2 after PARKING), updated decision loop |
| [decision_all.launch](src/decision_wego/launch/decision_all.launch) | Added traffic light + crosswalk mission parameters |
| [perception_all.launch](src/perception_wego/launch/perception_all.launch) | Added traffic light detection node launch |
| [decision_wego/CMakeLists.txt](src/decision_wego/CMakeLists.txt) | Added mission files to install list |
| [perception_wego/CMakeLists.txt](src/perception_wego/CMakeLists.txt) | Added detection node to install list |

---

## Architecture Summary

### Topic Flow
```
Camera (/usb_cam/image_raw/compressed)
  ↓
[traffic_light_detect_node] (Perception)
  ↓
/webot/traffic_light/state (String: "RED" | "GREEN" | "UNKNOWN")
  ↓
[mission_traffic_light] (Decision)
  ↓
[main_node] state machine
  ↓
Motor commands (AckermannDriveStamped)
```

### State Priority (main_node)
1. **PARKING** (0) - Marker found at trigger distance
2. **TRAFFIC_LIGHT** (3) - RED light detected → STOP
3. **CROSSWALK** (4) - Crosswalk detection triggered
4. **OBSTACLE** (1) - Object in path
5. **LANE** (0) - Default lane following

### Traffic Light Behavior
| State | Action | Speed | Steering |
|-------|--------|-------|----------|
| RED | Stop | 0.0 | 0.0 |
| GREEN | Continue (lane control) | lane_speed | lane_steer |
| UNKNOWN | Continue (lane control) | lane_speed | lane_steer |

---

## Deployment Steps

### 1. Push Changes to Repository
```bash
cd ~/wego_25winter_ws
git add -A
git commit -m "feat: modular traffic light perception + mission architecture"
git push origin main
```

### 2. On Robot (192.168.1.11)
```bash
ssh wego@192.168.1.11
cd ~/catkin_ws
git pull
catkin_make clean
catkin_make -j4
source devel/setup.bash
```

### 3. Launch System
```bash
# Terminal 1 - Perception nodes
roslaunch perception_wego perception_all.launch

# Terminal 2 - Decision nodes  
roslaunch decision_wego decision_all.launch
```

### 4. Verify Topics
```bash
# Check traffic light state updates
rostopic echo /webot/traffic_light/state

# Check main node state transitions
rostopic echo /wego_main_decision/state
```

### 5. Visualize
```bash
# View detection image
rqt_image_view /webot/traffic_light/image

# Tune parameters real-time
rqt_reconfigure
```

---

## Testing Scenarios

### Scenario 1: RED Light Detection
1. Point camera at red traffic light
2. Verify: `/webot/traffic_light/state` → "RED"
3. Verify: `/wego_main_decision/state` → "TRAFFIC_LIGHT | [TL_RED] stopped at RED light"
4. Verify: Vehicle stops (speed = 0.0)

### Scenario 2: GREEN Light Detection  
1. Point camera at green traffic light
2. Verify: `/webot/traffic_light/state` → "GREEN"
3. Verify: `/wego_main_decision/state` → "TRAFFIC_LIGHT | [TL_GREEN] proceeding with lane control"
4. Verify: Vehicle continues with lane control (uses lane_speed)

### Scenario 3: No Traffic Light (UNKNOWN)
1. Point camera away from traffic light
2. Verify: `/webot/traffic_light/state` → "UNKNOWN"
3. Verify: Vehicle uses lane control (graceful fallback)

### Scenario 4: Priority Test
1. Detect PARKING trigger (ArUco marker) while in traffic light
2. Expected: PARKING takes priority, traffic light deferred
3. Expected: Lane follows after parking completes

### Scenario 5: Crosswalk + Traffic Light
1. Trigger crosswalk while at red light
2. Expected: Traffic light priority > crosswalk
3. Vehicle stops for red, then crosswalk executes after green

---

## Parameter Tuning Guide

### If Red Lights Not Detected
1. Launch `rqt_reconfigure`
2. Expand `/traffic_light_detect` node
3. Adjust red HSV thresholds:
   - Increase `red_s_low` if lights too dark
   - Decrease `red_h_high1` or `red_h_low2` if color range too narrow
   - Adjust `brightness_factor` (0.1-1.0, lower = darker)

### If Green Lights Not Detected
1. Adjust green HSV thresholds
2. Verify `green_h_low`=35, `green_h_high`=85 reasonable for your lights
3. Check `min_area` threshold (detection size must exceed this)

### If False Positives
1. Increase `min_area` threshold
2. Increase `circularity_thresh` (0.6 default, 0.8 stricter)
3. Adjust ROI bounds to exclude non-traffic areas

---

## Comparison: Old vs New Architecture

### Old (traffic_light_solo.py)
```
- Monolithic node
- Detects color + publishes motor commands directly
- Hardcoded priority logic (intrudes on lane detection)
- Difficult to coordinate with other missions
- Debug via solo traffic light override flags
```

### New (perception + decision split)
```
✓ Clean separation of concerns
✓ Perception publishes pure detection data
✓ Mission implements control logic with step()
✓ Integrates with state machine (no priority conflicts)
✓ Easy to test each component independently
✓ Reusable mission pattern (same as parking, crosswalk)
```

---

## Dependencies & Requirements

### ROS Packages Required
- `rospy` - ROS Python client
- `std_msgs` - String message type
- `sensor_msgs` - Image message types
- `ackermann_msgs` - Motor control
- `cv_bridge` - OpenCV/ROS bridge
- `dynamic_reconfigure` - Parameter tuning (TrafficLight.cfg)
- `wego_cfg` - Config package dependency

### System Requirements
- OpenCV 3.4+ (for fisheye undistortion)
- Python 3.6+
- Camera calibration file: `/home/wego/catkin_ws/src/usb_cam/calibration/usb_cam.yaml`

---

## Common Issues & Solutions

| Issue | Diagnosis | Solution |
|-------|-----------|----------|
| Module import error | `ImportError: No module named 'mission_traffic_light'` | Verify CMakeLists.txt install list, run `catkin_make` |
| No traffic light state | `/webot/traffic_light/state` not published | Check perception node running: `rostopic list \| grep traffic` |
| Always shows "UNKNOWN" | Detection not working | Check camera connectivity, adjust HSV params via rqt_reconfigure |
| Vehicle doesn't stop on RED | Mission not taking control | Verify main_node state transitions: `rostopic echo /wego_main_decision/state` |
| State machine hangs | Node crashing silently | Check logs: `rostopic echo /rosout` |

---

## Verification Checklist (Before Deployment)

- [ ] Both new files created in correct locations
- [ ] CMakeLists.txt entries added for both packages
- [ ] Launch files include new nodes with correct parameters
- [ ] main_node.py compiles without import errors
- [ ] MissionState enum has TRAFFIC_LIGHT at priority 3
- [ ] _decide_state() checks traffic light is_active()
- [ ] spin() loop has elif branch for TRAFFIC_LIGHT case
- [ ] TrafficLight.cfg exists and is referenced
- [ ] All imports work (both cfg location fallbacks included)
- [ ] Documentation created (TRAFFIC_LIGHT_INTEGRATION.md)

---

## Next Phases (Future Enhancement)

1. **Advanced Traffic Light Detection**
   - Support yellow lights (caution state)
   - Arrow detection (left/right turn only)
   - Multiple traffic lights (choose correct lane)

2. **Integration with Traffic Rules**
   - Turn signals before turning
   - Ensure full clearance before proceeding
   - Queue position awareness

3. **Fail-Safe Mechanisms**
   - Sensor fusion (camera + V2X communication)
   - Timeout if light detection lost
   - Manual override capability

4. **Performance Optimization**
   - GPU-accelerated detection (CUDA/OpenGL)
   - Parallel processing with other perception modules
   - Reduced latency for safety-critical decisions

---

## References

- [ROS Mission Pattern](src/decision_wego/scripts/mission_parking.py) - Reference implementation
- [TrafficLight Configuration](src/inha25-winter-ros/wego/cfg/TrafficLight.cfg) - Parameter definitions
- [Dynamic Reconfigure Docs](http://wiki.ros.org/dynamic_reconfigure) - Real-time tuning

