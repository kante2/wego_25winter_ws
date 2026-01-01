# Traffic Light Detection & Mission Integration
## Architecture Refactoring Summary

### Overview
Refactored the old `traffic_light_solo.py` monolithic node into a modular architecture:
- **Perception Module**: `traffic_light_detect_node.py` - Pure detection, publishes color state
- **Decision Module**: `mission_traffic_light.py` - Step-based control logic
- **Integration**: Added to `main_node.py` state machine with priority ranking

---

## Files Created

### 1. **traffic_light_detect_node.py** (Perception)
**Location**: `/src/perception_wego/scripts/`

**Purpose**: 
- Detect traffic light state (RED/GREEN/UNKNOWN) from camera stream
- Applies fisheye undistortion, ROI extraction, HSV filtering
- No motor command publishing (pure perception)

**Key Features**:
- Subscribes to `/usb_cam/image_raw/compressed`
- Publishes `/webot/traffic_light/state` (String: RED, GREEN, UNKNOWN)
- Publishes debug images to `/webot/traffic_light/image` and `/webot/traffic_light/debug`
- Uses TrafficLight.cfg dynamic reconfigure for parameter tuning:
  - Red HSV ranges (two ranges for hue wrap-around)
  - Green HSV ranges
  - ROI bounds (traffic light at top of image)
  - Brightness adjustment factor
  - Min detection area and circularity threshold

**Parameters** (from TrafficLight.cfg):
```
red_h_low1=0, red_h_high1=10          # Red hue range 1 (0-10)
red_h_low2=160, red_h_high2=180       # Red hue range 2 (160-180) for wrap
red_s_low=100, red_s_high=255         # Red saturation
red_v_low=100, red_v_high=255         # Red value

green_h_low=35, green_h_high=85       # Green hue
green_s_low=100, green_s_high=255     # Green saturation
green_v_low=100, green_v_high=255     # Green value

min_area=500                          # Minimum contour area
circularity_thresh=0.6                # Shape validation (circular lights)

roi_top=0, roi_bottom=240             # ROI for light detection
roi_left=0, roi_right=640
brightness_factor=0.5                 # Adaptive brightness
```

### 2. **mission_traffic_light.py** (Decision)
**Location**: `/src/decision_wego/scripts/`

**Purpose**:
- Implements `TrafficLightMission` class with step() function
- Consumes `/webot/traffic_light/state` from perception
- Returns (speed, steering, debug_msg) for main_node integration

**Key Features**:
- **State Machine**:
  - IDLE → WAITING (on RED) → PROCEEDING (on GREEN)
  - UNKNOWN defaults to PROCEEDING with lane control
  
- **Control Logic**:
  - RED: Stop (speed=0.0, steer=0.0)
  - GREEN: Use lane control values (speed/steer from lane detection)
  - UNKNOWN: Use lane control values (graceful degradation)

- **Methods**:
  - `step(dt=0.05)`: Returns (speed, steering, debug_str)
  - `is_active()`: Returns True if RED light detected
  - `reset()`: Reset mission state
  - `get_debug_info()`: Return state dictionary

- **Initialization**:
  ```python
  mission = TrafficLightMission()
  mission.init_from_params("~traffic_light")  # Subscribe to ROS params + topics
  ```

### 3. **main_node.py** (Updated Decision Hub)
**Location**: `/src/decision_wego/scripts/`

**Changes**:
- Import `mission_traffic_light`
- Add `TRAFFIC_LIGHT = 3` to MissionState enum
- Adjust CROSSWALK priority to 4 (was 3)
- Instantiate `self.m_traffic_light = TrafficLightMission()`
- Add `m_traffic_light.init_from_params("~traffic_light")`
- Update `_decide_state()` priority:
  ```
  PARKING (highest) > TRAFFIC_LIGHT (RED only) > CROSSWALK > OBSTACLE > LANE (default)
  ```
- Add traffic light case in spin() loop

**New State Priority Hierarchy**:
1. **PARKING** - Highest (ArUco marker detected, distance < 0.5m)
2. **TRAFFIC_LIGHT** - RED light must stop (safety critical)
3. **CROSSWALK** - Parallel crosswalk maneuver
4. **OBSTACLE** - Avoid detected obstacles
5. **LANE** - Default lane following

---

## Files Modified

### 1. **decision_wego/CMakeLists.txt**
- Added `mission_crosswalk.py` to install list
- Added `mission_traffic_light.py` to install list

### 2. **perception_wego/CMakeLists.txt**
- Added `traffic_light_detect_node.py` to install list

### 3. **perception_wego/launch/perception_all.launch**
- Added traffic light detection node:
  ```xml
  <node pkg="perception_wego" type="traffic_light_detect_node.py" name="traffic_light_detect" 
        output="screen" respawn="false">
    <param name="~calibration_file" value="$(find usb_cam)/calibration/usb_cam.yaml"/>
  </node>
  ```

### 4. **decision_wego/launch/decision_all.launch**
- Added crosswalk mission parameters
- Added traffic light mission parameters

---

## Topic Organization

### Perception → Decision Pipeline

```
Camera Stream
    ↓
[traffic_light_detect_node.py]
    ↓
/webot/traffic_light/state (String: RED/GREEN/UNKNOWN)
    ↓
[mission_traffic_light.py]
    ↓
(speed, steering) → main_node
    ↓
[main_node.py - state priority]
    ↓
/low_level/ackermann_cmd_mux/input/navigation (AckermannDriveStamped)
    ↓
Motor Controller
```

### Debug Topics
- `/webot/traffic_light/image` - Visualization with ROI and detections
- `/webot/traffic_light/debug` - Mask visualization (red/green channels)

---

## Testing Checklist

1. **Build ROS Packages**
   ```bash
   cd ~/wego_25winter_ws
   catkin_make
   ```

2. **Launch Complete System**
   ```bash
   roslaunch perception_wego perception_all.launch
   roslaunch decision_wego decision_all.launch
   ```

3. **Verify Topics**
   ```bash
   # Should see traffic light state updates
   rostopic echo /webot/traffic_light/state
   
   # Verify main node state transitions
   rostopic echo /wego_main_decision/state
   ```

4. **Debug Visualization**
   ```bash
   # View traffic light detection image
   rqt_image_view /webot/traffic_light/image
   
   # View detection mask
   rqt_image_view /webot/traffic_light/debug
   
   # Tune parameters in real-time
   rqt_reconfigure  # Select /traffic_light_detect
   ```

5. **Real-World Test**
   - Point camera at traffic light
   - Verify `/webot/traffic_light/state` shows correct color
   - Test RED light: vehicle should STOP
   - Test GREEN light: vehicle should CONTINUE with lane control
   - Test UNKNOWN: vehicle should gracefully continue

---

## Integration with Existing Architecture

**Compatibility**:
- ✅ Follows same pattern as `mission_parking.py`, `mission_crosswalk.py`
- ✅ Uses ROS parameter server for configuration
- ✅ Integrated into main_node.py state machine
- ✅ Published on canonical `/webot/traffic_light/state` topic
- ✅ Supports dynamic reconfigure (TrafficLight.cfg)
- ✅ 20Hz control loop (matches other missions)

**State Priority Notes**:
- Traffic light has **HIGH priority** (2nd after PARKING) because RED is a safety-critical constraint
- Vehicle STOPS on RED regardless of lane/obstacle conditions
- GREEN/UNKNOWN defers to lane control (allows obstacle avoidance during green)

---

## Old File Status

**traffic_light_solo.py** (`/src/decision_wego/solo/`):
- Kept as-is for reference
- Replaced by modular perception + mission split
- Can be deprecated after testing new system

---

## Next Steps

1. **Deploy to Robot**
   ```bash
   git add -A
   git commit -m "feat: refactor traffic light into perception + mission modules"
   git push
   ```

2. **Test on Robot (192.168.1.11)**
   ```bash
   ssh wego@192.168.1.11
   cd ~/catkin_ws
   git pull
   catkin_make
   roslaunch perception_wego perception_all.launch &
   roslaunch decision_wego decision_all.launch
   ```

3. **Parameter Tuning** (if needed)
   - Use `rqt_reconfigure` to adjust HSV ranges for your specific traffic lights
   - Adjust ROI bounds based on camera mounting angle
   - Tune `brightness_factor` if images are too dark/bright

4. **Validation**
   - Record rosbag of mission execution
   - Verify state transitions in real-world conditions
   - Test edge cases (flashing lights, partially visible lights, shadows)

