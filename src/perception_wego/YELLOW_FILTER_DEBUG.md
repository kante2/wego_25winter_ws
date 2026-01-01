# 🟡 Yellow Lane Filter Debug Guide

## 목적
- 노란색 필터가 제대로 작동하는지 확인
- BEV 변환이 정상적인지 시각화
- 전체 프레임 중 노란색 픽셀의 비율 계산
- 실시간으로 HSV 범위 조정

## 방법 1: 스탠드얼론 스크립트 (로컬 개발 머신)

### 웹캠으로 실시간 디버깅
```bash
cd /home/kante/wego_25winter_ws
python3 src/perception_wego/scripts/yellow_filter_debug_standalone.py --webcam
```

### 이미지 파일로 디버깅
```bash
python3 src/perception_wego/scripts/yellow_filter_debug_standalone.py --image /path/to/image.jpg
```

### 사용 방법
1. 창이 5개 뜹니다:
   - **Original + ROI**: 원본 이미지 + ROI 폴리곤 (녹색 테두리)
   - **BEV Image**: 조감도(Bird's Eye View) 변환된 이미지
   - **Yellow Mask**: 필터링된 노란색 마스크
   - **HSV Range Control**: 트랙바로 HSV 범위 조정

2. 트랙바 조정:
   - `H Lower/Upper`: Hue (0-179) - 노란색은 18-38
   - `S Lower/Upper`: Saturation (0-255) - 채도는 100-255
   - `V Lower/Upper`: Value (0-255) - 밝기는 110-230

3. 터미널에서 실시간 비율 확인:
   ```
   [Yellow Filter] Ratio: 15.23% | Pixels: 48960/320000 | H:[18-38] S:[100-255] V:[110-230]
   ```

4. **최종 HSV 범위 저장**:
   - 'q' 키로 종료하면 최종 HSV 범위 출력됨
   - 이 값을 `lane_detect_perception_ver2.py`에 적용

## 방법 2: ROS 노드 (로봇에서 X11 포워딩)

### SSH로 X11 포워딩 연결
```bash
ssh -X wego@192.168.1.11
source ~/wego25_winter_ws/devel/setup.bash
rosrun perception_wego yellow_filter_debug.py
```

### ROS 토픽으로 모니터링
```bash
# 다른 터미널에서
rostopic echo /debug/yellow_ratio
```

## 현재 설정값

```python
# HSV Range for Yellow Lane
yellow_lower = np.array([18, 100, 110])
yellow_upper = np.array([38, 255, 230])

# ROI (Trapezoid)
roi_top_y_ratio = 0.60         # 이미지 상단 60% 지점
roi_left_top_ratio = 0.10      # 좌상단 10% x좌표
roi_right_top_ratio = 0.85     # 우상단 85% x좌표
roi_left_bot_ratio = -0.45     # 좌하단 -45% x좌표 (확장)
roi_right_bot_ratio = 1.40     # 우하단 140% x좌표 (확장)
```

## 예상 결과

### 좋은 결과 (차선이 잘 감지됨)
```
[Yellow Filter] Ratio: 10-20% | Yellow Pixels: 32000-64000 | ...
- 노란 차선이 명확하게 흰색(255) 표시
- 노이즈 최소화
```

### 나쁜 결과 (차선이 안 보임)
```
[Yellow Filter] Ratio: 0-5% | Yellow Pixels: 0-16000 | ...
- 흰색 픽셀이 거의 없음
- HSV 범위를 더 넓혀야 함
```

### 노이즈가 많은 경우
```
[Yellow Filter] Ratio: 40%+ | ...
- 도로, 배경 등 불필요한 부분도 감지됨
- HSV 범위를 더 좁혀야 함
```

## 트러블슈팅

### 1. 노란색이 안 보일 때
- H 범위를 더 넓혀보기: 18-38 → 15-45
- S 범위 낮추기: 100-255 → 80-255
- V 범위 낮추기: 110-230 → 80-255

### 2. 노이즈가 많을 때
- H 범위 좁히기: 18-38 → 20-36
- S 범위 높이기: 100-255 → 120-255
- V 범위 높이기: 110-230 → 140-255

### 3. BEV 변환이 이상할 때
- ROI 폴리곤이 제대로 그려졌는지 확인 (녹색 테두리)
- ROI 비율 값 확인:
  ```
  roi_top_y_ratio = 0.60       # 이미지의 어느 높이부터?
  roi_left/right_bot_ratio     # 하단이 얼마나 확장?
  ```

## 최종 적용

최적의 HSV 값을 찾은 후:

```python
# lane_detect_perception_ver2.py의 46-47줄
self.yellow_lower = np.array([H_LOWER, S_LOWER, V_LOWER])
self.yellow_upper = np.array([H_UPPER, S_UPPER, V_UPPER])
```

변경 후 다시 빌드:
```bash
cd ~/wego25_winter_ws
catkin_make
```
