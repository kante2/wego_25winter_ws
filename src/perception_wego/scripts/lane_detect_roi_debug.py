#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lane Detection ROI Debug Tool
- ROI 영역을 시각화하여 카메라 입력에 표시
- top_ratio, bottom_ratio를 실시간으로 조정
- HSV 필터링 결과도 함께 표시
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class LaneDetectROIDebug:
    def __init__(self):
        rospy.init_node('lane_detect_roi_debug')
        
        self.bridge = CvBridge()
        
        # 기본 파라미터
        self.roi_top_ratio = rospy.get_param('~roi_top_ratio', 0.3)
        self.roi_bottom_ratio = rospy.get_param('~roi_bottom_ratio', 1.0)
        
        # HSV 필터 파라미터
        self.hsv_h_low = rospy.get_param('~hsv_h_low', 0)
        self.hsv_h_high = rospy.get_param('~hsv_h_high', 180)
        self.hsv_s_low = rospy.get_param('~hsv_s_low', 0)
        self.hsv_s_high = rospy.get_param('~hsv_s_high', 50)
        self.hsv_v_low = rospy.get_param('~hsv_v_low', 200)
        self.hsv_v_high = rospy.get_param('~hsv_v_high', 255)
        
        # 마커 파라미터
        self.min_pixels = rospy.get_param('~min_pixels', 500)
        
        # Subscriber
        self.image_sub = rospy.Subscriber(
            '/usb_cam/image_raw/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo("="*60)
        rospy.loginfo("Lane Detect ROI Debug Tool")
        rospy.loginfo(f"ROI: top_ratio={self.roi_top_ratio}, bottom_ratio={self.roi_bottom_ratio}")
        rospy.loginfo("Controls:")
        rospy.loginfo("  - 'w': top_ratio 증가 (위 더 스킵)")
        rospy.loginfo("  - 'q': top_ratio 감소 (위 덜 스킵)")
        rospy.loginfo("  - 's': bottom_ratio 증가 (아래 더 스킵)")
        rospy.loginfo("  - 'a': bottom_ratio 감소 (아래 덜 스킵)")
        rospy.loginfo("  - 'r': 초기값 리셋")
        rospy.loginfo("  - ESC: 종료")
        rospy.loginfo("="*60)
    
    def image_callback(self, msg):
        try:
            # 이미지 디코드
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img is None:
                return
            
            height, width = img.shape[:2]
            
            # ========================================
            # ROI 계산
            # ========================================
            roi_top = int(height * self.roi_top_ratio)
            roi_bottom = int(height * self.roi_bottom_ratio)
            
            # ========================================
            # 원본 이미지에 ROI 영역 표시
            # ========================================
            img_with_roi = img.copy()
            
            # ROI 영역 (녹색 사각형)
            cv2.rectangle(img_with_roi, (0, roi_top), (width, roi_bottom), (0, 255, 0), 3)
            
            # ROI 제목
            cv2.putText(img_with_roi, f"ROI: top={self.roi_top_ratio:.2f}, bottom={self.roi_bottom_ratio:.2f}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # 스킵되는 영역 표시 (회색)
            if roi_top > 0:
                cv2.rectangle(img_with_roi, (0, 0), (width, roi_top), (100, 100, 100), -1)
                cv2.putText(img_with_roi, f"SKIP: {self.roi_top_ratio*100:.0f}%",
                           (10, roi_top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            if roi_bottom < height:
                cv2.rectangle(img_with_roi, (0, roi_bottom), (width, height), (100, 100, 100), -1)
                cv2.putText(img_with_roi, f"SKIP: {(1-self.roi_bottom_ratio)*100:.0f}%",
                           (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # ========================================
            # ROI 영역만 추출 및 HSV 필터링
            # ========================================
            roi = img[roi_top:roi_bottom, :]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            lower = np.array([self.hsv_h_low, self.hsv_s_low, self.hsv_v_low])
            upper = np.array([self.hsv_h_high, self.hsv_s_high, self.hsv_v_high])
            mask = cv2.inRange(hsv, lower, upper)
            
            # 형태학 연산
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # ========================================
            # 히스토그램으로 차선 중심 찾기
            # ========================================
            h, w = mask.shape
            histogram = np.sum(mask[h//2:, :], axis=0)
            
            midpoint = w // 2
            left_peak = np.argmax(histogram[:midpoint])
            right_peak = np.argmax(histogram[midpoint:]) + midpoint
            
            left_val = histogram[left_peak]
            right_val = histogram[right_peak]
            
            left_detected = left_val > self.min_pixels
            right_detected = right_val > self.min_pixels
            
            # 중심 계산
            if left_detected and right_detected:
                center_x = (left_peak + right_peak) // 2
                center_status = "BOTH"
            elif left_detected:
                center_x = left_peak + 50
                center_status = "LEFT"
            elif right_detected:
                center_x = right_peak - 50
                center_status = "RIGHT"
            else:
                center_x = w // 2
                center_status = "NONE"
            
            # ========================================
            # 4개 이미지 합치기
            # ========================================
            
            # ROI 부분 시각화
            roi_vis = roi.copy()
            
            # 차선 중심 표시
            if center_status != "NONE":
                cv2.circle(roi_vis, (center_x, h//2), 10, (0, 255, 0), -1)
                cv2.line(roi_vis, (center_x, 0), (center_x, h), (0, 255, 0), 2)
            
            # 피크 표시
            if left_detected:
                cv2.circle(roi_vis, (left_peak, h//2), 8, (255, 0, 0), -1)
                cv2.putText(roi_vis, f"L:{left_val:.0f}", (left_peak-30, h//2-15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            
            if right_detected:
                cv2.circle(roi_vis, (right_peak, h//2), 8, (0, 0, 255), -1)
                cv2.putText(roi_vis, f"R:{right_val:.0f}", (right_peak+10, h//2-15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # 상태 표시
            cv2.putText(roi_vis, f"Center Status: {center_status}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(roi_vis, f"min_pixels: {self.min_pixels}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            
            # 마스크를 컬러로 변환
            mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            
            # 리사이즈 (보기 좋게)
            display_h, display_w = height//2, width//2
            img_with_roi_resized = cv2.resize(img_with_roi, (display_w, display_h))
            roi_vis_resized = cv2.resize(roi_vis, (display_w, display_h))
            mask_resized = cv2.resize(mask_color, (display_w, display_h))
            
            # 레이블 추가
            cv2.putText(img_with_roi_resized, "1. Original with ROI", (10, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(roi_vis_resized, "2. ROI Lane Detection", (10, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(mask_resized, "3. HSV Mask", (10, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # 합치기
            top_row = np.hstack([img_with_roi_resized, roi_vis_resized])
            bottom_row = np.hstack([mask_resized, mask_resized])
            combined = np.vstack([top_row, bottom_row])
            
            # 표시
            cv2.imshow("Lane Detect ROI Debug", combined)
            
            # 키보드 입력
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                rospy.signal_shutdown("User closed debug window")
            elif key == ord('w'):  # top_ratio 증가
                self.roi_top_ratio = min(0.9, self.roi_top_ratio + 0.05)
                rospy.loginfo(f"ROI top_ratio: {self.roi_top_ratio:.2f}")
            elif key == ord('q'):  # top_ratio 감소
                self.roi_top_ratio = max(0.0, self.roi_top_ratio - 0.05)
                rospy.loginfo(f"ROI top_ratio: {self.roi_top_ratio:.2f}")
            elif key == ord('s'):  # bottom_ratio 증가
                self.roi_bottom_ratio = min(1.0, self.roi_bottom_ratio + 0.05)
                rospy.loginfo(f"ROI bottom_ratio: {self.roi_bottom_ratio:.2f}")
            elif key == ord('a'):  # bottom_ratio 감소
                self.roi_bottom_ratio = max(self.roi_top_ratio + 0.1, self.roi_bottom_ratio - 0.05)
                rospy.loginfo(f"ROI bottom_ratio: {self.roi_bottom_ratio:.2f}")
            elif key == ord('r'):  # 리셋
                self.roi_top_ratio = 0.3
                self.roi_bottom_ratio = 1.0
                rospy.loginfo("ROI reset to default (top=0.3, bottom=1.0)")
            
        except Exception as e:
            rospy.logerr(f"Error: {e}")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = LaneDetectROIDebug()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
