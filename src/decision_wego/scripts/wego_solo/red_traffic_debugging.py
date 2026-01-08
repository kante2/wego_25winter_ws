#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Red Traffic Light Debugging Tool
- Displays camera feed with red color mask
- Shows RED_FLAG when red ratio exceeds threshold
- Real-time visualization for debugging
"""

import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class RedTrafficDebugger:
    def __init__(self):
        rospy.init_node('red_traffic_debugger')
        
        self.cv_bridge = CvBridge()
        self.bgr = None
        
        # Red detection parameters
        self.red_lower1 = np.array([0, 100, 100], dtype=np.uint8)    # 빨간색 HSV 하한 (0-10)
        self.red_upper1 = np.array([10, 255, 255], dtype=np.uint8)   # 빨간색 HSV 상한 (0-10)
        self.red_lower2 = np.array([170, 100, 100], dtype=np.uint8)  # 빨간색 HSV 하한 (170-180)
        self.red_upper2 = np.array([180, 255, 255], dtype=np.uint8)  # 빨간색 HSV 상한 (170-180)
        
        self.red_threshold = rospy.get_param('~red_threshold', 0.01)  # 1% (기본값)
        
        # Subscriber
        self.image_sub = rospy.Subscriber(
            '/usb_cam/image_raw/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo("="*60)
        rospy.loginfo("Red Traffic Light Debugger Initialized")
        rospy.loginfo(f"Red Threshold: {self.red_threshold*100:.2f}%")
        rospy.loginfo("Press 'q' to quit")
        rospy.loginfo("="*60)
    
    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
            if cv_image is None:
                return
            self.bgr = cv_image
        except Exception as e:
            rospy.logerr(f"[RedDebug] Error: {e}")
    
    def detect_red(self, img):
        """
        Detect red color in lower-left portion of image
        좌측 하단부(60%~100%)에서 빨간색 감지
        Returns: (red_mask, red_ratio, red_detected)
        """
        h, w = img.shape[:2]
        # 좌측 절반 중에서, 세로 60%~100% (하단 40%) 영역만 추출
        roi_y_start = int(h * 0.6)  # 60%부터 시작
        roi_y_end = h  # 100%까지
        roi_x_start = 0  # 좌측
        roi_x_end = w // 2  # 절반까지
        
        lower_left_roi = img[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        
        # HSV 변환
        hsv = cv.cvtColor(lower_left_roi, cv.COLOR_BGR2HSV)
        
        # 빨간색은 HSV에서 0-10, 170-180 두 범위
        mask1 = cv.inRange(hsv, self.red_lower1, self.red_upper1)
        mask2 = cv.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv.bitwise_or(mask1, mask2)
        
        # 빨간색 픽셀 비율 계산
        red_pixel_count = cv.countNonZero(red_mask)
        total_pixels = red_mask.shape[0] * red_mask.shape[1]
        red_ratio = red_pixel_count / total_pixels if total_pixels > 0 else 0.0
        
        # 빨간색 감지 여부
        red_detected = (red_ratio > self.red_threshold)
        
        return red_mask, red_ratio, red_detected
    
    def draw_debug_info(self, img, red_mask, red_ratio, red_detected):
        """Draw debug information on image"""
        h, w = img.shape[:2]
        
        # 원본 이미지 복사
        debug_img = img.copy()
        
        # ROI 영역 계산
        roi_y_start = int(h * 0.6)  # 60%부터 시작
        roi_y_end = h  # 100%까지
        roi_x_start = 0  # 좌측
        roi_x_end = w // 2  # 절반까지
        
        # 좌측 하단부에 빨간색 마스크 오버레이
        lower_left_roi = debug_img[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        
        # 빨간색 마스크를 컬러로 변환
        red_overlay = cv.cvtColor(red_mask, cv.COLOR_GRAY2BGR)
        red_overlay[:, :, 0] = 0  # B 채널 0
        red_overlay[:, :, 1] = 0  # G 채널 0
        # R 채널은 마스크 값 그대로
        
        # 알파 블렌딩
        cv.addWeighted(lower_left_roi, 0.7, red_overlay, 0.3, 0, lower_left_roi)
        
        # ROI 경계선 그리기 (좌측 절반 세로선)
        cv.line(debug_img, (w//2, 0), (w//2, h), (0, 255, 255), 2)
        # ROI 경계선 그리기 (60% 가로선)
        cv.line(debug_img, (0, roi_y_start), (w//2, roi_y_start), (0, 255, 0), 2)
        # ROI 박스
        cv.rectangle(debug_img, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), (0, 255, 0), 2)
        
        # 텍스트 배경
        cv.rectangle(debug_img, (10, 10), (w//2 - 10, 120), (0, 0, 0), -1)
        cv.rectangle(debug_img, (10, 10), (w//2 - 10, 120), (255, 255, 255), 2)
        
        # 빨간색 비율 표시
        text1 = f"Red Ratio: {red_ratio*100:.2f}%"
        cv.putText(debug_img, text1, (20, 40), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 임계값 표시
        text2 = f"Threshold: {self.red_threshold*100:.2f}%"
        cv.putText(debug_img, text2, (20, 70), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # RED FLAG 표시
        if red_detected:
            # 빨간색 배경
            cv.rectangle(debug_img, (20, 85), (w//2 - 20, 115), (0, 0, 255), -1)
            cv.putText(debug_img, "RED FLAG!", (30, 107), 
                       cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv.LINE_AA)
            
            # 화면 전체 빨간색 테두리
            cv.rectangle(debug_img, (0, 0), (w-1, h-1), (0, 0, 255), 5)
        else:
            cv.putText(debug_img, "Status: CLEAR", (30, 107), 
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return debug_img
    
    def main(self):
        rate = rospy.Rate(30)  # 30 Hz
        
        while not rospy.is_shutdown():
            if self.bgr is not None:
                # 빨간색 감지
                red_mask, red_ratio, red_detected = self.detect_red(self.bgr)
                
                # 디버그 이미지 생성
                debug_img = self.draw_debug_info(self.bgr, red_mask, red_ratio, red_detected)
                
                # 빨간색 마스크만 별도 창에 표시
                h, w = self.bgr.shape[:2]
                full_red_mask = np.zeros((h, w), dtype=np.uint8)
                full_red_mask[:, :w//2] = red_mask
                
                # 화면 표시
                cv.imshow("Red Traffic Debug", debug_img)
                cv.imshow("Red Mask (Left Half)", full_red_mask)
                
                # 터미널 출력
                if red_detected:
                    rospy.logwarn_throttle(0.5, f"[RED DETECTED] Ratio: {red_ratio*100:.2f}%")
                else:
                    rospy.loginfo_throttle(2.0, f"[Clear] Red ratio: {red_ratio*100:.2f}%")
                
                # 키 입력 확인 ('q' 누르면 종료)
                key = cv.waitKey(1) & 0xFF
                if key == ord('q'):
                    rospy.loginfo("User requested quit")
                    break
            
            rate.sleep()
        
        cv.destroyAllWindows()

if __name__ == '__main__':
    try:
        debugger = RedTrafficDebugger()
        debugger.main()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv.destroyAllWindows()
