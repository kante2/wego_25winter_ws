#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Yellow Filter Debug Tool
- 실시간 카메라 입력에서 노란색 필터링 과정을 시각화
- HSV 범위, 형태학 연산, Contour 필터링 결과를 모두 표시
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class YellowFilterDebug:
    def __init__(self):
        rospy.init_node('yellow_filter_debug')
        
        self.bridge = CvBridge()
        
        # HSV 필터 파라미터
        self.h_low = rospy.get_param('~h_low', 15)
        self.h_high = rospy.get_param('~h_high', 35)
        self.s_low = rospy.get_param('~s_low', 80)
        self.s_high = rospy.get_param('~s_high', 255)
        self.v_low = rospy.get_param('~v_low', 80)
        self.v_high = rospy.get_param('~v_high', 255)
        self.min_area = rospy.get_param('~min_area', 500)
        
        # 형태학 커널 크기
        self.kernel_size = rospy.get_param('~kernel_size', 5)
        
        # Subscriber
        self.sub_image = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage,
                                          self.image_callback, queue_size=1)
        
        rospy.loginfo("="*60)
        rospy.loginfo("Yellow Filter Debug Tool Started")
        rospy.loginfo(f"HSV Range: H[{self.h_low}-{self.h_high}] S[{self.s_low}-{self.s_high}] V[{self.v_low}-{self.v_high}]")
        rospy.loginfo(f"Min Area: {self.min_area}")
        rospy.loginfo(f"Kernel Size: {self.kernel_size}x{self.kernel_size}")
        rospy.loginfo("="*60)
        rospy.loginfo("OpenCV 윈도우에 다음이 표시됩니다:")
        rospy.loginfo("  1. 원본 이미지 (좌상)")
        rospy.loginfo("  2. HSV 필터링 후 마스크 (우상)")
        rospy.loginfo("  3. 형태학 연산 후 마스크 (좌하)")
        rospy.loginfo("  4. Dilate 적용 후 마스크 (우하)")
        rospy.loginfo("  5. Contour + 면적 필터링 결과 (큰 윈도우)")
        rospy.loginfo("="*60)
    
    def image_callback(self, msg):
        try:
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img is None:
                return
            
            height, width = img.shape[:2]
            
            # ========================================
            # 1. HSV 변환
            # ========================================
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            # ========================================
            # 2. HSV 범위 필터링
            # ========================================
            lower_yellow = np.array([self.h_low, self.s_low, self.v_low])
            upper_yellow = np.array([self.h_high, self.s_high, self.v_high])
            mask_hsv = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            # ========================================
            # 3. 형태학 연산 (OPEN + CLOSE)
            # ========================================
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.kernel_size, self.kernel_size))
            mask_open = cv2.morphologyEx(mask_hsv, cv2.MORPH_OPEN, kernel)
            mask_morphology = cv2.morphologyEx(mask_open, cv2.MORPH_CLOSE, kernel)
            
            # ========================================
            # 4. Dilate 적용
            # ========================================
            mask_dilate = cv2.dilate(mask_morphology, kernel, iterations=1)
            
            # ========================================
            # 5. Contour 추출 및 면적 필터링
            # ========================================
            contours, _ = cv2.findContours(mask_dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 면적 필터링
            filtered_contours = [c for c in contours if cv2.contourArea(c) > self.min_area]
            
            # 최종 결과 이미지
            result_img = img.copy()
            
            # Contour 그리기
            cv2.drawContours(result_img, filtered_contours, -1, (0, 255, 0), 2)
            
            # 감지된 콘 개수 표시
            yellow_count = len(filtered_contours)
            text = f"Yellow Cones: {yellow_count}"
            cv2.putText(result_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            
            # ===== 콘별 픽셀 정보 추출 =====
            cone_info = []  # (area, centroid_x, centroid_y, contour)
            max_area = 0
            max_cone_idx = -1
            
            for i, c in enumerate(filtered_contours):
                area = cv2.contourArea(c)
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cone_info.append((area, cx, cy, c))
                    
                    # 최대 면적 추적
                    if area > max_area:
                        max_area = area
                        max_cone_idx = i
                    
                    # 원본 이미지에 정보 표시
                    cv2.circle(result_img, (cx, cy), 5, (255, 0, 0), -1)
                    cv2.putText(result_img, f"{area:.0f}px", (cx - 20, cy - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
            
            # ===== 가장 큰 콘 정보 강조 표시 =====
            if max_cone_idx >= 0:
                max_area, max_cx, max_cy, _ = cone_info[max_cone_idx]
                # 가장 큰 콘을 빨간색 원으로 표시
                cv2.circle(result_img, (max_cx, max_cy), 20, (0, 0, 255), 3)
                cv2.putText(result_img, f"MAX: {max_area:.0f}px", (max_cx - 50, max_cy - 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            # ===== 콘별 상세 정보 로그 =====
            log_str = f"\n{'='*60}\n[Yellow Cones Pixel Analysis]\n"
            log_str += f"Total Cones: {yellow_count}\n"
            log_str += f"Max Pixel Area: {max_area:.0f}px\n"
            log_str += f"{'-'*60}\n"
            
            # 면적 내림차순 정렬
            cone_info_sorted = sorted(cone_info, key=lambda x: x[0], reverse=True)
            
            for idx, (area, cx, cy, _) in enumerate(cone_info_sorted, 1):
                log_str += f"Cone #{idx}: {area:.0f}px (center: {cx}, {cy})\n"
            
            log_str += f"{'='*60}\n"
            
            # ========================================
            # 6. 4개의 이미지를 합쳐서 표시
            # ========================================
            
            # 크기 조정 (표시 목적)
            display_size = (height//2, width//2)
            
            img_resized = cv2.resize(img, display_size)
            mask_hsv_resized = cv2.resize(cv2.cvtColor(mask_hsv, cv2.COLOR_GRAY2BGR), display_size)
            mask_morph_resized = cv2.resize(cv2.cvtColor(mask_morphology, cv2.COLOR_GRAY2BGR), display_size)
            mask_dilate_resized = cv2.resize(cv2.cvtColor(mask_dilate, cv2.COLOR_GRAY2BGR), display_size)
            
            # 라벨 추가
            cv2.putText(img_resized, "1. Original", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(mask_hsv_resized, "2. HSV Filter", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(mask_morph_resized, "3. Morphology", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(mask_dilate_resized, "4. Dilate", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 4개 이미지 합치기
            top_row = np.hstack([img_resized, mask_hsv_resized])
            bottom_row = np.hstack([mask_morph_resized, mask_dilate_resized])
            combined = np.vstack([top_row, bottom_row])
            
            # 윈도우 표시
            cv2.imshow("Yellow Filter Debug - 4-Step Process", combined)
            cv2.imshow("Yellow Cones Detection Result", result_img)
            
            # ESC 키로 종료
            key = cv2.waitKey(1)
            if key == 27:  # ESC
                rospy.loginfo("Debug tool closed by user")
                rospy.signal_shutdown("User closed debug window")
            
            # 로그 출력
            rospy.loginfo_throttle(1.0, log_str + f"HSV pixels: {cv2.countNonZero(mask_hsv)} | "
                                       f"After Morphology: {cv2.countNonZero(mask_morphology)} | "
                                       f"After Dilate: {cv2.countNonZero(mask_dilate)}")
            
        except Exception as e:
            rospy.logerr(f"Error: {e}")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = YellowFilterDebug()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
