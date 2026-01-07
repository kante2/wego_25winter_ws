#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2 as cv
import numpy as np
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import PointStamped
from dynamic_reconfigure.server import Server
from wego_cfg.cfg import LaneDetectConfig

class LaneDetectPerception:
    def __init__(self):
        rospy.init_node('lane_detect_perception')
        self.cv_bridge = CvBridge()

        # Parameters
        self.debug_view = rospy.get_param("~debug_view", True)
        self.config = None
        
        # Dynamic Reconfigure
        self.srv = Server(LaneDetectConfig, self.reconfigure_callback)
        
        # Subscribers
        self.image_sub = rospy.Subscriber(
            '/usb_cam/image_raw/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        # Publishers
        self.pub_center_x = rospy.Publisher('/webot/lane_center_x', Int32, queue_size=1)
        self.pub_center = rospy.Publisher('/webot/lane_center', PointStamped, queue_size=1)
        
        if self.debug_view:
            self.pub_binary = rospy.Publisher('/perception/lane_binary', Image, queue_size=1)
            self.pub_sliding = rospy.Publisher('/perception/lane_sliding', Image, queue_size=1)

        # HSV thresholds for white lane
        self.white_lower = np.array([0, 0, 180], dtype=np.uint8)
        self.white_upper = np.array([180, 40, 255], dtype=np.uint8)

        # Load calibration
        self.camera_matrix, self.dist_coeffs = self._load_calibration()

        # Warp matrices (same as dh_lanefollow)
        self.src_points = np.float32([
            [0, 310],
            [640, 310],
            [0, 480],
            [640, 480]
        ])
        self.dst_points = np.float32([
            [0, 310],
            [640, 310],
            [225, 480],
            [415, 480]
        ])
        self.warp_mat = cv.getPerspectiveTransform(self.src_points, self.dst_points)
        
        self.bgr = None
        self.gaussian_sigma = 1

        rospy.loginfo("Lane Detect Perception Node Initialized")

    def reconfigure_callback(self, config, level):
        self.config = config
        rospy.loginfo(f"[LaneDetectPerception] Config updated: masked_pixel={config.masked_pixel}")
        return config

    def _load_calibration(self):
        try:
            calib_file = rospy.get_param('~calibration_file',
                '/home/wego/catkin_ws/src/usb_cam/calibration/usb_cam.yaml')
            with open(calib_file, 'r') as f:
                calib = yaml.safe_load(f)
            camera_matrix = np.array(calib['camera_matrix']['data']).reshape(3, 3)
            dist_coeffs = np.array(calib['distortion_coefficients']['data'])
            rospy.loginfo("Calibration loaded")
            return camera_matrix, dist_coeffs
        except:
            rospy.logwarn("Calibration load failed")
            return None, None

    def undistort(self, img):
        if self.camera_matrix is None:
            return img
        h, w = img.shape[:2]
        new_K = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.camera_matrix, self.dist_coeffs, (w, h), np.eye(3), balance=0.0
        )
        return cv.fisheye.undistortImage(
            img, self.camera_matrix, self.dist_coeffs, Knew=new_K
        )

    def warpping(self, img):
        h, w = img.shape[:2]
        return cv.warpPerspective(img, self.warp_mat, (w, h))

    def roi_set(self, img):
        return img[310:480, 0:640]

    def Gaussian_filter(self, img):
        return cv.GaussianBlur(img, (0, 0), self.gaussian_sigma)

    def white_color_filter_hsv(self, img):
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        return cv.inRange(hsv, self.white_lower, self.white_upper)

    def sliding_window_right(self, img, n_windows=10, margin=12, minpix=5):
        """오른쪽 차선 기반 슬라이딩 윈도우 (dh_lanefollow 방식)"""
        y = img.shape[0]
        x = img.shape[1]
        
        # 히스토그램 영역 (아래쪽 절반)
        hist_area = np.copy(img[y // 2:, :])
        
        # 중앙 마스킹 (cfg에서 가져오기)
        center_x = x // 2
        mask_width = self.config.masked_pixel if self.config else 30
        start_col = center_x - (mask_width // 2)
        end_col = center_x + (mask_width // 2) + (mask_width % 2)
        hist_area[:, start_col:end_col] = 0
        
        # 히스토그램 계산
        histogram = np.sum(hist_area, axis=0)
        midpoint = int(histogram.shape[0] / 2)
        
        # 오른쪽 차선 시작점
        if sum(histogram[midpoint:]) < 15:
            rightx_current = midpoint * 2
        else:
            rightx_current = np.argmax(histogram[midpoint:]) + midpoint
        
        window_height = int(y / n_windows)
        nz = img.nonzero()
        
        right_lane_inds = []
        rx, ry = [], []
        
        out_img = np.dstack((img, img, img)) * 255
        
        for window in range(n_windows):
            win_yl = y - (window + 1) * window_height
            win_yh = y - window * window_height
            win_xrl = rightx_current - margin
            win_xrh = rightx_current + margin
            
            cv.rectangle(out_img, (win_xrl, win_yl), (win_xrh, win_yh), (0, 255, 0), 2)
            
            good_right_inds = ((nz[0] >= win_yl) & (nz[0] < win_yh) &
                              (nz[1] >= win_xrl) & (nz[1] < win_xrh)).nonzero()[0]
            
            right_lane_inds.append(good_right_inds)
            
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nz[1][good_right_inds]))
            
            rx.append(rightx_current)
            ry.append((win_yl + win_yh) / 2)
        
        right_lane_inds = np.concatenate(right_lane_inds)
        rfit = np.polyfit(np.array(ry), np.array(rx), 1)
        
        out_img[nz[0][right_lane_inds], nz[1][right_lane_inds]] = [0, 0, 255]
        
        if self.debug_view:
            self.pub_sliding.publish(self.cv_bridge.cv2_to_imgmsg(out_img, encoding="bgr8"))
        
        return rfit

    def cal_center_from_right(self, rfit):
        """오른쪽 차선에서 중앙선 계산 (dh_lanefollow 방식)"""
        a, b = rfit
        # 오른쪽 차선에서 120px 왼쪽이 중앙선
        cfit = [a, b - 120]
        
        h, w = 170, 640
        y_eval = h * 0.75
        
        x_center = cfit[0] * y_eval + cfit[1]
        
        return int(x_center)

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
            if cv_image is None:
                return
            
            self.bgr = self.undistort(cv_image)
            
            # Process
            warp_img_ori = self.warpping(self.bgr)
            warp_img = self.roi_set(warp_img_ori)
            g_filtered = self.Gaussian_filter(warp_img)
            white_img = self.white_color_filter_hsv(g_filtered)
            
            if self.debug_view:
                self.pub_binary.publish(self.cv_bridge.cv2_to_imgmsg(white_img, encoding="mono8"))
            
            # 오른쪽 차선 검출
            rfit = self.sliding_window_right(white_img)
            x_center = self.cal_center_from_right(rfit)
            
            # Publish
            self.pub_center_x.publish(Int32(x_center))
            
            point_msg = PointStamped()
            point_msg.header.stamp = rospy.Time.now()
            point_msg.header.frame_id = "camera"
            point_msg.point.x = float(x_center)
            point_msg.point.y = 0.0
            point_msg.point.z = 0.0
            self.pub_center.publish(point_msg)
            
        except Exception as e:
            rospy.logerr(f"[LanePerception] Error: {e}")

if __name__ == '__main__':
    try:
        node = LaneDetectPerception()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass