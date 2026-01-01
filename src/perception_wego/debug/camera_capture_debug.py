#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Image Capture Debug Script
- Subscribe to camera topic
- Display live feed
- Press 'a' to capture image
- Saves to timestamped files
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import os
from datetime import datetime


class CameraDebug:
    def __init__(self):
        rospy.init_node('camera_capture_debug', anonymous=True)
        
        self.bridge = CvBridge()
        self.current_image = None
        self.capture_count = 0
        
        # Create capture directory
        self.capture_dir = os.path.expanduser('~/catkin_ws/camera_captures')
        if not os.path.exists(self.capture_dir):
            os.makedirs(self.capture_dir)
            rospy.loginfo(f"Created capture directory: {self.capture_dir}")
        
        # Subscribe to camera
        self.sub_image = rospy.Subscriber(
            '/usb_cam/image_raw/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo("="*50)
        rospy.loginfo("Camera Capture Debug Started")
        rospy.loginfo("="*50)
        rospy.loginfo("Controls:")
        rospy.loginfo("  [a] - Capture image")
        rospy.loginfo("  [q] - Quit")
        rospy.loginfo(f"Captures saved to: {self.capture_dir}")
        rospy.loginfo("="*50)
    
    def image_callback(self, msg):
        """Process incoming compressed image"""
        try:
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if self.current_image is None:
                return
            
            # Display live feed
            display_img = self.current_image.copy()
            
            # Add info text
            cv2.putText(display_img, "Press 'a' to capture | 'q' to quit", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_img, f"Captured: {self.capture_count}", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Show timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            cv2.putText(display_img, timestamp, 
                       (10, display_img.shape[0] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            cv2.imshow("Camera Feed - Press 'a' to capture", display_img)
            
            # Handle key press
            key = cv2.waitKey(1) & 0xFF
            if key == ord('a'):
                self._capture_image()
            elif key == ord('q'):
                rospy.loginfo("Quitting...")
                rospy.signal_shutdown("User quit")
        
        except Exception as e:
            rospy.logwarn(f"Error processing image: {e}")
    
    def _capture_image(self):
        """Capture and save current image"""
        if self.current_image is None:
            rospy.logwarn("No image to capture yet")
            return
        
        try:
            # Create filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = f"capture_{timestamp}.jpg"
            filepath = os.path.join(self.capture_dir, filename)
            
            # Save image
            cv2.imwrite(filepath, self.current_image)
            self.capture_count += 1
            
            # Get image info
            h, w = self.current_image.shape[:2]
            size_kb = os.path.getsize(filepath) / 1024
            
            rospy.loginfo("="*50)
            rospy.loginfo(f"✓ Image captured: {filename}")
            rospy.loginfo(f"  Size: {w}x{h} pixels")
            rospy.loginfo(f"  File size: {size_kb:.1f} KB")
            rospy.loginfo(f"  Path: {filepath}")
            rospy.loginfo(f"  Total captured: {self.capture_count}")
            rospy.loginfo("="*50)
            
        except Exception as e:
            rospy.logerr(f"Failed to save image: {e}")
    
    def run(self):
        """Keep node running"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = CameraDebug()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
