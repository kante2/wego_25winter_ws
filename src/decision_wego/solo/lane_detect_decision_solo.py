#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lane Detection Decision Node for WEGO
- Receives perception data (steering_offset, lane_speed)
- Receives stop flag from traffic light
- Controls motor via AckermannDriveStamped
"""
import rospy
from std_msgs.msg import Float32, Bool
from ackermann_msgs.msg import AckermannDriveStamped


class LaneDetectDecision:
    def __init__(self):
        rospy.init_node('lane_detect_decision', anonymous=True)
        
        # Current perception data
        self.steering_offset = 0.0
        self.lane_speed = 0.3
        self.stop_flag = False
        
        # Publisher - motor control
        self.cmd_vel_pub = rospy.Publisher('/low_level/ackermann_cmd_mux/input/navigation', 
                                            AckermannDriveStamped, queue_size=1)
        
        # Subscribers - perception data
        self.sub_steering = rospy.Subscriber('/webot/steering_offset', Float32, 
                                             self.steering_callback, queue_size=1)
        self.sub_speed = rospy.Subscriber('/webot/lane_speed', Float32, 
                                          self.speed_callback, queue_size=1)
        self.sub_stop = rospy.Subscriber('/webot/traffic_stop', Bool, 
                                         self.stop_callback, queue_size=1)
        
        # Timer for control loop
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        
        rospy.loginfo("="*50)
        rospy.loginfo("Lane Detect Decision Node initialized")
        rospy.loginfo("Listening to:")
        rospy.loginfo("  - /webot/steering_offset")
        rospy.loginfo("  - /webot/lane_speed")
        rospy.loginfo("  - /webot/traffic_stop")
        rospy.loginfo("Publishing to:")
        rospy.loginfo("  - /low_level/ackermann_cmd_mux/input/navigation")
        rospy.loginfo("="*50)
    
    def steering_callback(self, msg):
        """Receive steering offset from perception"""
        self.steering_offset = msg.data
    
    def speed_callback(self, msg):
        """Receive speed from perception"""
        self.lane_speed = msg.data
    
    def stop_callback(self, msg):
        """Receive stop flag from traffic light or other nodes"""
        self.stop_flag = msg.data
        if self.stop_flag:
            rospy.loginfo_throttle(2, "[LaneDecision] STOP FLAG ACTIVE")
    
    def control_loop(self, event):
        """Main control loop - send motor commands"""
        # Create AckermannDriveStamped message
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = rospy.Time.now()
        ackermann_msg.header.frame_id = "base_link"
        
        # Check stop flag
        if self.stop_flag:
            # Stop the robot
            ackermann_msg.drive.speed = 0.0
            ackermann_msg.drive.steering_angle = 0.0
        else:
            # Normal operation - use perception data
            ackermann_msg.drive.speed = self.lane_speed
            ackermann_msg.drive.steering_angle = self.steering_offset
        
        # Publish motor command
        self.cmd_vel_pub.publish(ackermann_msg)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = LaneDetectDecision()
        node.run()
    except rospy.ROSInterruptException:
        pass