#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Obstacle Avoidance Decision Node for WEGO (Gap Finding Algorithm)
- Receives perception data (gap_angle, gap_width, min_distance)
- Receives lane data (steering_offset, lane_speed)
- Decides when to avoid vs lane trace
- Controls motor via AckermannDriveStamped
"""

import rospy
import numpy as np
from std_msgs.msg import Float32, Int32, String, Bool
from ackermann_msgs.msg import AckermannDriveStamped
from dynamic_reconfigure.server import Server
from wego.cfg import ObstacleAvoidConfig


class ObstacleAvoidDecision:
    def __init__(self):
        rospy.init_node('obstacle_avoid_decision')
        
        # Parameters
        self.safe_distance = rospy.get_param('~safe_distance', 0.5)
        self.stop_distance = rospy.get_param('~stop_distance', 0.2)
        self.avoid_speed = rospy.get_param('~avoid_speed', 0.2)
        self.max_steering = rospy.get_param('~max_steering', 0.5)
        self.steering_gain = rospy.get_param('~steering_gain', 0.02)
        self.clear_threshold = rospy.get_param('~clear_threshold', 20)
        
        # Current perception data
        self.gap_angle = 0.0
        self.gap_width = 1
        self.min_distance = 10.0
        
        # Current lane data
        self.lane_steering = 0.0
        self.lane_speed = 0.3
        
        # State
        self.avoiding = False
        self.clear_count = 0
        self.last_state = ""
        
        # Dynamic reconfigure
        self.srv = Server(ObstacleAvoidConfig, self.reconfigure_callback)
        
        # Publisher - motor control
        self.cmd_vel_pub = rospy.Publisher('/low_level/ackermann_cmd_mux/input/navigation', 
                                            AckermannDriveStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/webot/obstacle/state', String, queue_size=1)
        self.avoiding_pub = rospy.Publisher('/webot/obstacle/is_avoiding', Bool, queue_size=1)
        
        # Subscribers - perception data
        self.sub_gap_angle = rospy.Subscriber('/webot/obstacle/gap_angle', Float32, 
                                              self.gap_angle_callback, queue_size=1)
        self.sub_gap_width = rospy.Subscriber('/webot/obstacle/gap_width', Int32, 
                                              self.gap_width_callback, queue_size=1)
        self.sub_min_distance = rospy.Subscriber('/webot/obstacle/min_distance', Float32, 
                                                 self.min_distance_callback, queue_size=1)
        
        # Subscribers - lane data
        self.sub_steering = rospy.Subscriber('/webot/steering_offset', Float32, 
                                             self.steering_callback, queue_size=1)
        self.sub_speed = rospy.Subscriber('/webot/lane_speed', Float32, 
                                          self.speed_callback, queue_size=1)
        
        # Timer for control loop
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        
        rospy.loginfo("="*50)
        rospy.loginfo("Obstacle Avoid Decision Node initialized (Gap Finding)")
        rospy.loginfo("Listening to:")
        rospy.loginfo("  - /webot/obstacle/gap_angle")
        rospy.loginfo("  - /webot/obstacle/gap_width")
        rospy.loginfo("  - /webot/obstacle/min_distance")
        rospy.loginfo("  - /webot/steering_offset")
        rospy.loginfo("  - /webot/lane_speed")
        rospy.loginfo("Publishing to:")
        rospy.loginfo("  - /low_level/ackermann_cmd_mux/input/navigation")
        rospy.loginfo("="*50)
    
    def gap_angle_callback(self, msg):
        """Receive gap angle from perception"""
        self.gap_angle = msg.data
    
    def gap_width_callback(self, msg):
        """Receive gap width from perception"""
        self.gap_width = msg.data
    
    def min_distance_callback(self, msg):
        """Receive minimum distance from perception"""
        self.min_distance = msg.data
    
    def steering_callback(self, msg):
        """Receive steering offset from lane detection"""
        self.lane_steering = msg.data
    
    def speed_callback(self, msg):
        """Receive speed from lane detection"""
        self.lane_speed = msg.data
    
    def reconfigure_callback(self, config, level):
        self.safe_distance = config.safe_distance
        self.stop_distance = config.stop_distance
        self.avoid_speed = config.avoid_speed
        self.max_steering = config.max_steering
        self.steering_gain = config.steering_gain
        self.clear_threshold = config.clear_threshold
        rospy.loginfo(f"[ObstacleDecision] Config updated: avoid_speed={self.avoid_speed}, steering_gain={self.steering_gain}")
        return config
    
    def _has_obstacle_in_front(self):
        """Check if there's an obstacle in front"""
        return self.min_distance < self.safe_distance
    
    def control_loop(self, event):
        """Main control loop - make decision and send motor commands"""
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = rospy.Time.now()
        ackermann_msg.header.frame_id = "base_link"
        
        state = "IDLE"
        
        # Too close - emergency stop
        if self.min_distance < self.stop_distance:
            ackermann_msg.drive.speed = 0.0
            ackermann_msg.drive.steering_angle = 0.0
            state = "TOO_CLOSE"
            self.avoiding = True
            self.clear_count = 0
        
        # Obstacle in front - start avoiding
        elif self._has_obstacle_in_front():
            self.avoiding = True
            self.clear_count = 0
            
            # Use gap angle to steer toward the best gap
            steering = -self.steering_gain * self.gap_angle
            steering = np.clip(steering, -self.max_steering, self.max_steering)
            
            ackermann_msg.drive.speed = self.avoid_speed
            ackermann_msg.drive.steering_angle = steering
            state = f"AVOIDING (gap={self.gap_angle:.1f}°, width={self.gap_width})"
        
        # Avoiding mode - continue avoiding while checking for obstacles
        elif self.avoiding:
            # Continue avoiding, check if obstacle is cleared
            steering = -self.steering_gain * self.gap_angle
            steering = np.clip(steering, -self.max_steering, self.max_steering)
            
            ackermann_msg.drive.speed = self.avoid_speed
            ackermann_msg.drive.steering_angle = steering
            
            # Increment clear counter
            self.clear_count += 1
            
            if self.clear_count >= self.clear_threshold:
                # Obstacle is cleared for long enough - return to lane tracing
                self.avoiding = False
                self.clear_count = 0
                ackermann_msg.drive.speed = self.lane_speed
                ackermann_msg.drive.steering_angle = self.lane_steering
                state = "LANE_TRACING"
            else:
                state = f"AVOIDING_CLEAR ({self.clear_count}/{self.clear_threshold})"
        
        # Clear - normal lane tracing
        else:
            ackermann_msg.drive.speed = self.lane_speed
            ackermann_msg.drive.steering_angle = self.lane_steering
            state = "LANE_TRACING"
        
        # Log state changes
        if state != self.last_state:
            rospy.loginfo(f"[ObstacleDecision] {state}")
            self.last_state = state
        
        # Publish motor command
        self.cmd_vel_pub.publish(ackermann_msg)
        self.state_pub.publish(String(data=state))
        self.avoiding_pub.publish(Bool(data=self.avoiding))
        
        rospy.loginfo_throttle(1, f"[ObstacleDecision] {state} | Speed: {ackermann_msg.drive.speed:.2f} | Min: {self.min_distance:.2f}m")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ObstacleAvoidDecision()
        node.run()
    except rospy.ROSInterruptException:
        pass
