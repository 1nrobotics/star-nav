#!/usr/bin/env python3
"""
Simple walking test for Lite3 quadruped robot.
This script makes the robot walk forward, backward, turn, and strafe to test basic locomotion.
"""

import rospy
from geometry_msgs.msg import Twist
import time

class Lite3WalkTest:
    def __init__(self):
        rospy.init_node('lite3_walk_test', anonymous=True)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Control parameters
        self.rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Lite3 Walk Test Node Initialized")
        rospy.loginfo("Publishing to /cmd_vel")
        
    def send_velocity(self, linear_x, linear_y, angular_z, duration):
        """
        Send velocity command for a specified duration.
        
        Args:
            linear_x: Forward velocity (m/s, positive = forward)
            linear_y: Lateral velocity (m/s, positive = left)
            angular_z: Yaw rate (rad/s, positive = left turn)
            duration: How long to maintain this velocity (seconds)
        """
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_z
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
            
    def stop(self, duration=1.0):
        """Stop the robot for specified duration."""
        rospy.loginfo("STOP")
        self.send_velocity(0.0, 0.0, 0.0, duration)
        
    def walk_forward(self, speed=0.3, duration=3.0):
        """Walk forward at specified speed."""
        rospy.loginfo(f"Walking forward at {speed} m/s for {duration}s")
        self.send_velocity(speed, 0.0, 0.0, duration)
        
    def walk_backward(self, speed=0.3, duration=3.0):
        """Walk backward at specified speed."""
        rospy.loginfo(f"Walking backward at {speed} m/s for {duration}s")
        self.send_velocity(-speed, 0.0, 0.0, duration)
        
    def strafe_left(self, speed=0.2, duration=3.0):
        """Strafe left at specified speed."""
        rospy.loginfo(f"Strafing left at {speed} m/s for {duration}s")
        self.send_velocity(0.0, speed, 0.0, duration)
        
    def strafe_right(self, speed=0.2, duration=3.0):
        """Strafe right at specified speed."""
        rospy.loginfo(f"Strafing right at {speed} m/s for {duration}s")
        self.send_velocity(0.0, -speed, 0.0, duration)
        
    def turn_left(self, angular_speed=0.5, duration=3.0):
        """Turn left (counterclockwise) at specified angular speed."""
        rospy.loginfo(f"Turning left at {angular_speed} rad/s for {duration}s")
        self.send_velocity(0.0, 0.0, angular_speed, duration)
        
    def turn_right(self, angular_speed=0.5, duration=3.0):
        """Turn right (clockwise) at specified angular speed."""
        rospy.loginfo(f"Turning right at {angular_speed} rad/s for {duration}s")
        self.send_velocity(0.0, 0.0, -angular_speed, duration)
        
    def circle(self, linear_speed=0.3, angular_speed=0.3, duration=5.0):
        """Walk in a circle."""
        rospy.loginfo(f"Walking in circle for {duration}s")
        self.send_velocity(linear_speed, 0.0, angular_speed, duration)
        
    def run_basic_test(self):
        """Run a basic walking test sequence."""
        rospy.loginfo("="*50)
        rospy.loginfo("Starting Basic Walking Test")
        rospy.loginfo("="*50)
        
        try:
            # Wait for robot to be ready
            rospy.loginfo("Starting in 3 seconds...")
            rospy.sleep(3.0)
            
            # Test 1: Walk forward
            self.walk_forward(speed=0.3, duration=3.0)
            self.stop(duration=1.0)
            
            # Test 2: Walk backward
            self.walk_backward(speed=0.3, duration=3.0)
            self.stop(duration=1.0)
            
            # Test 3: Turn left
            self.turn_left(angular_speed=0.5, duration=2.0)
            self.stop(duration=1.0)
            
            # Test 4: Turn right
            self.turn_right(angular_speed=0.5, duration=2.0)
            self.stop(duration=1.0)
            
            # Test 5: Strafe left
            self.strafe_left(speed=0.2, duration=2.0)
            self.stop(duration=1.0)
            
            # Test 6: Strafe right
            self.strafe_right(speed=0.2, duration=2.0)
            self.stop(duration=1.0)
            
            # Test 7: Walk in circle
            self.circle(linear_speed=0.3, angular_speed=0.3, duration=5.0)
            self.stop(duration=1.0)
            
            rospy.loginfo("="*50)
            rospy.loginfo("Basic Walking Test Completed!")
            rospy.loginfo("="*50)
            
        except rospy.ROSInterruptException:
            rospy.loginfo("Test interrupted by user")
        finally:
            # Always stop at the end
            self.stop(duration=1.0)
            
    def run_simple_forward_test(self):
        """Run a simple forward walking test."""
        rospy.loginfo("="*50)
        rospy.loginfo("Starting Simple Forward Walking Test")
        rospy.loginfo("="*50)
        
        try:
            rospy.loginfo("Starting in 3 seconds...")
            rospy.sleep(3.0)
            
            # Walk forward at slow speed
            self.walk_forward(speed=0.2, duration=5.0)
            self.stop(duration=2.0)
            
            rospy.loginfo("="*50)
            rospy.loginfo("Simple Forward Test Completed!")
            rospy.loginfo("="*50)
            
        except rospy.ROSInterruptException:
            rospy.loginfo("Test interrupted by user")
        finally:
            self.stop(duration=1.0)

def main():
    try:
        walker = Lite3WalkTest()
        
        # Check which test to run (can be set via launch file param)
        test_mode = rospy.get_param('~test_mode', 'simple')  # 'simple' or 'full'
        
        if test_mode == 'full':
            walker.run_basic_test()
        else:
            walker.run_simple_forward_test()
            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
