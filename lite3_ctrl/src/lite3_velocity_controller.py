#!/usr/bin/env python3
"""
Lite3 Velocity Controller Node

Converts EGO planner trajectory commands (position, velocity, acceleration)
to velocity commands for Deep Robotics Lite3 quadruped robot.

The node:
1. Subscribes to /drone_0_planning/pos_cmd (quadrotor_msgs/PositionCommand)
2. Extracts velocity commands from the trajectory
3. Publishes velocity commands to /lite3/cmd_vel (geometry_msgs/Twist)

Features:
- Velocity feedforward from planner
- Position error feedback for tracking
- Velocity smoothing and limiting
- Safety checks and emergency stop
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
import tf2_ros
import tf2_geometry_msgs
from collections import deque


class Lite3VelocityController:
    def __init__(self):
        rospy.init_node('lite3_velocity_controller', anonymous=False)
        
        # Parameters
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 1.0)  # m/s
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.0)  # rad/s
        self.position_gain = rospy.get_param('~position_gain', 0.5)  # P gain for position tracking
        self.velocity_alpha = rospy.get_param('~velocity_alpha', 0.3)  # Smoothing factor (0-1)
        self.control_rate = rospy.get_param('~control_rate', 50.0)  # Hz
        self.odom_timeout = rospy.get_param('~odom_timeout', 0.5)  # seconds
        self.cmd_timeout = rospy.get_param('~cmd_timeout', 1.0)  # seconds
        
        # State variables
        self.current_odom = None
        self.last_odom_time = None
        self.last_pos_cmd = None
        self.last_cmd_time = None
        self.filtered_vel = np.zeros(3)
        self.last_yaw = 0.0
        
        # Publishers
        self.vel_pub = rospy.Publisher('/lite3/cmd_vel', Twist, queue_size=10)
        
        # Subscribers
        self.pos_cmd_sub = rospy.Subscriber(
            '/drone_0_planning/pos_cmd', 
            PositionCommand, 
            self.pos_cmd_callback, 
            queue_size=10
        )
        
        self.odom_sub = rospy.Subscriber(
            '/leg_odom',
            Odometry,
            self.odom_callback,
            queue_size=10
        )
        
        # Timer for control loop
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate),
            self.control_loop
        )
        
        rospy.loginfo("Lite3 Velocity Controller initialized")
        rospy.loginfo(f"  Max linear velocity: {self.max_linear_vel} m/s")
        rospy.loginfo(f"  Max angular velocity: {self.max_angular_vel} rad/s")
        rospy.loginfo(f"  Position gain: {self.position_gain}")
        rospy.loginfo(f"  Control rate: {self.control_rate} Hz")
        
    def odom_callback(self, msg):
        """Store current odometry for state feedback"""
        self.current_odom = msg
        self.last_odom_time = rospy.Time.now()
        
    def pos_cmd_callback(self, msg):
        """Receive position command from EGO planner"""
        self.last_pos_cmd = msg
        self.last_cmd_time = rospy.Time.now()
        
    def control_loop(self, event):
        """Main control loop - converts position commands to velocity commands"""
        
        # Safety checks
        if not self.check_safety():
            self.publish_zero_velocity()
            return
            
        if self.last_pos_cmd is None or self.current_odom is None:
            return
            
        # Extract current state
        current_pos = np.array([
            self.current_odom.pose.pose.position.x,
            self.current_odom.pose.pose.position.y,
            self.current_odom.pose.pose.position.z
        ])
        
        current_vel = np.array([
            self.current_odom.twist.twist.linear.x,
            self.current_odom.twist.twist.linear.y,
            self.current_odom.twist.twist.linear.z
        ])
        
        # Extract desired state from planner
        desired_pos = np.array([
            self.last_pos_cmd.position.x,
            self.last_pos_cmd.position.y,
            self.last_pos_cmd.position.z
        ])
        
        desired_vel = np.array([
            self.last_pos_cmd.velocity.x,
            self.last_pos_cmd.velocity.y,
            self.last_pos_cmd.velocity.z
        ])
        
        # Compute position error
        pos_error = desired_pos - current_pos
        
        # Velocity command = feedforward + position feedback
        # For quadruped, we ignore Z (height is handled by robot's own controller)
        vel_cmd = desired_vel[:2] + self.position_gain * pos_error[:2]
        
        # Apply velocity smoothing (exponential moving average)
        self.filtered_vel[:2] = (self.velocity_alpha * vel_cmd + 
                                 (1 - self.velocity_alpha) * self.filtered_vel[:2])
        
        # Limit velocity
        vel_magnitude = np.linalg.norm(self.filtered_vel[:2])
        if vel_magnitude > self.max_linear_vel:
            self.filtered_vel[:2] = (self.filtered_vel[:2] / vel_magnitude * 
                                     self.max_linear_vel)
        
        # Compute yaw command (face direction of motion)
        desired_yaw = self.last_pos_cmd.yaw
        yaw_rate = self.last_pos_cmd.yaw_dot
        
        # Limit yaw rate
        yaw_rate = np.clip(yaw_rate, -self.max_angular_vel, self.max_angular_vel)
        
        # Publish velocity command
        self.publish_velocity(self.filtered_vel[:2], yaw_rate)
        
        # Logging (throttled)
        if rospy.Time.now().to_sec() % 2.0 < 0.02:  # Log every 2 seconds
            rospy.loginfo(f"Vel cmd: [{self.filtered_vel[0]:.2f}, {self.filtered_vel[1]:.2f}] m/s, "
                         f"yaw_rate: {yaw_rate:.2f} rad/s, "
                         f"pos_error: [{pos_error[0]:.2f}, {pos_error[1]:.2f}] m")
    
    def check_safety(self):
        """Check safety conditions"""
        now = rospy.Time.now()
        
        # Check odometry timeout
        if self.last_odom_time is None:
            rospy.logwarn_throttle(5.0, "No odometry received yet")
            return False
            
        if (now - self.last_odom_time).to_sec() > self.odom_timeout:
            rospy.logwarn_throttle(1.0, "Odometry timeout - stopping robot")
            return False
        
        # Check command timeout
        if self.last_cmd_time is None:
            return False
            
        if (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
            rospy.logwarn_throttle(1.0, "Position command timeout - stopping robot")
            return False
            
        return True
    
    def publish_velocity(self, linear_vel, angular_vel):
        """Publish velocity command to Lite3"""
        cmd = Twist()
        cmd.linear.x = float(linear_vel[0])
        cmd.linear.y = float(linear_vel[1])
        cmd.linear.z = 0.0  # Height controlled by robot
        
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = float(angular_vel)
        
        self.vel_pub.publish(cmd)
    
    def publish_zero_velocity(self):
        """Send zero velocity (stop command)"""
        cmd = Twist()
        self.vel_pub.publish(cmd)
        self.filtered_vel = np.zeros(3)
    
    def shutdown(self):
        """Cleanup on shutdown"""
        rospy.loginfo("Shutting down Lite3 velocity controller - sending stop command")
        self.publish_zero_velocity()


if __name__ == '__main__':
    try:
        controller = Lite3VelocityController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
