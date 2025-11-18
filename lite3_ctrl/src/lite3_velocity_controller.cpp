/**
 * @file lite3_velocity_controller.cpp
 * @brief Velocity controller for Deep Robotics Lite3 quadruped robot
 * 
 * Converts EGO planner trajectory commands to velocity commands for Lite3.
 * Implements velocity feedforward + position feedback control.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <Eigen/Dense>
#include <deque>

class Lite3VelocityController {
public:
    Lite3VelocityController(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
        : nh_(nh), pnh_(pnh) {
        
        // Load parameters
        pnh_.param<double>("max_linear_vel", max_linear_vel_, 1.0);
        pnh_.param<double>("max_angular_vel", max_angular_vel_, 1.0);
        pnh_.param<double>("position_gain", position_gain_, 0.5);
        pnh_.param<double>("velocity_alpha", velocity_alpha_, 0.3);
        pnh_.param<double>("control_rate", control_rate_, 50.0);
        pnh_.param<double>("odom_timeout", odom_timeout_, 0.5);
        pnh_.param<double>("cmd_timeout", cmd_timeout_, 1.0);
        
        // Initialize state
        filtered_vel_.setZero();
        has_odom_ = false;
        has_pos_cmd_ = false;
        
        // Publishers
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/lite3/cmd_vel", 10);
        
        // Subscribers
        pos_cmd_sub_ = nh_.subscribe("/drone_0_planning/pos_cmd", 10,
                                     &Lite3VelocityController::posCmdCallback, this);
        odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10,
                                  &Lite3VelocityController::odomCallback, this);
        
        // Control timer
        control_timer_ = nh_.createTimer(
            ros::Duration(1.0 / control_rate_),
            &Lite3VelocityController::controlLoop, this);
        
        ROS_INFO("Lite3 Velocity Controller initialized (C++)");
        ROS_INFO("  Max linear velocity: %.2f m/s", max_linear_vel_);
        ROS_INFO("  Max angular velocity: %.2f rad/s", max_angular_vel_);
        ROS_INFO("  Position gain: %.2f", position_gain_);
        ROS_INFO("  Control rate: %.1f Hz", control_rate_);
    }
    
    ~Lite3VelocityController() {
        publishZeroVelocity();
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher vel_pub_;
    ros::Subscriber pos_cmd_sub_;
    ros::Subscriber odom_sub_;
    ros::Timer control_timer_;
    
    // Parameters
    double max_linear_vel_;
    double max_angular_vel_;
    double position_gain_;
    double velocity_alpha_;
    double control_rate_;
    double odom_timeout_;
    double cmd_timeout_;
    
    // State
    nav_msgs::Odometry current_odom_;
    quadrotor_msgs::PositionCommand last_pos_cmd_;
    ros::Time last_odom_time_;
    ros::Time last_cmd_time_;
    Eigen::Vector3d filtered_vel_;
    bool has_odom_;
    bool has_pos_cmd_;
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_odom_ = *msg;
        last_odom_time_ = ros::Time::now();
        has_odom_ = true;
    }
    
    void posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        last_pos_cmd_ = *msg;
        last_cmd_time_ = ros::Time::now();
        has_pos_cmd_ = true;
    }
    
    void controlLoop(const ros::TimerEvent& event) {
        // Safety checks
        if (!checkSafety()) {
            publishZeroVelocity();
            return;
        }
        
        if (!has_odom_ || !has_pos_cmd_) {
            return;
        }
        
        // Extract current state
        Eigen::Vector3d current_pos(
            current_odom_.pose.pose.position.x,
            current_odom_.pose.pose.position.y,
            current_odom_.pose.pose.position.z
        );
        
        Eigen::Vector3d current_vel(
            current_odom_.twist.twist.linear.x,
            current_odom_.twist.twist.linear.y,
            current_odom_.twist.twist.linear.z
        );
        
        // Extract desired state from planner
        Eigen::Vector3d desired_pos(
            last_pos_cmd_.position.x,
            last_pos_cmd_.position.y,
            last_pos_cmd_.position.z
        );
        
        Eigen::Vector3d desired_vel(
            last_pos_cmd_.velocity.x,
            last_pos_cmd_.velocity.y,
            last_pos_cmd_.velocity.z
        );
        
        // Compute position error
        Eigen::Vector3d pos_error = desired_pos - current_pos;
        
        // Velocity command = feedforward + position feedback
        // For quadruped, we only control X and Y (ignore Z)
        Eigen::Vector2d vel_cmd = desired_vel.head<2>() + position_gain_ * pos_error.head<2>();
        
        // Apply velocity smoothing (exponential moving average)
        filtered_vel_.head<2>() = velocity_alpha_ * vel_cmd + 
                                  (1.0 - velocity_alpha_) * filtered_vel_.head<2>();
        
        // Limit velocity
        double vel_magnitude = filtered_vel_.head<2>().norm();
        if (vel_magnitude > max_linear_vel_) {
            filtered_vel_.head<2>() = filtered_vel_.head<2>() / vel_magnitude * max_linear_vel_;
        }
        
        // Compute yaw command
        double yaw_rate = last_pos_cmd_.yaw_dot;
        
        // Limit yaw rate
        yaw_rate = std::max(-max_angular_vel_, std::min(max_angular_vel_, yaw_rate));
        
        // Publish velocity command
        publishVelocity(filtered_vel_.head<2>(), yaw_rate);
        
        // Logging (throttled)
        static ros::Time last_log_time = ros::Time::now();
        if ((ros::Time::now() - last_log_time).toSec() > 2.0) {
            ROS_INFO("Vel cmd: [%.2f, %.2f] m/s, yaw_rate: %.2f rad/s, pos_error: [%.2f, %.2f] m",
                     filtered_vel_(0), filtered_vel_(1), yaw_rate, 
                     pos_error(0), pos_error(1));
            last_log_time = ros::Time::now();
        }
    }
    
    bool checkSafety() {
        ros::Time now = ros::Time::now();
        
        // Check odometry timeout
        if (!has_odom_) {
            ROS_WARN_THROTTLE(5.0, "No odometry received yet");
            return false;
        }
        
        if ((now - last_odom_time_).toSec() > odom_timeout_) {
            ROS_WARN_THROTTLE(1.0, "Odometry timeout - stopping robot");
            return false;
        }
        
        // Check command timeout
        if (!has_pos_cmd_) {
            return false;
        }
        
        if ((now - last_cmd_time_).toSec() > cmd_timeout_) {
            ROS_WARN_THROTTLE(1.0, "Position command timeout - stopping robot");
            return false;
        }
        
        return true;
    }
    
    void publishVelocity(const Eigen::Vector2d& linear_vel, double angular_vel) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_vel(0);
        cmd.linear.y = linear_vel(1);
        cmd.linear.z = 0.0;  // Height controlled by robot
        
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = angular_vel;
        
        vel_pub_.publish(cmd);
    }
    
    void publishZeroVelocity() {
        geometry_msgs::Twist cmd;
        vel_pub_.publish(cmd);
        filtered_vel_.setZero();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lite3_velocity_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    Lite3VelocityController controller(nh, pnh);
    
    ros::spin();
    
    return 0;
}
