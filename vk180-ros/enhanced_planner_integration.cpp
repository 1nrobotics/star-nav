#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <memory>
#include <iostream>
#include <vk_sdk/Sdk.hpp>
#include <vk_sdk/capnp/image.capnp.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct CameraExtrinsics {
    Eigen::Vector3d body_position;
    Eigen::Quaterniond body_rotation;
    Eigen::Matrix4d cam_to_body_transform;
    bool is_valid = false;
    
    void updateTransform() {
        cam_to_body_transform = Eigen::Matrix4d::Identity();
        cam_to_body_transform.block<3,3>(0,0) = body_rotation.toRotationMatrix();
        cam_to_body_transform.block<3,1>(0,3) = body_position;
    }
};

struct CameraIntrinsics {
    double fx, fy, cx, cy;
    double baseline;  // For stereo disparity to depth conversion
    bool is_valid = false;
};

class EnhancedPlannerIntegration {
public:
    EnhancedPlannerIntegration(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        // Publishers for planner
        planner_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("grid_map/odom", 10);
        planner_depth_pub_ = nh_.advertise<sensor_msgs::Image>("grid_map/depth", 10);
        
        // Debug publishers
        raw_depth_pub_ = nh_.advertise<sensor_msgs::Image>("vk180/raw_depth", 10);
        transformed_depth_pub_ = nh_.advertise<sensor_msgs::Image>("vk180/body_frame_depth", 10);
        
        // Parameters
        pnh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
        pnh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
        pnh_.param<bool>("use_body_frame_transform", use_body_frame_transform_, true);
        pnh_.param<double>("max_depth", max_depth_, 10.0);
        pnh_.param<double>("min_depth", min_depth_, 0.1);
        
        initializeEcal();
    }

    ~EnhancedPlannerIntegration() {
        if (visualkit_) {
            visualkit_->source().stop(false);
        }
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher planner_odom_pub_;
    ros::Publisher planner_depth_pub_;
    ros::Publisher raw_depth_pub_;
    ros::Publisher transformed_depth_pub_;
    
    std::string odom_frame_id_;
    std::string base_frame_id_;
    bool use_body_frame_transform_;
    double max_depth_, min_depth_;
    
    std::unique_ptr<vkc::VisualKit> visualkit_;
    
    // Camera calibration data
    CameraExtrinsics stereo1_l_extrinsics_;
    CameraIntrinsics stereo1_l_intrinsics_;
    
    void initializeEcal() {
        visualkit_ = vkc::VisualKit::create(std::nullopt);
        
        if (visualkit_ == nullptr) {
            ROS_ERROR("Failed to create visualkit connection.");
            return;
        }

        // Install receivers
        auto odom_receiver = std::make_unique<OdometryReceiver>(*this);
        visualkit_->source().install("S1/vio_odom", std::move(odom_receiver));

        auto image_receiver = std::make_unique<ImageReceiver>(*this);
        visualkit_->source().install("S1/stereo1_l", std::move(image_receiver));

        auto disparity_receiver = std::make_unique<DisparityReceiver>(*this);
        visualkit_->source().install("S1/stereo1_l/disparity", std::move(disparity_receiver));

        visualkit_->source().start();
        
        ROS_INFO("Enhanced Planner Integration initialized");
        ROS_INFO("Waiting for stereo1_l extrinsics extraction...");
    }

    // Extract extrinsics from stereo1_l image stream
    class ImageReceiver : public vkc::Receiver<vkc::Image> {
    public:
        explicit ImageReceiver(EnhancedPlannerIntegration& integration) : integration_(integration) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Image>>& message) override {
            auto image = message.payload.reader();
            
            // Only process stereo1_l for extrinsics
            std::string stream_name = image.getStreamName().cStr();
            if (stream_name == "S1/stereo1_l") {
                integration_.extractExtrinsics(image);
                integration_.extractIntrinsics(image);
            }
            
            return vkc::ReceiverStatus::Open;
        }

    private:
        EnhancedPlannerIntegration& integration_;
    };

    class OdometryReceiver : public vkc::Receiver<vkc::Odometry3d> {
    public:
        explicit OdometryReceiver(EnhancedPlannerIntegration& integration) : integration_(integration) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Odometry3d>>& message) override {
            auto odom_data = message.payload.reader();
            integration_.publishOdometry(odom_data);
            return vkc::ReceiverStatus::Open;
        }

    private:
        EnhancedPlannerIntegration& integration_;
    };

    class DisparityReceiver : public vkc::Receiver<vkc::Disparity> {
    public:
        explicit DisparityReceiver(EnhancedPlannerIntegration& integration) : integration_(integration) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Disparity>>& message) override {
            auto disparity_data = message.payload.reader();
            integration_.publishDepthForPlanner(disparity_data);
            return vkc::ReceiverStatus::Open;
        }

    private:
        EnhancedPlannerIntegration& integration_;
    };

    void extractExtrinsics(const vkc::Image::Reader& image) {
        if (stereo1_l_extrinsics_.is_valid) {
            return;  // Already extracted
        }
        
        auto extrinsic = image.getExtrinsic();
        auto body_frame = extrinsic.getBodyFrame();
        
        // Extract position
        auto position = body_frame.getPosition();
        stereo1_l_extrinsics_.body_position = Eigen::Vector3d(
            position.getX(), position.getY(), position.getZ()
        );
        
        // Extract rotation  
        auto orientation = body_frame.getOrientation();
        stereo1_l_extrinsics_.body_rotation = Eigen::Quaterniond(
            orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ()
        );
        
        stereo1_l_extrinsics_.updateTransform();
        stereo1_l_extrinsics_.is_valid = true;
        
        ROS_INFO("✅ Extracted stereo1_l extrinsics for depth transformation");
        ROS_INFO("Body position: [%.3f, %.3f, %.3f]", 
                 stereo1_l_extrinsics_.body_position.x(),
                 stereo1_l_extrinsics_.body_position.y(),
                 stereo1_l_extrinsics_.body_position.z());
    }

    void extractIntrinsics(const vkc::Image::Reader& image) {
        if (stereo1_l_intrinsics_.is_valid) {
            return;  // Already extracted
        }
        
        // Camera intrinsics will be extracted from disparity data
        // Mark as placeholder for now - actual extraction in disparity handler
        // stereo1_l_intrinsics_.is_valid = true;
        
        ROS_INFO("Image stream available, waiting for disparity data for intrinsics");
    }

    void publishOdometry(const vkc::Odometry3d::Reader& odom_data) {
        nav_msgs::Odometry odom_msg;
        
        // Header
        auto header = odom_data.getHeader();
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = base_frame_id_;

        // Pose
        auto pose = odom_data.getPose();
        auto position = pose.getPosition();
        auto orientation = pose.getOrientation();

        odom_msg.pose.pose.position.x = position.getX();
        odom_msg.pose.pose.position.y = position.getY();
        odom_msg.pose.pose.position.z = position.getZ();

        odom_msg.pose.pose.orientation.w = orientation.getW();
        odom_msg.pose.pose.orientation.x = orientation.getX();
        odom_msg.pose.pose.orientation.y = orientation.getY();
        odom_msg.pose.pose.orientation.z = orientation.getZ();

        // Velocity - VK180 odometry doesn't have separate velocity, set to zero
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;

        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        // Publish to planner
        planner_odom_pub_.publish(odom_msg);
    }

    void publishDepthForPlanner(const vkc::Disparity::Reader& disparity_data) {
        // Extract intrinsics from disparity data if not done yet
        if (!stereo1_l_intrinsics_.is_valid) {
            stereo1_l_intrinsics_.fx = disparity_data.getFx();
            stereo1_l_intrinsics_.fy = disparity_data.getFy();
            stereo1_l_intrinsics_.cx = disparity_data.getCx();
            stereo1_l_intrinsics_.cy = disparity_data.getCy();
            stereo1_l_intrinsics_.baseline = disparity_data.getBaseline();
            stereo1_l_intrinsics_.is_valid = true;
            
            ROS_INFO("✅ Extracted stereo1_l intrinsics from disparity");
            ROS_INFO("fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f, baseline=%.3f", 
                     stereo1_l_intrinsics_.fx, stereo1_l_intrinsics_.fy,
                     stereo1_l_intrinsics_.cx, stereo1_l_intrinsics_.cy,
                     stereo1_l_intrinsics_.baseline);
        }

        if (!stereo1_l_extrinsics_.is_valid) {
            ROS_WARN_THROTTLE(5.0, "Extrinsics not yet available for depth conversion");
            return;
        }

        // Convert disparity to depth
        cv::Mat depth_image = convertDisparityToDepth(disparity_data);
        
        // Publish raw depth for debugging
        publishDepthImage(depth_image, raw_depth_pub_, "camera_optical_frame");
        
        if (use_body_frame_transform_) {
            // Transform depth points to body frame
            cv::Mat body_frame_depth = transformDepthToBodyFrame(depth_image);
            publishDepthImage(body_frame_depth, transformed_depth_pub_, base_frame_id_);
            
            // Use body frame depth for planner
            publishDepthImage(body_frame_depth, planner_depth_pub_, base_frame_id_);
        } else {
            // Use camera frame depth directly
            publishDepthImage(depth_image, planner_depth_pub_, "camera_optical_frame");
        }
    }

    cv::Mat convertDisparityToDepth(const vkc::Disparity::Reader& disparity_data) {
        int width = disparity_data.getWidth();
        int height = disparity_data.getHeight();
        
        // Get disparity values
        auto disparity_values = disparity_data.getData();
        
        // Create depth image (16UC1 format in millimeters for planner compatibility)
        cv::Mat depth_image(height, width, CV_16UC1);
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;
                float disparity = disparity_values[idx];
                
                if (disparity > 0) {
                    // Convert disparity to depth: depth = (baseline * fx) / disparity
                    double depth_m = (stereo1_l_intrinsics_.baseline * stereo1_l_intrinsics_.fx) / disparity;
                    
                    // Clamp depth to valid range
                    depth_m = std::max(min_depth_, std::min(max_depth_, depth_m));
                    
                    // Convert to millimeters (16UC1 format)
                    uint16_t depth_mm = static_cast<uint16_t>(depth_m * 1000.0);
                    depth_image.at<uint16_t>(y, x) = depth_mm;
                } else {
                    depth_image.at<uint16_t>(y, x) = 0;  // Invalid depth
                }
            }
        }
        
        return depth_image;
    }

    cv::Mat transformDepthToBodyFrame(const cv::Mat& camera_depth) {
        cv::Mat body_depth = camera_depth.clone();
        
        int width = camera_depth.cols;
        int height = camera_depth.rows;
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                uint16_t depth_mm = camera_depth.at<uint16_t>(y, x);
                
                if (depth_mm > 0) {
                    // Convert pixel to 3D point in camera frame
                    double depth_m = depth_mm / 1000.0;
                    Eigen::Vector3d camera_point(
                        (x - stereo1_l_intrinsics_.cx) * depth_m / stereo1_l_intrinsics_.fx,
                        (y - stereo1_l_intrinsics_.cy) * depth_m / stereo1_l_intrinsics_.fy,
                        depth_m
                    );
                    
                    // Transform to body frame
                    Eigen::Vector4d camera_point_h(camera_point.x(), camera_point.y(), camera_point.z(), 1.0);
                    Eigen::Vector4d body_point_h = stereo1_l_extrinsics_.cam_to_body_transform * camera_point_h;
                    Eigen::Vector3d body_point = body_point_h.head<3>();
                    
                    // For occupancy mapping, we typically want the distance from robot center
                    double body_distance = body_point.norm();
                    body_distance = std::max(min_depth_, std::min(max_depth_, body_distance));
                    
                    // Store transformed depth
                    uint16_t transformed_depth_mm = static_cast<uint16_t>(body_distance * 1000.0);
                    body_depth.at<uint16_t>(y, x) = transformed_depth_mm;
                }
            }
        }
        
        return body_depth;
    }

    void publishDepthImage(const cv::Mat& depth_image, ros::Publisher& pub, const std::string& frame_id) {
        sensor_msgs::Image depth_msg;
        depth_msg.header.stamp = ros::Time::now();
        depth_msg.header.frame_id = frame_id;
        depth_msg.height = depth_image.rows;
        depth_msg.width = depth_image.cols;
        depth_msg.encoding = "16UC1";  // Millimeters
        depth_msg.is_bigendian = false;
        depth_msg.step = depth_image.cols * 2;  // 2 bytes per pixel for 16UC1
        
        depth_msg.data.resize(depth_msg.step * depth_msg.height);
        std::memcpy(depth_msg.data.data(), depth_image.data, depth_msg.data.size());
        
        pub.publish(depth_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "enhanced_planner_integration");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    ROS_INFO("🚀 Starting Enhanced Planner Integration with Body Frame Transformation");
    
    EnhancedPlannerIntegration integration(nh, pnh);
    
    ros::spin();
    
    return 0;
}