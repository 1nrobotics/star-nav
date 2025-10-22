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
        disparity_pub_ = nh_.advertise<sensor_msgs::Image>("vk180/disparity", 10);
        
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
    ros::Publisher disparity_pub_;
    
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
        explicit DisparityReceiver(EnhancedPlannerIntegration& integration) : integration_(integration), message_count_(0) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Disparity>>& message) override {
            message_count_++;
            auto disparity_data = message.payload.reader();
            
            // Debug: Print disparity data info every 30 messages (roughly every second at 30Hz)
            if (message_count_ % 30 == 1) {
                ROS_INFO("🔍 DISPARITY DEBUG [Message #%d]:", message_count_);
                ROS_INFO("  Stream: %s", disparity_data.getStreamName().cStr());
                ROS_INFO("  Dimensions: %dx%d", disparity_data.getWidth(), disparity_data.getHeight());
                ROS_INFO("  Step: %d", disparity_data.getStep());
                ROS_INFO("  Data size: %zu bytes", disparity_data.getData().size());
                ROS_INFO("  Encoding: %d", static_cast<int>(disparity_data.getEncoding()));
                ROS_INFO("  Max disparity: %u", disparity_data.getMaxDisparity());
                ROS_INFO("  Baseline: %.4f m", disparity_data.getBaseline());
                ROS_INFO("  Intrinsics: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f", 
                         disparity_data.getFx(), disparity_data.getFy(),
                         disparity_data.getCx(), disparity_data.getCy());
                
                // Check data validity
                auto data = disparity_data.getData();
                if (data.size() > 0) {
                    float min_val = *std::min_element(data.begin(), data.end());
                    float max_val = *std::max_element(data.begin(), data.end());
                    int valid_pixels = std::count_if(data.begin(), data.end(), [](float d) { return d > 0; });
                    ROS_INFO("  Data range: [%.2f - %.2f], Valid pixels: %d/%zu (%.1f%%)", 
                             min_val, max_val, valid_pixels, data.size(), 
                             100.0 * valid_pixels / data.size());
                } else {
                    ROS_WARN("  ⚠️  No disparity data received!");
                }
            }
            
            try {
                integration_.publishDepthForPlanner(disparity_data);
                integration_.publishDisparityImage(disparity_data);
                
                // Success feedback (throttled)
                if (message_count_ % 100 == 1) {
                    ROS_INFO("✅ Successfully processed disparity message #%d", message_count_);
                }
            } catch (const std::exception& e) {
                ROS_ERROR("❌ Error processing disparity data: %s", e.what());
            }
            
            return vkc::ReceiverStatus::Open;
        }

    private:
        EnhancedPlannerIntegration& integration_;
        int message_count_;
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
        static int publish_count = 0;
        publish_count++;
        
        ROS_INFO_THROTTLE(2.0, "🔄 Processing disparity for depth conversion [%d]", publish_count);
        
        // Extract intrinsics from disparity data if not done yet
        if (!stereo1_l_intrinsics_.is_valid) {
            ROS_INFO("📐 Extracting camera intrinsics from disparity data...");
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
            ROS_WARN_THROTTLE(5.0, "⏳ Extrinsics not yet available for depth conversion - waiting for S1/stereo1_l image stream");
            return;
        }

        ROS_INFO_THROTTLE(5.0, "🔧 Converting disparity to depth image...");
        
        // Convert disparity to depth
        cv::Mat depth_image = convertDisparityToDepth(disparity_data);
        
        if (depth_image.empty()) {
            ROS_ERROR("❌ Failed to convert disparity to depth - empty depth image");
            return;
        }
        
        ROS_INFO_THROTTLE(5.0, "📷 Publishing raw depth image (size: %dx%d)", depth_image.cols, depth_image.rows);
        
        // Publish raw depth for debugging
        publishDepthImage(depth_image, raw_depth_pub_, "camera_optical_frame");
        
        if (use_body_frame_transform_) {
            ROS_INFO_THROTTLE(5.0, "🔄 Transforming depth to body frame...");
            // Transform depth points to body frame
            cv::Mat body_frame_depth = transformDepthToBodyFrame(depth_image);
            
            if (body_frame_depth.empty()) {
                ROS_ERROR("❌ Failed to transform depth to body frame");
                return;
            }
            
            ROS_INFO_THROTTLE(5.0, "📡 Publishing body frame depth for planner");
            publishDepthImage(body_frame_depth, transformed_depth_pub_, base_frame_id_);
            
            // Use body frame depth for planner
            publishDepthImage(body_frame_depth, planner_depth_pub_, base_frame_id_);
        } else {
            ROS_INFO_THROTTLE(5.0, "📡 Publishing camera frame depth for planner");
            // Use camera frame depth directly
            publishDepthImage(depth_image, planner_depth_pub_, "camera_optical_frame");
        }
        
        ROS_INFO_THROTTLE(10.0, "✅ Depth processing pipeline completed successfully [%d processed]", publish_count);
    }

    cv::Mat convertDisparityToDepth(const vkc::Disparity::Reader& disparity_data) {
        static int conversion_count = 0;
        conversion_count++;
        
        int width = disparity_data.getWidth();
        int height = disparity_data.getHeight();
        
        ROS_INFO_THROTTLE(3.0, "🔧 Converting disparity to depth [%d] (size: %dx%d)", 
                         conversion_count, width, height);
        
        // Get disparity values
        auto disparity_values = disparity_data.getData();
        
        if (!disparity_values.size()) {
            ROS_ERROR("❌ Empty disparity data received");
            return cv::Mat();
        }
        
        ROS_INFO_THROTTLE(5.0, "📊 Disparity data size: %lu pixels, expected: %d", 
                         disparity_values.size(), width * height);
        
        // Create depth image (16UC1 format in millimeters for planner compatibility)
        cv::Mat depth_image(height, width, CV_16UC1);
        
        int valid_pixels = 0;
        float min_disparity = FLT_MAX, max_disparity = -FLT_MAX;
        float sum_disparity = 0.0f;
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;
                float disparity = disparity_values[idx];
                
                if (disparity > 0) {
                    valid_pixels++;
                    sum_disparity += disparity;
                    min_disparity = std::min(min_disparity, disparity);
                    max_disparity = std::max(max_disparity, disparity);
                    
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
        
        float avg_disparity = (valid_pixels > 0) ? sum_disparity / valid_pixels : 0.0f;
        float depth_validity_ratio = static_cast<float>(valid_pixels) / (width * height);
        
        ROS_INFO_THROTTLE(5.0, "📈 Disparity stats: valid=%d/%d (%.1f%%), range=[%.2f,%.2f], avg=%.2f", 
                         valid_pixels, width * height, depth_validity_ratio * 100.0f,
                         min_disparity, max_disparity, avg_disparity);
        
        if (valid_pixels == 0) {
            ROS_WARN("⚠️  No valid disparity pixels found - depth image will be empty");
        }
        
        ROS_INFO_THROTTLE(8.0, "✅ Depth conversion completed: %dx%d, validity=%.1f%%", 
                         width, height, depth_validity_ratio * 100.0f);
        
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
        static std::map<std::string, int> pub_counts;
        pub_counts[frame_id]++;
        
        if (depth_image.empty()) {
            ROS_ERROR("📷 Cannot publish empty depth image for frame: %s", frame_id.c_str());
            return;
        }

        ROS_INFO_THROTTLE(3.0, "📡 Publishing depth image to %s [count: %d] (size: %dx%d, type: %d)", 
                         frame_id.c_str(), pub_counts[frame_id], 
                         depth_image.cols, depth_image.rows, depth_image.type());
        
        // Check if publisher has subscribers
        if (pub.getNumSubscribers() == 0) {
            ROS_INFO_THROTTLE(10.0, "⚠️  No subscribers for depth topic on frame: %s", frame_id.c_str());
        } else {
            ROS_INFO_THROTTLE(5.0, "👥 %d subscriber(s) for depth topic on frame: %s", 
                             pub.getNumSubscribers(), frame_id.c_str());
        }
        
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
        
        // Analyze depth data for debugging
        cv::Scalar mean_val, std_val;
        cv::meanStdDev(depth_image, mean_val, std_val);
        double min_val, max_val;
        cv::minMaxLoc(depth_image, &min_val, &max_val);
        
        ROS_INFO_THROTTLE(8.0, "📊 Depth stats for %s: mean=%.2f, std=%.2f, min=%.2f, max=%.2f", 
                         frame_id.c_str(), mean_val[0], std_val[0], min_val, max_val);
        
        try {
            pub.publish(depth_msg);
            ROS_INFO_THROTTLE(8.0, "✅ Successfully published depth image to %s", frame_id.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("❌ Exception publishing depth image to %s: %s", frame_id.c_str(), e.what());
        }
    }

    void publishDisparityImage(const vkc::Disparity::Reader& disparity_data) {
        static int disparity_pub_count = 0;
        disparity_pub_count++;
        
        int width = disparity_data.getWidth();
        int height = disparity_data.getHeight();
        
        ROS_INFO_THROTTLE(3.0, "📡 Publishing disparity image [%d] (size: %dx%d)", 
                         disparity_pub_count, width, height);
        
        // Check if publisher has subscribers
        if (disparity_pub_.getNumSubscribers() == 0) {
            ROS_INFO_THROTTLE(10.0, "⚠️  No subscribers for disparity topic");
            return; // Skip processing if no one is listening
        }
        
        ROS_INFO_THROTTLE(5.0, "👥 %d subscriber(s) for disparity topic", 
                         disparity_pub_.getNumSubscribers());
        
        // Get disparity values
        auto disparity_values = disparity_data.getData();
        
        if (!disparity_values.size()) {
            ROS_ERROR("❌ Empty disparity data - cannot publish");
            return;
        }
        
        sensor_msgs::Image disparity_msg;
        disparity_msg.header.stamp = ros::Time::now();
        disparity_msg.header.frame_id = "camera_optical_frame";
        disparity_msg.height = height;
        disparity_msg.width = width;
        disparity_msg.encoding = "32FC1";  // 32-bit float, 1 channel
        disparity_msg.is_bigendian = false;
        disparity_msg.step = width * sizeof(float);
        
        size_t data_size = width * height * sizeof(float);
        disparity_msg.data.resize(data_size);
        
        // Copy disparity data - iterate through the Cap'n Proto data safely
        float* disparity_data_ptr = reinterpret_cast<float*>(disparity_msg.data.data());
        size_t expected_size = width * height;
        size_t actual_size = disparity_values.size();
        
        if (actual_size != expected_size) {
            ROS_WARN("Disparity data size mismatch: expected %lu, got %lu", expected_size, actual_size);
            return;
        }
        
        for (size_t i = 0; i < actual_size; ++i) {
            disparity_data_ptr[i] = disparity_values[i];
        }
        
        // Analyze disparity data for debugging
        float min_disp = FLT_MAX, max_disp = -FLT_MAX;
        float sum_disp = 0.0f;
        int valid_pixels = 0;
        
        for (size_t i = 0; i < disparity_values.size(); ++i) {
            float disp = disparity_values[i];
            if (disp > 0) {
                valid_pixels++;
                sum_disp += disp;
                min_disp = std::min(min_disp, disp);
                max_disp = std::max(max_disp, disp);
            }
        }
        
        float avg_disp = (valid_pixels > 0) ? sum_disp / valid_pixels : 0.0f;
        float validity_ratio = static_cast<float>(valid_pixels) / (width * height);
        
        ROS_INFO_THROTTLE(8.0, "📊 Disparity stats: valid=%d/%d (%.1f%%), range=[%.2f,%.2f], avg=%.2f", 
                         valid_pixels, width * height, validity_ratio * 100.0f,
                         min_disp, max_disp, avg_disp);
        
        try {
            disparity_pub_.publish(disparity_msg);
            ROS_INFO_THROTTLE(8.0, "✅ Successfully published disparity image");
        } catch (const std::exception& e) {
            ROS_ERROR("❌ Exception publishing disparity image: %s", e.what());
        }
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