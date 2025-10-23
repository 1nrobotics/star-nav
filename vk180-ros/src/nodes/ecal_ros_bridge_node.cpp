#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>
#include <iostream>
#include <vk_sdk/Sdk.hpp>
#include <vk_sdk/capnp/image.capnp.h>
#include <cmath>
#include <array>
#include <string>
#include <cstring>

class EcalRosBridge {
public:
    EcalRosBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        // Initialize publishers for VK180 topics
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("vk180/odometry", 10);
        disparity_pub_ = nh_.advertise<sensor_msgs::Image>("vk180/disparity/image", 10);
        disparity_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("vk180/disparity/camera_info", 10);
        // Note: S1/stereo1_l image stream monitored but not published (using printImage for info only)
        
        // Initialize publishers for planner integration
        planner_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("grid_map/odom", 10);
        planner_depth_pub_ = nh_.advertise<sensor_msgs::Image>("grid_map/depth", 10);
        
        // Parameters
        pnh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
        pnh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
        pnh_.param<std::string>("camera_frame_id", camera_frame_id_, "camera_link");
        pnh_.param<bool>("publish_tf", publish_tf_, true);
        pnh_.param<bool>("publish_to_planner", publish_to_planner_, true);
        pnh_.param<std::string>("ecal_odom_topic", ecal_odom_topic_, "S1/vio_odom");
        pnh_.param<std::string>("ecal_disparity_topic", ecal_disparity_topic_, "S1/stereo1_l/disparity");
        pnh_.param<std::string>("ecal_image_topic", ecal_image_topic_, "S1/stereo1_l");
        
        // Initialize eCAL
        initializeEcal();
    }

    ~EcalRosBridge() {
        if (visualkit_) {
            visualkit_->source().stop(false);
        }
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher odom_pub_;
    ros::Publisher disparity_pub_;
    ros::Publisher disparity_info_pub_;
    
    // Publishers for planner integration
    ros::Publisher planner_odom_pub_;
    ros::Publisher planner_depth_pub_;
    
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string camera_frame_id_;
    std::string ecal_odom_topic_;
    std::string ecal_disparity_topic_;
    std::string ecal_image_topic_;
    bool publish_tf_;
    bool publish_to_planner_;
    
    std::unique_ptr<vkc::VisualKit> visualkit_;

    void initializeEcal() {
        visualkit_ = vkc::VisualKit::create(std::nullopt);
        
        if (visualkit_ == nullptr) {
            ROS_ERROR("Failed to create visualkit connection.");
            return;
        }

        // Create and install odometry receiver
        auto odom_receiver = std::make_unique<OdometryReceiver>(*this);
        visualkit_->source().install(ecal_odom_topic_, std::move(odom_receiver));

        // Create and install disparity receiver
        auto disparity_receiver = std::make_unique<DisparityReceiver>(*this);
        visualkit_->source().install(ecal_disparity_topic_, std::move(disparity_receiver));

        // Create and install image receiver
        auto image_receiver = std::make_unique<ImageReceiver>(*this);
        visualkit_->source().install(ecal_image_topic_, std::move(image_receiver));

        // Start the data source
        visualkit_->source().start();
        
        ROS_INFO("eCAL ROS Bridge initialized");
        ROS_INFO("Odometry topic: %s -> /vk180/odometry", ecal_odom_topic_.c_str());
        ROS_INFO("Disparity topic: %s -> /vk180/disparity/image", ecal_disparity_topic_.c_str());
        ROS_INFO("Image topic: %s (monitoring only, not publishing)", ecal_image_topic_.c_str());
    }

    class OdometryReceiver : public vkc::Receiver<vkc::Odometry3d> {
    public:
        explicit OdometryReceiver(EcalRosBridge& bridge) : bridge_(bridge) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Odometry3d>>& message) override {
            auto odom_data = message.payload.reader();
            bridge_.publishOdometry(odom_data);
            return vkc::ReceiverStatus::Open;
        }

    private:
        EcalRosBridge& bridge_;
    };

    class DisparityReceiver : public vkc::Receiver<vkc::Disparity> {
    public:
        explicit DisparityReceiver(EcalRosBridge& bridge) : bridge_(bridge) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Disparity>>& message) override {
            auto disparity_data = message.payload.reader();
            bridge_.publishDisparity(disparity_data);
            return vkc::ReceiverStatus::Open;
        }

    private:
        EcalRosBridge& bridge_;
    };

    class ImageReceiver : public vkc::Receiver<vkc::Image> {
    public:
        explicit ImageReceiver(EcalRosBridge& bridge) : bridge_(bridge) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Image>>& message) override {
            auto image_data = message.payload.reader();
            bridge_.printImage(image_data);
            return vkc::ReceiverStatus::Open;
        }

    private:
        EcalRosBridge& bridge_;
    };

    void publishOdometry(const vkc::Odometry3d::Reader& odom_data) {
        nav_msgs::Odometry odom_msg;
        
        // Header
        auto header = odom_data.getHeader();
        odom_msg.header.seq = header.getSeq();
        odom_msg.header.stamp = ros::Time::now(); // Could use header.getStampMonotonic() with conversion
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

        // Pose covariance (6x6 matrix)
        auto pose_cov = odom_data.getPoseCovariance();
        for (size_t i = 0; i < 36 && i < pose_cov.size(); ++i) {
            odom_msg.pose.covariance[i] = pose_cov[i];
        }

        // Twist
        auto twist = odom_data.getTwist();
        auto linear = twist.getLinear();
        auto angular = twist.getAngular();

        odom_msg.twist.twist.linear.x = linear.getX();
        odom_msg.twist.twist.linear.y = linear.getY();
        odom_msg.twist.twist.linear.z = linear.getZ();

        odom_msg.twist.twist.angular.x = angular.getX();
        odom_msg.twist.twist.angular.y = angular.getY();
        odom_msg.twist.twist.angular.z = angular.getZ();

        // Twist covariance (6x6 matrix)
        auto twist_cov = odom_data.getTwistCovariance();
        for (size_t i = 0; i < 36 && i < twist_cov.size(); ++i) {
            odom_msg.twist.covariance[i] = twist_cov[i];
        }

        // Publish odometry
        odom_pub_.publish(odom_msg);
        
        // Publish to planner if enabled
        if (publish_to_planner_) {
            planner_odom_pub_.publish(odom_msg);
        }

        // Publish transform if enabled
        if (publish_tf_) {
            publishTransform(odom_msg);
        }
    }

    void publishTransform(const nav_msgs::Odometry& odom_msg) {
        geometry_msgs::TransformStamped transform;
        
        transform.header = odom_msg.header;
        transform.child_frame_id = odom_msg.child_frame_id;
        
        transform.transform.translation.x = odom_msg.pose.pose.position.x;
        transform.transform.translation.y = odom_msg.pose.pose.position.y;
        transform.transform.translation.z = odom_msg.pose.pose.position.z;
        
        transform.transform.rotation = odom_msg.pose.pose.orientation;
        
        tf_broadcaster_.sendTransform(transform);
    }

    void publishDisparity(const vkc::Disparity::Reader& disparity_data) {
        // Publish disparity image
        sensor_msgs::Image disparity_msg;
        
        auto header = disparity_data.getHeader();
        disparity_msg.header.seq = header.getSeq();
        disparity_msg.header.stamp = ros::Time::now();
        disparity_msg.header.frame_id = camera_frame_id_;

        disparity_msg.width = disparity_data.getWidth();
        disparity_msg.height = disparity_data.getHeight();
        disparity_msg.step = disparity_data.getStep();

        // Set encoding based on eCAL disparity encoding
        auto encoding = disparity_data.getEncoding();
        switch (encoding) {
            case vkc::Disparity::Encoding::DISPARITY8:
                disparity_msg.encoding = "mono8";
                break;
            case vkc::Disparity::Encoding::DISPARITY16:
                disparity_msg.encoding = "mono16";
                break;
            case vkc::Disparity::Encoding::DEPTH16:
                disparity_msg.encoding = "16UC1";
                break;
            case vkc::Disparity::Encoding::DEPTH32:
                disparity_msg.encoding = "32FC1";
                break;
            case vkc::Disparity::Encoding::DISTANCE16:
                disparity_msg.encoding = "16UC1";
                break;
            case vkc::Disparity::Encoding::DISTANCE32:
                disparity_msg.encoding = "32FC1";
                break;
            case vkc::Disparity::Encoding::CONFIDENCE8:
                disparity_msg.encoding = "mono8";
                break;
            default:
                disparity_msg.encoding = "mono8";
        }

        // Copy image data
        auto data = disparity_data.getData();
        disparity_msg.data.resize(data.size());
        std::memcpy(disparity_msg.data.data(), data.begin(), data.size());

        disparity_pub_.publish(disparity_msg);
        
        // Publish to planner if enabled (convert disparity to depth for planner)
        if (publish_to_planner_) {
            publishDepthForPlanner(disparity_data, disparity_msg.header);
        }

        // Publish camera info
        publishCameraInfo(disparity_data, disparity_msg.header);
    }

    void publishCameraInfo(const vkc::Disparity::Reader& disparity_data, const std_msgs::Header& header) {
        sensor_msgs::CameraInfo camera_info;
        
        camera_info.header = header;
        camera_info.width = disparity_data.getWidth();
        camera_info.height = disparity_data.getHeight();
        
        // Camera intrinsics
        double fx = disparity_data.getFx();
        double fy = disparity_data.getFy();
        double cx = disparity_data.getCx();
        double cy = disparity_data.getCy();
        
        // K matrix (3x3)
        camera_info.K[0] = fx;  camera_info.K[1] = 0;   camera_info.K[2] = cx;
        camera_info.K[3] = 0;   camera_info.K[4] = fy;  camera_info.K[5] = cy;
        camera_info.K[6] = 0;   camera_info.K[7] = 0;   camera_info.K[8] = 1;
        
        // P matrix (3x4) - for stereo, baseline info could be used
        double baseline = disparity_data.getBaseline();
        camera_info.P[0] = fx;  camera_info.P[1] = 0;   camera_info.P[2] = cx;   camera_info.P[3] = -fx * baseline;
        camera_info.P[4] = 0;   camera_info.P[5] = fy;  camera_info.P[6] = cy;   camera_info.P[7] = 0;
        camera_info.P[8] = 0;   camera_info.P[9] = 0;   camera_info.P[10] = 1;   camera_info.P[11] = 0;
        
        // Distortion model - assume plumb_bob (pinhole)
        camera_info.distortion_model = "plumb_bob";
        camera_info.D.resize(5, 0.0); // No distortion parameters available from eCAL
        
        disparity_info_pub_.publish(camera_info);
    }
    
    void publishDepthForPlanner(const vkc::Disparity::Reader& disparity_data, const std_msgs::Header& header) {
        // Convert disparity to depth image for planner (RealSense-compatible format)
        sensor_msgs::Image depth_msg;
        
        depth_msg.header = header;
        depth_msg.width = disparity_data.getWidth();
        depth_msg.height = disparity_data.getHeight();
        
        auto encoding = disparity_data.getEncoding();
        auto data = disparity_data.getData();
        
        // For planner compatibility: use 16UC1 format (millimeters) like RealSense
        depth_msg.encoding = "16UC1";  // Same as RealSense
        depth_msg.step = depth_msg.width * 2; // 2 bytes per uint16
        depth_msg.data.resize(depth_msg.height * depth_msg.step);
        
        uint16_t* depth_ptr = reinterpret_cast<uint16_t*>(depth_msg.data.data());
        
        // Convert all formats to millimeters (uint16) for planner compatibility
        if (encoding == vkc::Disparity::Encoding::DEPTH32) {
            // Depth in meters (32-bit float) → millimeters (16-bit uint)
            const float* src_ptr = reinterpret_cast<const float*>(data.begin());
            for (size_t i = 0; i < depth_msg.width * depth_msg.height; ++i) {
                float depth_meters = src_ptr[i];
                if (depth_meters > 0 && depth_meters < 65.535) {  // Valid range for 16-bit mm
                    depth_ptr[i] = static_cast<uint16_t>(depth_meters * 1000.0f); // meters → mm
                } else {
                    depth_ptr[i] = 0; // Invalid depth
                }
            }
        }
        else if (encoding == vkc::Disparity::Encoding::DEPTH16) {
            // Already depth in millimeters (16-bit) - direct copy
            std::memcpy(depth_ptr, data.begin(), data.size());
        }
        else if (encoding == vkc::Disparity::Encoding::DISPARITY16 || 
                 encoding == vkc::Disparity::Encoding::DISPARITY8) {
            // Raw disparity - convert to depth in millimeters
            double fx = disparity_data.getFx();
            double baseline = disparity_data.getBaseline();
            
            if (encoding == vkc::Disparity::Encoding::DISPARITY16) {
                const uint16_t* src_ptr = reinterpret_cast<const uint16_t*>(data.begin());
                for (size_t i = 0; i < depth_msg.width * depth_msg.height; ++i) {
                    if (src_ptr[i] > 0) {
                        float depth_meters = static_cast<float>((fx * baseline) / src_ptr[i]);
                        if (depth_meters < 65.535) {
                            depth_ptr[i] = static_cast<uint16_t>(depth_meters * 1000.0f); // meters → mm
                        } else {
                            depth_ptr[i] = 0; // Too far
                        }
                    } else {
                        depth_ptr[i] = 0; // Invalid depth
                    }
                }
            } else { // DISPARITY8
                const uint8_t* src_ptr = reinterpret_cast<const uint8_t*>(data.begin());
                for (size_t i = 0; i < depth_msg.width * depth_msg.height; ++i) {
                    if (src_ptr[i] > 0) {
                        float depth_meters = static_cast<float>((fx * baseline) / src_ptr[i]);
                        if (depth_meters < 65.535) {
                            depth_ptr[i] = static_cast<uint16_t>(depth_meters * 1000.0f); // meters → mm
                        } else {
                            depth_ptr[i] = 0; // Too far
                        }
                    } else {
                        depth_ptr[i] = 0; // Invalid depth
                    }
                }
            }
        }
        else {
            // Unknown format - assume depth in meters, convert to millimeters
            ROS_WARN_THROTTLE(5.0, "Unknown disparity encoding for planner conversion, assuming depth in meters");
            const float* src_ptr = reinterpret_cast<const float*>(data.begin());
            for (size_t i = 0; i < depth_msg.width * depth_msg.height; ++i) {
                float depth_meters = src_ptr[i];
                if (depth_meters > 0 && depth_meters < 65.535) {
                    depth_ptr[i] = static_cast<uint16_t>(depth_meters * 1000.0f); // meters → mm
                } else {
                    depth_ptr[i] = 0;
                }
            }
        }
        
        planner_depth_pub_.publish(depth_msg);
    }

    void printImage(const vkc::Image::Reader& image_data) {
        static int image_count = 0;
        image_count++;
        
        // Only print every 30 messages to avoid spam
        if (image_count % 30 != 1) {
            return;
        }
        
        auto header = image_data.getHeader();
        int width = image_data.getWidth();
        int height = image_data.getHeight();
        auto data = image_data.getData();
        
        std::string encoding_str = "unknown";
        switch (image_data.getEncoding()) {
            case vkc::Image::Encoding::MONO8:
                encoding_str = "mono8";
                break;
            case vkc::Image::Encoding::MONO16:
                encoding_str = "mono16";
                break;
            case vkc::Image::Encoding::YUV420:
                encoding_str = "yuv420";
                break;
            case vkc::Image::Encoding::BGR8:
                encoding_str = "bgr8";
                break;
            case vkc::Image::Encoding::JPEG:
                encoding_str = "jpeg";
                break;
            case vkc::Image::Encoding::PNG:
                encoding_str = "png";
                break;
            case vkc::Image::Encoding::H264:
                encoding_str = "h264";
                break;
            case vkc::Image::Encoding::H265:
                encoding_str = "h265";
                break;
            default:
                encoding_str = "unknown";
        }
        
        ROS_INFO("Image [#%d] from %s: %dx%d, encoding=%s, data=%zu bytes, seq=%u, stream=%s", 
                 image_count, ecal_image_topic_.c_str(), width, height,
                 encoding_str.c_str(), data.size(), header.getSeq(),
                 image_data.getStreamName().cStr());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ecal_ros_bridge");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    ROS_INFO("Starting eCAL ROS Bridge...");
    
    try {
        EcalRosBridge bridge(nh, pnh);
        
        ROS_INFO("eCAL ROS Bridge started successfully");
        ros::spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in eCAL ROS Bridge: %s", e.what());
        return 1;
    }
    
    return 0;
}