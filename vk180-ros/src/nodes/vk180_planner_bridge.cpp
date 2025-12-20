// VK180 Planner Bridge Node
// 
// This node bridges VK180 sensor data (odometry, disparity, point clouds) to ROS for autonomous navigation.
// Features:
// - Publishes raw VIO odometry and vision pose to PX4 EKF2 for sensor fusion
// - Generates combined point cloud from dual stereo cameras in world and body frames
// - Distance filtering: Removes points closer than 40cm from drone center to avoid near-field artifacts
// - Virtual body-centered camera: Projects body-frame points to depth image using FLU→camera transform
//   * Body frame: X=forward, Y=left, Z=up (FLU convention)
//   * Virtual camera frame: X=right, Y=down, Z=forward (standard camera convention)
//   * Transform: cam_x=body_y, cam_y=-body_z, cam_z=body_x

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <iostream>
#include <vk_sdk/Sdk.hpp>
#include <vk_sdk/capnp/image.capnp.h>
#include <vk_sdk/DisparityToPointCloud.hpp>
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

class VK180PlannerBridge {
public:
    VK180PlannerBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        // Publishers for planner
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("vk180/odom_raw", 10);  // Raw VIO before PX4 fusion
        vision_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);  // Vision pose to PX4 EKF2
        depth_image_rect_raw_pub_ = nh_.advertise<sensor_msgs::Image>("vk180/depth/image_rect_raw", 10);  // For exploration planner
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("vk180/depth/camera_info", 10);  // Camera intrinsics for planner
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("vk180/pointcloud", 10);
        image_left_pub_ = nh_.advertise<sensor_msgs::Image>("vk180/image_left", 10);
        image_right_pub_ = nh_.advertise<sensor_msgs::Image>("vk180/image_right", 10);
        
        // Subscriber for MAVROS fused odometry (for point cloud transformation)
        mavros_odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, 
                                         &VK180PlannerBridge::mavrosOdomCallback, this);
        
        // Parameters
        pnh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
        pnh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
        pnh_.param<bool>("use_body_frame_transform", use_body_frame_transform_, true);
        pnh_.param<bool>("subscribe_to_image_stream", subscribe_to_image_stream_, true);  // Enable for RGB data
        pnh_.param<double>("max_depth", max_depth_, 10.0);
        pnh_.param<double>("min_depth", min_depth_, 0.1);
        
        ROS_INFO("VK180 Planner Bridge initialized with MAVROS vision pose output");
        ROS_INFO("  Publishing raw VIO to: /vk180/odom_raw");
        ROS_INFO("  Publishing vision pose to PX4 EKF2: /mavros/vision_pose/pose");
        ROS_INFO("  Subscribing to MAVROS fused odometry: /mavros/local_position/odom");
        ROS_INFO("  Using MAVROS odometry for point cloud transformation to world frame");
        
        initializePointCloudParams();
        initializeEcal();
    }

    ~VK180PlannerBridge() {
        if (visualkit_) {
            visualkit_->source().stop(false);
        }
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher odom_pub_;
    ros::Publisher vision_pose_pub_;
    ros::Publisher depth_image_rect_raw_pub_;
    ros::Publisher camera_info_pub_;
    ros::Publisher pointcloud_pub_;
    ros::Publisher image_left_pub_;
    ros::Publisher image_right_pub_;
    
    ros::Subscriber mavros_odom_sub_;
    
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    std::string odom_frame_id_;
    std::string base_frame_id_;
    bool use_body_frame_transform_;
    bool subscribe_to_image_stream_;
    double max_depth_, min_depth_;
    
    std::unique_ptr<vkc::VisualKit> visualkit_;
    
    // Camera calibration data (extrinsics for transforming points to body frame)
    CameraExtrinsics stereo1_l_extrinsics_;
    CameraExtrinsics stereo1_r_extrinsics_;
    
    // Virtual camera intrinsics (ideal pinhole model for depth image generation)
    static constexpr double VIRTUAL_CAM_FX = 320.0;
    static constexpr double VIRTUAL_CAM_FY = 320.0;
    static constexpr double VIRTUAL_CAM_CX = 320.0;
    static constexpr double VIRTUAL_CAM_CY = 240.0;
    
    // Point cloud generation parameters
    vkc::PointCloudParams pc_params_;
    
    // Latest image data for point cloud generation (left and right cameras)
    std::mutex image_mutex_;
    cv::Mat latest_image_left_;
    cv::Mat latest_image_right_;
    
    // Latest disparity data for combined point cloud generation
    std::mutex disparity_mutex_;
    vkc::Shared<vkc::Disparity> latest_disparity_left_;
    vkc::Shared<vkc::Disparity> latest_disparity_right_;
    bool has_left_disparity_ = false;
    bool has_right_disparity_ = false;
    
    // Latest odometry for transforming point cloud to world frame
    std::mutex odom_mutex_;
    Eigen::Matrix4d body_to_world_transform_;
    ros::Time latest_odom_time_;
    bool has_odom_ = false;
    
    void initializeEcal() {
        visualkit_ = vkc::VisualKit::create(std::nullopt);
        
        if (visualkit_ == nullptr) {
            ROS_ERROR("Failed to create visualkit connection.");
            return;
        }

        // Install receivers
        auto odom_receiver = std::make_unique<OdometryReceiver>(*this);
        visualkit_->source().install("S1/vio_odom", std::move(odom_receiver));

        // Subscribe to both left and right image streams if enabled
        if (subscribe_to_image_stream_) {
            ROS_INFO("Creating ImageReceivers for stereo cameras...");
            
            auto image_receiver_left = std::make_unique<ImageReceiverLeft>(*this);
            visualkit_->source().install("S1/stereo1_l", std::move(image_receiver_left));
            ROS_INFO("Successfully subscribed to S1/stereo1_l image stream");
            
            auto image_receiver_right = std::make_unique<ImageReceiverRight>(*this);
            visualkit_->source().install("S1/stereo2_r", std::move(image_receiver_right));
            ROS_INFO("Successfully subscribed to S1/stereo2_r image stream");
        } else {
            ROS_INFO("Skipping stereo image stream subscriptions (subscribe_to_image_stream=false)");
        }
        
        // Subscribe to both left and right disparity streams
        auto disparity_receiver_left = std::make_unique<DisparityReceiverLeft>(*this);
        visualkit_->source().install("S1/stereo1_l/disparity", std::move(disparity_receiver_left));
        
        auto disparity_receiver_right = std::make_unique<DisparityReceiverRight>(*this);
        visualkit_->source().install("S1/stereo2_r/disparity", std::move(disparity_receiver_right));

        visualkit_->source().start();
        
        ROS_INFO("VK180 Planner Bridge initialized");
        if (subscribe_to_image_stream_) {
            ROS_INFO("Waiting for stereo camera extrinsics extraction...");
        }
    }

    // Left camera image receiver
    class ImageReceiverLeft : public vkc::Receiver<vkc::Image> {
    public:
        explicit ImageReceiverLeft(VK180PlannerBridge& integration) : integration_(integration), message_count_(0) {
            ROS_INFO("ImageReceiverLeft constructor called");
        }

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Image>>& message) override {
            message_count_++;
            
            try {
                auto image = message.payload.reader();
                
                if (message_count_ == 1) {
                    ROS_INFO("!!! RECEIVED FIRST LEFT IMAGE from stream: %s !!!", image.getStreamName().cStr());
                    ROS_INFO("Left image dimensions: %dx%d", image.getWidth(), image.getHeight());
                }
                
                std::string stream_name = image.getStreamName().cStr();
                ROS_INFO_THROTTLE(5.0, "Processing left image (message #%d, stream: %s)", 
                                 message_count_, stream_name.c_str());
                
                integration_.extractExtrinsicsLeft(image);
                integration_.storeLatestImageLeft(image);
                
            } catch (const std::exception& e) {
                ROS_ERROR("Exception in ImageReceiverLeft::handle: %s", e.what());
            }
            
            return vkc::ReceiverStatus::Open;
        }

    private:
        VK180PlannerBridge& integration_;
        int message_count_;
    };

    // Right camera image receiver
    class ImageReceiverRight : public vkc::Receiver<vkc::Image> {
    public:
        explicit ImageReceiverRight(VK180PlannerBridge& integration) : integration_(integration), message_count_(0) {
            ROS_INFO("ImageReceiverRight constructor called");
        }

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Image>>& message) override {
            message_count_++;
            
            try {
                auto image = message.payload.reader();
                
                if (message_count_ == 1) {
                    ROS_INFO("!!! RECEIVED FIRST RIGHT IMAGE from stream: %s !!!", image.getStreamName().cStr());
                    ROS_INFO("Right image dimensions: %dx%d", image.getWidth(), image.getHeight());
                }
                                
                std::string stream_name = image.getStreamName().cStr();
                ROS_INFO_THROTTLE(5.0, "Processing right image (message #%d, stream: %s)", 
                                 message_count_, stream_name.c_str());
                
                integration_.extractExtrinsicsRight(image);
                integration_.storeLatestImageRight(image);
                
            } catch (const std::exception& e) {
                ROS_ERROR("Exception in ImageReceiverRight::handle: %s", e.what());
            }
            
            return vkc::ReceiverStatus::Open;
        }

    private:
        VK180PlannerBridge& integration_;
        int message_count_;
    };

    class OdometryReceiver : public vkc::Receiver<vkc::Odometry3d> {
    public:
        explicit OdometryReceiver(VK180PlannerBridge& integration) : integration_(integration) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Odometry3d>>& message) override {
            auto odom_data = message.payload.reader();
            integration_.publishOdometry(odom_data);
            return vkc::ReceiverStatus::Open;
        }

    private:
        VK180PlannerBridge& integration_;
    };

    class DisparityReceiverLeft : public vkc::Receiver<vkc::Disparity> {
    public:
        explicit DisparityReceiverLeft(VK180PlannerBridge& integration) : integration_(integration), message_count_(0) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Disparity>>& message) override {
            message_count_++;
            auto disparity_data = message.payload.reader();
            
            try {
                // Store left disparity for combined point cloud generation
                integration_.storeLeftDisparity(message.payload);
                
                // Success feedback (throttled)
                if (message_count_ % 100 == 1) {
                    ROS_INFO("Successfully processed left disparity message #%d", message_count_);
                }
            } catch (const std::exception& e) {
                ROS_ERROR("Error processing left disparity data: %s", e.what());
            }
            
            return vkc::ReceiverStatus::Open;
        }

    private:
        VK180PlannerBridge& integration_;
        int message_count_;
    };

    class DisparityReceiverRight : public vkc::Receiver<vkc::Disparity> {
    public:
        explicit DisparityReceiverRight(VK180PlannerBridge& integration) : integration_(integration), message_count_(0) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Disparity>>& message) override {
            message_count_++;
            auto disparity_data = message.payload.reader();
            
            try {
                // Store right disparity for combined point cloud generation
                integration_.storeRightDisparity(message.payload);
                
                // Success feedback (throttled)
                if (message_count_ % 100 == 1) {
                    ROS_INFO("Successfully processed right disparity message #%d", message_count_);
                }
            } catch (const std::exception& e) {
                ROS_ERROR("Error processing right disparity data: %s", e.what());
            }
            
            return vkc::ReceiverStatus::Open;
        }

    private:
        VK180PlannerBridge& integration_;
        int message_count_;
    };

    void extractExtrinsicsLeft(const vkc::Image::Reader& image) {
        if (stereo1_l_extrinsics_.is_valid) {
            return;  // Already extracted
        }
        
        // Extract extrinsics the same way as ecal_ros_bridge_node
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
        
        ROS_INFO("========== Extracted Stereo1_L Extrinsics ==========");
        ROS_INFO("Body Frame Transform:");
        ROS_INFO("  Position: [%.3f, %.3f, %.3f]", 
                 stereo1_l_extrinsics_.body_position.x(),
                 stereo1_l_extrinsics_.body_position.y(),
                 stereo1_l_extrinsics_.body_position.z());
        ROS_INFO("  Orientation (quaternion): w=%.3f, x=%.3f, y=%.3f, z=%.3f", 
                 stereo1_l_extrinsics_.body_rotation.w(),
                 stereo1_l_extrinsics_.body_rotation.x(),
                 stereo1_l_extrinsics_.body_rotation.y(),
                 stereo1_l_extrinsics_.body_rotation.z());
        
        // Also print IMU frame for reference (not used in current implementation)
        auto imu_frame = extrinsic.getImuFrame();
        auto imu_position = imu_frame.getPosition();
        auto imu_orientation = imu_frame.getOrientation();
        ROS_INFO("IMU Frame Transform (reference only):");
        ROS_INFO("  Position: [%.3f, %.3f, %.3f]", 
                 imu_position.getX(), imu_position.getY(), imu_position.getZ());
        ROS_INFO("  Orientation (quaternion): w=%.3f, x=%.3f, y=%.3f, z=%.3f", 
                 imu_orientation.getW(), imu_orientation.getX(), 
                 imu_orientation.getY(), imu_orientation.getZ());
        ROS_INFO("Last Modified: %lu", extrinsic.getLastModified());
        ROS_INFO("===================================================");
        
        // Publish TF transform from base_link to camera_link
        publishCameraTransform();
    }

    void extractExtrinsicsRight(const vkc::Image::Reader& image) {
        if (stereo1_r_extrinsics_.is_valid) {
            return;  // Already extracted
        }
        
        auto extrinsic = image.getExtrinsic();
        auto body_frame = extrinsic.getBodyFrame();
        
        auto position = body_frame.getPosition();
        stereo1_r_extrinsics_.body_position = Eigen::Vector3d(
            position.getX(), position.getY(), position.getZ()
        );
        
        auto orientation = body_frame.getOrientation();
        stereo1_r_extrinsics_.body_rotation = Eigen::Quaterniond(
            orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ()
        );
        
        stereo1_r_extrinsics_.updateTransform();
        stereo1_r_extrinsics_.is_valid = true;
        
        ROS_INFO("========== Extracted Stereo1_R Extrinsics ==========");
        ROS_INFO("Body Frame Transform:");
        ROS_INFO("  Position: [%.3f, %.3f, %.3f]", 
                 stereo1_r_extrinsics_.body_position.x(),
                 stereo1_r_extrinsics_.body_position.y(),
                 stereo1_r_extrinsics_.body_position.z());
        ROS_INFO("  Orientation (quaternion): w=%.3f, x=%.3f, y=%.3f, z=%.3f", 
                 stereo1_r_extrinsics_.body_rotation.w(),
                 stereo1_r_extrinsics_.body_rotation.x(),
                 stereo1_r_extrinsics_.body_rotation.y(),
                 stereo1_r_extrinsics_.body_rotation.z());
        ROS_INFO("===================================================");
    }

    void storeLatestImageLeft(const vkc::Image::Reader& image) {
        uint32_t image_height = image.getHeight();
        uint32_t image_width = image.getWidth();
        auto image_encoding = image.getEncoding();
        
        cv::Mat image_mat;
        
        // Convert image to RGB format
        switch (image_encoding) {
        case vkc::Image::Encoding::MONO8:
            image_mat = cv::Mat(image_height, image_width, CV_8UC1,
                               const_cast<unsigned char*>(image.getData().asBytes().begin()));
            cv::cvtColor(image_mat, image_mat, cv::COLOR_GRAY2RGB);
            break;
            
        case vkc::Image::Encoding::YUV420:
            image_mat = cv::Mat(image_height * 3 / 2, image_width, CV_8UC1,
                               const_cast<unsigned char*>(image.getData().asBytes().begin()));
            cv::cvtColor(image_mat, image_mat, cv::COLOR_YUV2BGR_IYUV);
            break;
            
        case vkc::Image::Encoding::BGR8:
            image_mat = cv::Mat(image_height, image_width, CV_8UC3,
                               const_cast<unsigned char*>(image.getData().asBytes().begin()));
            image_mat = image_mat.reshape(1, image_height);
            cv::cvtColor(image_mat, image_mat, cv::COLOR_BGR2RGB);
            break;
            
        case vkc::Image::Encoding::JPEG:
            {
                cv::Mat mat_jpeg(1, image.getData().size(), CV_8UC1,
                                const_cast<unsigned char*>(image.getData().asBytes().begin()));
                image_mat = cv::imdecode(mat_jpeg, cv::IMREAD_COLOR);
            }
            break;
            
        default:
            ROS_WARN_THROTTLE(5.0, "Unsupported left image encoding: %d", static_cast<int>(image_encoding));
            return;
        }
        
        if (!image_mat.empty()) {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_left_ = image_mat.clone();
            ROS_INFO_THROTTLE(10.0, "Stored latest LEFT image: %dx%d, encoding=%d", 
                             image_width, image_height, static_cast<int>(image_encoding));
        }
        
        // Publish the left image
        if (!image_mat.empty() && image_left_pub_.getNumSubscribers() > 0) {
            publishImageLeft(image_mat);
        }
    }

    void storeLatestImageRight(const vkc::Image::Reader& image) {
        uint32_t image_height = image.getHeight();
        uint32_t image_width = image.getWidth();
        auto image_encoding = image.getEncoding();
        
        cv::Mat image_mat;
        
        // Convert image to RGB format
        switch (image_encoding) {
        case vkc::Image::Encoding::MONO8:
            image_mat = cv::Mat(image_height, image_width, CV_8UC1,
                               const_cast<unsigned char*>(image.getData().asBytes().begin()));
            cv::cvtColor(image_mat, image_mat, cv::COLOR_GRAY2RGB);
            break;
            
        case vkc::Image::Encoding::YUV420:
            image_mat = cv::Mat(image_height * 3 / 2, image_width, CV_8UC1,
                               const_cast<unsigned char*>(image.getData().asBytes().begin()));
            cv::cvtColor(image_mat, image_mat, cv::COLOR_YUV2BGR_IYUV);
            break;
            
        case vkc::Image::Encoding::BGR8:
            image_mat = cv::Mat(image_height, image_width, CV_8UC3,
                               const_cast<unsigned char*>(image.getData().asBytes().begin()));
            image_mat = image_mat.reshape(1, image_height);
            cv::cvtColor(image_mat, image_mat, cv::COLOR_BGR2RGB);
            break;
            
        case vkc::Image::Encoding::JPEG:
            {
                cv::Mat mat_jpeg(1, image.getData().size(), CV_8UC1,
                                const_cast<unsigned char*>(image.getData().asBytes().begin()));
                image_mat = cv::imdecode(mat_jpeg, cv::IMREAD_COLOR);
            }
            break;
            
        default:
            ROS_WARN_THROTTLE(5.0, "Unsupported right image encoding: %d", static_cast<int>(image_encoding));
            return;
        }
        
        if (!image_mat.empty()) {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_right_ = image_mat.clone();
            ROS_INFO_THROTTLE(10.0, "Stored latest RIGHT image: %dx%d, encoding=%d", 
                             image_width, image_height, static_cast<int>(image_encoding));
        }
        
        // Publish the right image
        if (!image_mat.empty() && image_right_pub_.getNumSubscribers() > 0) {
            publishImageRight(image_mat);
        }
    }

    void initializePointCloudParams() {
        // Initialize point cloud conversion parameters (similar to PointCloudGenerator.cpp)
        pc_params_.disparityOffset = 0;  // Can be made configurable via ROS params
        pc_params_.maxDepth = max_depth_;
        ROS_INFO("Initialized point cloud parameters: disparityOffset=%d, maxDepth=%.2f", 
                 pc_params_.disparityOffset, pc_params_.maxDepth);
    }

    void storeLeftDisparity(const vkc::Shared<vkc::Disparity>& disparity) {
        {
            std::lock_guard<std::mutex> lock(disparity_mutex_);
            latest_disparity_left_ = disparity;
            has_left_disparity_ = true;
        }
        
        // Try to generate combined point cloud when both disparities available
        generateCombinedPointCloud();
    }

    void storeRightDisparity(const vkc::Shared<vkc::Disparity>& disparity) {
        {
            std::lock_guard<std::mutex> lock(disparity_mutex_);
            latest_disparity_right_ = disparity;
            has_right_disparity_ = true;
        }
        
        // Try to generate combined point cloud when both disparities available
        // generateCombinedPointCloud();
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

        // Publish raw VIO odometry
        odom_pub_.publish(odom_msg);
        
        // Publish vision pose to MAVROS for PX4 EKF2 fusion
        geometry_msgs::PoseStamped vision_pose;
        vision_pose.header.stamp = odom_msg.header.stamp;
        vision_pose.header.frame_id = odom_frame_id_;
        vision_pose.pose = odom_msg.pose.pose;
        vision_pose_pub_.publish(vision_pose);
        
        // Note: We no longer store body_to_world transform here
        // Instead, we use the MAVROS fused odometry from mavrosOdomCallback()
        
        // Publish TF: odom -> base_link
        try {
            geometry_msgs::TransformStamped odom_transform;
            odom_transform.header.stamp = odom_msg.header.stamp;
            odom_transform.header.frame_id = odom_frame_id_;
            odom_transform.child_frame_id = base_frame_id_;
            
            odom_transform.transform.translation.x = odom_msg.pose.pose.position.x;
            odom_transform.transform.translation.y = odom_msg.pose.pose.position.y;
            odom_transform.transform.translation.z = odom_msg.pose.pose.position.z;
            
            odom_transform.transform.rotation = odom_msg.pose.pose.orientation;
            
            tf_broadcaster_.sendTransform(odom_transform);
            
            // Also publish camera transform if available (needs to be republished since base_link is moving)
            if (stereo1_l_extrinsics_.is_valid) {
                publishCameraTransform();
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Exception publishing odometry TF: %s", e.what());
        }
    }

    void mavrosOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Store MAVROS fused odometry for point cloud transformation
        std::lock_guard<std::mutex> lock(odom_mutex_);
        
        Eigen::Quaterniond body_q(msg->pose.pose.orientation.w,
                                 msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y,
                                 msg->pose.pose.orientation.z);
        Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
        
        body_to_world_transform_ = Eigen::Matrix4d::Identity();
        body_to_world_transform_.block<3,3>(0,0) = body_r_m;
        body_to_world_transform_(0,3) = msg->pose.pose.position.x;
        body_to_world_transform_(1,3) = msg->pose.pose.position.y;
        body_to_world_transform_(2,3) = msg->pose.pose.position.z;
        
        latest_odom_time_ = msg->header.stamp;
        has_odom_ = true;
        
        ROS_INFO_ONCE("Receiving MAVROS fused odometry for point cloud transformation");
    }

    void generateCombinedPointCloud() {
        static int pointcloud_count = 0;
        pointcloud_count++;
        
        // Check if any publisher has subscribers
        bool has_pointcloud_subscribers = pointcloud_pub_.getNumSubscribers() > 0;
        bool has_depth_subscribers = depth_image_rect_raw_pub_.getNumSubscribers() > 0;
        
        if (!has_pointcloud_subscribers && !has_depth_subscribers) {
            ROS_INFO_THROTTLE(10.0, "No subscribers for point cloud or depth image topics");
            return;
        }
        
        ROS_INFO_THROTTLE(8.0, "Processing data (pointcloud subs: %d, depth subs: %d)", 
                         pointcloud_pub_.getNumSubscribers(), depth_image_rect_raw_pub_.getNumSubscribers());
        
        // Need current MAVROS fused odometry pose to transform points to world frame
        if (!has_odom_) {
            ROS_WARN_THROTTLE(10.0, "No MAVROS fused odometry available yet, skipping point cloud generation");
            return;
        }
        
        // Check if we have both disparities
        vkc::Shared<vkc::Disparity> left_disparity, right_disparity;
        bool has_left, has_right;
        {
            std::lock_guard<std::mutex> lock(disparity_mutex_);
            has_left = has_left_disparity_;
            has_right = has_right_disparity_;
            if (has_left) left_disparity = latest_disparity_left_;
            if (has_right) right_disparity = latest_disparity_right_;
        }
        
        if (!has_left && !has_right) {
            ROS_WARN_THROTTLE(10.0, "No disparity data available yet, skipping point cloud generation");
            return;
        }
        
        ROS_INFO_THROTTLE(5.0, "Generating combined point cloud [%d] (left=%s, right=%s)", 
                         pointcloud_count, has_left ? "yes" : "no", has_right ? "yes" : "no");
        
        // Get image data for both cameras
        cv::Mat left_image_copy, right_image_copy;
        unsigned char* left_image_data = nullptr;
        unsigned char* right_image_data = nullptr;
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            if (!latest_image_left_.empty()) {
                left_image_copy = latest_image_left_.clone();
                left_image_data = left_image_copy.data;
            }
            if (!latest_image_right_.empty()) {
                right_image_copy = latest_image_right_.clone();
                right_image_data = right_image_copy.data;
            }
        }
        
        if (has_left && !left_image_data) {
            ROS_WARN_THROTTLE(10.0, "No left image data available, skipping left point cloud");
            has_left = false;
        }
        if (has_right && !right_image_data) {
            ROS_WARN_THROTTLE(10.0, "No right image data available, skipping right point cloud");
            has_right = false;
        }
        
        // Create combined PCL point cloud (world frame)
        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Keep body-frame points for virtual camera depth image generation
        pcl::PointCloud<pcl::PointXYZ>::Ptr body_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Get body_to_world transform
        Eigen::Matrix4d body_to_world;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            body_to_world = body_to_world_transform_;
        }

        int total_valid_points = 0;
        
        // Process left camera point cloud
        if (has_left) {
            vkc::Shared<vkc::PointCloud> left_vk_pc = vkc::convertToPointCloud(left_disparity, left_image_data, pc_params_);
            int left_valid_points = processPointCloud(left_vk_pc, stereo1_l_extrinsics_, body_to_world, 
                                                     combined_cloud, body_cloud, "LEFT");
            total_valid_points += left_valid_points;
        }
        
        // Process right camera point cloud
        if (has_right) {
            vkc::Shared<vkc::PointCloud> right_vk_pc = vkc::convertToPointCloud(right_disparity, right_image_data, pc_params_);
            int right_valid_points = processPointCloud(right_vk_pc, stereo1_r_extrinsics_, body_to_world, 
                                                      combined_cloud, body_cloud, "RIGHT");
            total_valid_points += right_valid_points;
        }
        
        combined_cloud->width = total_valid_points;
        combined_cloud->height = 1;
        combined_cloud->is_dense = false;
        
        ROS_INFO_THROTTLE(8.0, "Combined point cloud stats: %d total valid points", total_valid_points);
        
        // Convert to ROS message and publish
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*combined_cloud, cloud_msg);
        
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "world";  // Points transformed to world frame
        
        try {
            pointcloud_pub_.publish(cloud_msg);
            ROS_INFO_THROTTLE(8.0, "Successfully published combined point cloud with %d points in frame: %s", 
                             total_valid_points, cloud_msg.header.frame_id.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Exception publishing combined point cloud: %s", e.what());
        }
        
        // Convert body-frame point cloud to depth image using virtual body-centered camera
        if (body_cloud->points.size() > 0) {
            convertBodyPointCloudToDepthImage(body_cloud);
        } else {
            ROS_WARN_THROTTLE(10.0, "No body points available, skipping depth image generation");
        }
    }

    int processPointCloud(const vkc::Shared<vkc::PointCloud>& vk_pointcloud, 
                          const CameraExtrinsics& extrinsics,
                          const Eigen::Matrix4d& body_to_world,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& output_world_cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& output_body_cloud,
                          const std::string& camera_name) {
        auto pc_reader = vk_pointcloud.reader();
        auto points_data = pc_reader.getPoints().asBytes();
        int point_stride = static_cast<int>(pc_reader.getPointStride());
        int points_count = points_data.size() / point_stride;
        
        ROS_INFO_THROTTLE(8.0, "%s camera VK SDK point cloud: %d points, stride=%d bytes", 
                         camera_name.c_str(), points_count, point_stride);
        
        // Extract field offsets for X, Y, Z
        std::vector<int> field_offsets;
        for (const auto& field : pc_reader.getFields()) {
            field_offsets.push_back(static_cast<int>(field.getOffset()));
        }
        
        if (field_offsets.size() < 3) {
            ROS_ERROR("%s camera: VK point cloud has insufficient fields (need at least X,Y,Z)", camera_name.c_str());
            return 0;
        }
        
        // Minimum distance filter (body frame)
        const double min_distance = 0.4;  // 40cm - filter out points too close to drone center
        
        // FOV filter parameters (in radians)
        const double fov_horizontal_left = 40.0 * M_PI / 180.0;   // 40 degrees left
        const double fov_horizontal_right = -50.0 * M_PI / 180.0; // 50 degrees right
        const double fov_vertical_up = 10.0 * M_PI / 180.0;       // +10 degrees up
        const double fov_vertical_down = -60.0 * M_PI / 180.0;    // -60 degrees down
        
        unsigned char* pc_data = const_cast<unsigned char*>(points_data.begin());
        int valid_points = 0;
        int distance_filtered_points = 0;
        int fov_filtered_points = 0;
        
        for (int i = 0; i < points_count; ++i) {
            unsigned char* point_ptr = pc_data + i * point_stride;
            
            float x = *reinterpret_cast<float*>(point_ptr + field_offsets[0]);
            float y = *reinterpret_cast<float*>(point_ptr + field_offsets[1]);
            float z = *reinterpret_cast<float*>(point_ptr + field_offsets[2]);
            
            // Skip invalid points (NaN or zero depth)
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 0) {
                continue;
            }
            
            // Transform point from camera frame to body frame for distance filtering
            Eigen::Vector4d camera_point(x, y, z, 1.0);
            Eigen::Vector3d body_point_3d;
            
            if (extrinsics.is_valid) {
                // Transform to body frame
                Eigen::Vector4d body_point = extrinsics.cam_to_body_transform * camera_point;
                body_point_3d = body_point.head<3>();
            } else {
                // Fallback: assume camera_link == base_link
                ROS_WARN_THROTTLE(5.0, "%s camera: Extrinsics invalid, assuming camera frame == body frame", camera_name.c_str());
                body_point_3d = camera_point.head<3>();
            }
            
            // Calculate distance from drone center
            double distance_from_center = body_point_3d.norm();
            
            // Filter out points too close to the drone (within 40cm)
            if (distance_from_center < min_distance) {
                distance_filtered_points++;
                continue;
            }
            
            // FOV filter: check horizontal and vertical angles in body frame
            // Horizontal angle: atan2(y, x) - left/right (positive = left, negative = right)
            // Vertical angle: atan2(z, sqrt(x^2 + y^2)) - up/down
            double horizontal_angle = std::atan2(body_point_3d.y(), body_point_3d.x());
            double horizontal_distance = std::sqrt(body_point_3d.x() * body_point_3d.x() + 
                                                   body_point_3d.y() * body_point_3d.y());
            double vertical_angle = std::atan2(body_point_3d.z(), horizontal_distance);
            
            // Filter points outside FOV (asymmetric: 40° left, 50° right, 10° up, 60° down)
            if (horizontal_angle > fov_horizontal_left || 
                horizontal_angle < fov_horizontal_right ||
                vertical_angle > fov_vertical_up || 
                vertical_angle < fov_vertical_down) {
                fov_filtered_points++;
                continue;
            }
            
            // Point passes distance and FOV filters
            // Add to world frame point cloud
            Eigen::Vector4d world_point;
            if (extrinsics.is_valid) {
                Eigen::Vector4d body_point_4d(body_point_3d.x(), body_point_3d.y(), body_point_3d.z(), 1.0);
                world_point = body_to_world * body_point_4d;
            } else {
                world_point = body_to_world * camera_point;
            }
            
            pcl::PointXYZ world_pt;
            world_pt.x = world_point.x();
            world_pt.y = world_point.y();
            world_pt.z = world_point.z();
            output_world_cloud->points.push_back(world_pt);
            
            // Add to body frame point cloud for virtual camera depth image
            pcl::PointXYZ body_pt;
            body_pt.x = body_point_3d.x();
            body_pt.y = body_point_3d.y();
            body_pt.z = body_point_3d.z();
            output_body_cloud->points.push_back(body_pt);
            
            valid_points++;
        }
        
        float validity_ratio = static_cast<float>(valid_points) / points_count;
        float distance_filter_ratio = static_cast<float>(distance_filtered_points) / points_count;
        float fov_filter_ratio = static_cast<float>(fov_filtered_points) / points_count;
        ROS_INFO_THROTTLE(8.0, "%s camera: %d/%d valid points (%.1f%%), %d filtered by distance <0.4m (%.1f%%), %d filtered by FOV (%.1f%%)", 
                         camera_name.c_str(), valid_points, points_count, validity_ratio * 100.0f,
                         distance_filtered_points, distance_filter_ratio * 100.0f,
                         fov_filtered_points, fov_filter_ratio * 100.0f);
        
        return valid_points;
    }

    void convertBodyPointCloudToDepthImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& body_cloud) {
        // Create depth image (640x480, 16UC1 in millimeters)
        const int width = 640;
        const int height = 480;
        cv::Mat depth_image = cv::Mat::zeros(height, width, CV_16UC1);
        
        // Virtual camera intrinsics (ideal pinhole model)
        const double fx = VIRTUAL_CAM_FX;
        const double fy = VIRTUAL_CAM_FY;
        const double cx = VIRTUAL_CAM_CX;
        const double cy = VIRTUAL_CAM_CY;
        
        int projected_points = 0;
        int behind_camera = 0;
        int out_of_bounds = 0;
        
        // Virtual camera at body center with FLU (Forward-Left-Up) frame:
        // Body frame: X=forward, Y=left, Z=up
        // Camera frame convention: X=right, Y=down, Z=forward
        // Transform: cam_x = body_y, cam_y = -body_z, cam_z = body_x
        
        // Project each body-frame point through virtual camera
        for (const auto& pt : body_cloud->points) {
            // Body frame coordinates
            double x_body = pt.x;  // forward
            double y_body = pt.y;  // left
            double z_body = pt.z;  // up
            
            // Transform to virtual camera frame (FLU → camera convention)
            double x_cam = y_body;   // body left → camera right
            double y_cam = -z_body;  // body up → camera down (negated)
            double z_cam = x_body;   // body forward → camera forward (depth)
            
            // Skip points behind virtual camera or too close
            if (z_cam <= 0.2) {
                behind_camera++;
                continue;
            }
            
            // Project to image plane using pinhole model
            int u = static_cast<int>(std::round(fx * x_cam / z_cam + cx));
            int v = static_cast<int>(std::round(fy * y_cam / z_cam + cy));
            
            // Check if projection is within image bounds
            if (u < 0 || u >= width || v < 0 || v >= height) {
                out_of_bounds++;
                continue;
            }
            
            // Convert depth to millimeters
            uint16_t depth_mm = static_cast<uint16_t>(std::min(65535.0, std::max(0.0, z_cam * 1000.0)));
            
            // Keep closest depth (Z-buffer)
            uint16_t& existing_depth = depth_image.at<uint16_t>(v, u);
            if (existing_depth == 0 || depth_mm < existing_depth) {
                existing_depth = depth_mm;
            }
            
            projected_points++;
        }
        
        ROS_INFO_THROTTLE(5.0, "Virtual camera depth: %zu body points, %d projected, %d behind cam, %d out of bounds",
                         body_cloud->points.size(), projected_points, behind_camera, out_of_bounds);
        
        if (projected_points == 0) {
            ROS_WARN_THROTTLE(2.0, "No points projected to depth image! Camera may not see any valid depth.");
        }
        
        // Get synchronized timestamp
        ros::Time stamp;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            stamp = latest_odom_time_;
        }
        
        // Publish depth image (use camera_link to match TF and camera pose)
        publishDepthImage(depth_image, depth_image_rect_raw_pub_, "camera_link", stamp);
        
        // Publish camera_info for planner
        publishCameraInfo(stamp);
    }

    void publishDepthImage(const cv::Mat& depth_image, ros::Publisher& pub, const std::string& frame_id, const ros::Time& stamp = ros::Time::now()) {
        static std::map<std::string, int> pub_counts;
        pub_counts[frame_id]++;
        
        if (depth_image.empty()) {
            ROS_ERROR("Cannot publish empty depth image for frame: %s", frame_id.c_str());
            return;
        }

        ROS_INFO_THROTTLE(3.0, "Publishing depth image to %s [count: %d] (size: %dx%d, type: %d)", 
                         frame_id.c_str(), pub_counts[frame_id], 
                         depth_image.cols, depth_image.rows, depth_image.type());
        
        // Check if publisher has subscribers
        if (pub.getNumSubscribers() == 0) {
            ROS_INFO_THROTTLE(10.0, "No subscribers for depth topic on frame: %s", frame_id.c_str());
        } else {
            ROS_INFO_THROTTLE(5.0, "%d subscriber(s) for depth topic on frame: %s", 
                             pub.getNumSubscribers(), frame_id.c_str());
        }
        
        sensor_msgs::Image depth_msg;
        depth_msg.header.stamp = stamp;
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
        
        ROS_INFO_THROTTLE(8.0, "Depth stats for %s: mean=%.2f, std=%.2f, min=%.2f, max=%.2f", 
                         frame_id.c_str(), mean_val[0], std_val[0], min_val, max_val);
        
        try {
            pub.publish(depth_msg);
            ROS_INFO_THROTTLE(8.0, "Successfully published depth image to %s", frame_id.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Exception publishing depth image to %s: %s", frame_id.c_str(), e.what());
        }
    }

    void publishImageLeft(const cv::Mat& image) {
        static int image_pub_count = 0;
        image_pub_count++;
        
        if (image.empty()) {
            ROS_ERROR("Cannot publish empty left image");
            return;
        }
        
        ROS_INFO_THROTTLE(5.0, "Publishing LEFT RGB image [%d] (size: %dx%d)", 
                         image_pub_count, image.cols, image.rows);
        
        sensor_msgs::Image image_msg;
        image_msg.header.stamp = ros::Time::now();
        image_msg.header.frame_id = "camera_left_optical_frame";
        image_msg.height = image.rows;
        image_msg.width = image.cols;
        image_msg.encoding = "rgb8";
        image_msg.is_bigendian = false;
        image_msg.step = image.cols * 3;  // 3 bytes per pixel for RGB
        
        image_msg.data.resize(image_msg.step * image_msg.height);
        std::memcpy(image_msg.data.data(), image.data, image_msg.data.size());
        
        try {
            image_left_pub_.publish(image_msg);
            ROS_INFO_THROTTLE(10.0, "Successfully published LEFT RGB image");
        } catch (const std::exception& e) {
            ROS_ERROR("Exception publishing LEFT RGB image: %s", e.what());
        }
    }
    
    void publishImageRight(const cv::Mat& image) {
        static int image_pub_count = 0;
        image_pub_count++;
        
        if (image.empty()) {
            ROS_ERROR("Cannot publish empty right image");
            return;
        }
        
        ROS_INFO_THROTTLE(5.0, "Publishing RIGHT RGB image [%d] (size: %dx%d)", 
                         image_pub_count, image.cols, image.rows);
        
        sensor_msgs::Image image_msg;
        image_msg.header.stamp = ros::Time::now();
        image_msg.header.frame_id = "camera_right_optical_frame";
        image_msg.height = image.rows;
        image_msg.width = image.cols;
        image_msg.encoding = "rgb8";
        image_msg.is_bigendian = false;
        image_msg.step = image.cols * 3;  // 3 bytes per pixel for RGB
        
        image_msg.data.resize(image_msg.step * image_msg.height);
        std::memcpy(image_msg.data.data(), image.data, image_msg.data.size());
        
        try {
            image_right_pub_.publish(image_msg);
            ROS_INFO_THROTTLE(10.0, "Successfully published RIGHT RGB image");
        } catch (const std::exception& e) {
            ROS_ERROR("Exception publishing RIGHT RGB image: %s", e.what());
        }
    }
    
    void publishCameraInfo(const ros::Time& stamp) {
        sensor_msgs::CameraInfo camera_info;
        camera_info.header.stamp = stamp;
        camera_info.header.frame_id = "camera_link";
        
        camera_info.width = 640;
        camera_info.height = 480;
        camera_info.distortion_model = "plumb_bob";
        
        // Ideal camera model (no distortion)
        camera_info.D.resize(5, 0.0);
        
        // Camera intrinsic matrix K = [fx  0 cx]
        //                              [ 0 fy cy]
        //                              [ 0  0  1]
        camera_info.K[0] = VIRTUAL_CAM_FX;  // fx
        camera_info.K[2] = VIRTUAL_CAM_CX;  // cx
        camera_info.K[4] = VIRTUAL_CAM_FY;  // fy
        camera_info.K[5] = VIRTUAL_CAM_CY;  // cy
        camera_info.K[8] = 1.0;
        
        // Rectification matrix (identity for ideal camera)
        camera_info.R[0] = 1.0;
        camera_info.R[4] = 1.0;
        camera_info.R[8] = 1.0;
        
        // Projection matrix P = [fx  0 cx  0]
        //                        [ 0 fy cy  0]
        //                        [ 0  0  1  0]
        camera_info.P[0] = VIRTUAL_CAM_FX;
        camera_info.P[2] = VIRTUAL_CAM_CX;
        camera_info.P[5] = VIRTUAL_CAM_FY;
        camera_info.P[6] = VIRTUAL_CAM_CY;
        camera_info.P[10] = 1.0;
        
        camera_info_pub_.publish(camera_info);
        ROS_INFO_THROTTLE(8.0, "Published camera_info with fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
                         VIRTUAL_CAM_FX, VIRTUAL_CAM_FY, VIRTUAL_CAM_CX, VIRTUAL_CAM_CY);
    }
    
    void publishCameraTransform() {
        if (!stereo1_l_extrinsics_.is_valid) {
            return;
        }
        
        try {
            geometry_msgs::TransformStamped transform_stamped;
            
            transform_stamped.header.stamp = ros::Time::now();
            transform_stamped.header.frame_id = base_frame_id_;
            transform_stamped.child_frame_id = "camera_link";
            
            // Set translation from extrinsics
            transform_stamped.transform.translation.x = stereo1_l_extrinsics_.body_position.x();
            transform_stamped.transform.translation.y = stereo1_l_extrinsics_.body_position.y();
            transform_stamped.transform.translation.z = stereo1_l_extrinsics_.body_position.z();
            
            // Set rotation from extrinsics
            transform_stamped.transform.rotation.w = stereo1_l_extrinsics_.body_rotation.w();
            transform_stamped.transform.rotation.x = stereo1_l_extrinsics_.body_rotation.x();
            transform_stamped.transform.rotation.y = stereo1_l_extrinsics_.body_rotation.y();
            transform_stamped.transform.rotation.z = stereo1_l_extrinsics_.body_rotation.z();
            
            tf_broadcaster_.sendTransform(transform_stamped);
            
            ROS_INFO_ONCE("Publishing TF: %s -> camera_link from VK180 extrinsics", base_frame_id_.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Exception publishing camera TF: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vk180_planner_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    // Set logging level to reduce console output
    // Options: Debug, Info, Warn, Error, Fatal
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    ROS_INFO("Starting VK180 Planner Bridge with Body Frame Transformation");
    
    VK180PlannerBridge integration(nh, pnh);
    
    ros::spin();
    
    return 0;
}