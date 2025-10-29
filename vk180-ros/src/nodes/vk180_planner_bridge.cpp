#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
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

struct CameraIntrinsics {
    double fx, fy, cx, cy;
    double baseline;  // For stereo disparity to depth conversion
    bool is_valid = false;
};

class VK180PlannerBridge {
public:
    VK180PlannerBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        // Publishers for planner
        planner_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("grid_map/odom", 10);
        planner_depth_pub_ = nh_.advertise<sensor_msgs::Image>("grid_map/depth", 10);
        
        // Debug publishers
        raw_depth_pub_ = nh_.advertise<sensor_msgs::Image>("vk180/raw_depth", 10);
        transformed_depth_pub_ = nh_.advertise<sensor_msgs::Image>("vk180/body_frame_depth", 10);
        disparity_pub_ = nh_.advertise<sensor_msgs::Image>("vk180/disparity", 10);
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("vk180/pointcloud", 10);
        image_pub_ = nh_.advertise<sensor_msgs::Image>("vk180/image", 10);
        
        // Parameters
        pnh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
        pnh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
        pnh_.param<bool>("use_body_frame_transform", use_body_frame_transform_, true);
        pnh_.param<bool>("subscribe_to_image_stream", subscribe_to_image_stream_, true);  // Enable for RGB data
        pnh_.param<double>("max_depth", max_depth_, 10.0);
        pnh_.param<double>("min_depth", min_depth_, 0.1);
        
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
    ros::Publisher planner_odom_pub_;
    ros::Publisher planner_depth_pub_;
    ros::Publisher raw_depth_pub_;
    ros::Publisher transformed_depth_pub_;
    ros::Publisher disparity_pub_;
    ros::Publisher pointcloud_pub_;
    ros::Publisher image_pub_;
    
    std::string odom_frame_id_;
    std::string base_frame_id_;
    bool use_body_frame_transform_;
    bool subscribe_to_image_stream_;
    double max_depth_, min_depth_;
    
    std::unique_ptr<vkc::VisualKit> visualkit_;
    
    // Camera calibration data
    CameraExtrinsics stereo1_l_extrinsics_;
    CameraIntrinsics stereo1_l_intrinsics_;
    
    // Point cloud generation parameters
    vkc::PointCloudParams pc_params_;
    
    // Latest image data for point cloud generation
    std::mutex image_mutex_;
    cv::Mat latest_image_;
    
    void initializeEcal() {
        visualkit_ = vkc::VisualKit::create(std::nullopt);
        
        if (visualkit_ == nullptr) {
            ROS_ERROR("Failed to create visualkit connection.");
            return;
        }

        // Install receivers
        auto odom_receiver = std::make_unique<OdometryReceiver>(*this);
        visualkit_->source().install("S1/vio_odom", std::move(odom_receiver));

        // Only subscribe to image stream if body frame transform is needed
        if (subscribe_to_image_stream_) {
            ROS_INFO("Creating ImageReceiver for S1/stereo1_l stream...");
            auto image_receiver = std::make_unique<ImageReceiver>(*this);
            ROS_INFO("Installing image receiver on stream: S1/stereo1_l");
            visualkit_->source().install("S1/stereo1_l", std::move(image_receiver));
            ROS_INFO("Successfully subscribed to S1/stereo1_l image stream for extrinsics");
        } else {
            ROS_INFO("Skipping S1/stereo1_l image stream subscription (subscribe_to_image_stream=false)");
        }

        auto disparity_receiver = std::make_unique<DisparityReceiver>(*this);
        visualkit_->source().install("S1/stereo1_l/disparity", std::move(disparity_receiver));

        visualkit_->source().start();
        
        ROS_INFO("VK180 Planner Bridge initialized");
        if (subscribe_to_image_stream_) {
            ROS_INFO("Waiting for stereo1_l extrinsics extraction...");
        }
    }

    // Extract extrinsics from stereo1_l image stream
    class ImageReceiver : public vkc::Receiver<vkc::Image> {
    public:
        explicit ImageReceiver(VK180PlannerBridge& integration) : integration_(integration), message_count_(0) {
            ROS_INFO("ImageReceiver constructor called");
        }

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Image>>& message) override {
            message_count_++;
            
            try {
                auto image = message.payload.reader();
                
                // Log first image reception
                if (message_count_ == 1) {
                    ROS_INFO("!!! RECEIVED FIRST IMAGE MESSAGE from stream: %s !!!", image.getStreamName().cStr());
                    ROS_INFO("Image dimensions: %dx%d", image.getWidth(), image.getHeight());
                }
                
                // Always log every 30 messages to prove we're receiving
                if (message_count_ % 30 == 1) {
                    ROS_INFO("IMAGE STREAM ALIVE: Received image #%d from %s", 
                             message_count_, image.getStreamName().cStr());
                }
                
                // Process all images from S1/stereo1_l subscription
                // Note: getStreamName() may return "stereo1_l_encoded" or "S1/stereo1_l" 
                // but we subscribed to the correct topic, so process all images
                std::string stream_name = image.getStreamName().cStr();
                ROS_INFO_THROTTLE(5.0, "Processing image (message #%d, stream: %s)", 
                                 message_count_, stream_name.c_str());
                
                // Extract extrinsics for body frame transformation (same way as ecal_ros_bridge_node)
                integration_.extractExtrinsics(image);
                integration_.extractIntrinsics(image);
                // Store latest image for point cloud generation (RGB data)
                integration_.storeLatestImage(image);
                
            } catch (const std::exception& e) {
                ROS_ERROR("Exception in ImageReceiver::handle: %s", e.what());
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

    class DisparityReceiver : public vkc::Receiver<vkc::Disparity> {
    public:
        explicit DisparityReceiver(VK180PlannerBridge& integration) : integration_(integration), message_count_(0) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Disparity>>& message) override {
            message_count_++;
            auto disparity_data = message.payload.reader();
            
            // Debug: Print disparity data info every 30 messages (roughly every second at 30Hz)
            if (message_count_ % 30 == 1) {
                ROS_INFO("DISPARITY DEBUG [Message #%d]:", message_count_);
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
                
                // Check data validity - interpret based on encoding
                auto data = disparity_data.getData();
                auto encoding = disparity_data.getEncoding();
                if (data.size() > 0) {
                    if (encoding == vkc::Disparity::Encoding::DISPARITY16) {
                        const uint16_t* disp_data = reinterpret_cast<const uint16_t*>(data.begin());
                        int num_pixels = data.size() / sizeof(uint16_t);
                        uint16_t min_val = *std::min_element(disp_data, disp_data + num_pixels);
                        uint16_t max_val = *std::max_element(disp_data, disp_data + num_pixels);
                        int valid_pixels = std::count_if(disp_data, disp_data + num_pixels, [](uint16_t d) { return d > 0; });
                        ROS_INFO("  Data range: [%u - %u], Valid pixels: %d/%d (%.1f%%)", 
                                 min_val, max_val, valid_pixels, num_pixels, 
                                 100.0 * valid_pixels / num_pixels);
                    } else {
                        ROS_INFO("  Data size: %zu bytes (encoding %d not DISPARITY16, skipping stats)", 
                                 data.size(), static_cast<int>(encoding));
                    }
                } else {
                    ROS_WARN("  No disparity data received!");
                }
            }
            
            try {
                integration_.publishDepthForPlanner(disparity_data);
                integration_.publishDisparityImage(disparity_data);
                // Pass the Shared<Disparity> to convertToPointCloud for VK SDK usage
                integration_.convertDisparityToPointCloud(message.payload);
                
                // Success feedback (throttled)
                if (message_count_ % 100 == 1) {
                    ROS_INFO("Successfully processed disparity message #%d", message_count_);
                }
            } catch (const std::exception& e) {
                ROS_ERROR("Error processing disparity data: %s", e.what());
            }
            
            return vkc::ReceiverStatus::Open;
        }

    private:
        VK180PlannerBridge& integration_;
        int message_count_;
    };

    void extractExtrinsics(const vkc::Image::Reader& image) {
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

    void storeLatestImage(const vkc::Image::Reader& image) {
        uint32_t image_height = image.getHeight();
        uint32_t image_width = image.getWidth();
        auto image_encoding = image.getEncoding();
        
        cv::Mat image_mat;
        
        // Convert image to RGB format (same logic as PointCloudGenerator.cpp)
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
            ROS_WARN_THROTTLE(5.0, "Unsupported image encoding: %d", static_cast<int>(image_encoding));
            return;
        }
        
        if (!image_mat.empty()) {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_ = image_mat.clone();
            ROS_INFO_THROTTLE(10.0, "Stored latest image: %dx%d, encoding=%d", 
                             image_width, image_height, static_cast<int>(image_encoding));
        }
        
        // Publish the image
        if (!image_mat.empty() && image_pub_.getNumSubscribers() > 0) {
            publishImage(image_mat);
        }
    }

    void initializePointCloudParams() {
        // Initialize point cloud conversion parameters (similar to PointCloudGenerator.cpp)
        pc_params_.disparityOffset = 0;  // Can be made configurable via ROS params
        pc_params_.maxDepth = max_depth_;
        ROS_INFO("Initialized point cloud parameters: disparityOffset=%d, maxDepth=%.2f", 
                 pc_params_.disparityOffset, pc_params_.maxDepth);
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
        
        ROS_INFO_THROTTLE(5.0, "Processing disparity for depth conversion [%d]", publish_count);
        
        // Extract intrinsics from disparity data if not done yet
        if (!stereo1_l_intrinsics_.is_valid) {
            ROS_INFO("Extracting camera intrinsics from disparity data...");
            stereo1_l_intrinsics_.fx = disparity_data.getFx();
            stereo1_l_intrinsics_.fy = disparity_data.getFy();
            stereo1_l_intrinsics_.cx = disparity_data.getCx();
            stereo1_l_intrinsics_.cy = disparity_data.getCy();
            stereo1_l_intrinsics_.baseline = disparity_data.getBaseline();
            stereo1_l_intrinsics_.is_valid = true;
            
            ROS_INFO("Extracted stereo1_l intrinsics from disparity");
            ROS_INFO("fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f, baseline=%.3f", 
                     stereo1_l_intrinsics_.fx, stereo1_l_intrinsics_.fy,
                     stereo1_l_intrinsics_.cx, stereo1_l_intrinsics_.cy,
                     stereo1_l_intrinsics_.baseline);
        }

        if (!stereo1_l_extrinsics_.is_valid && use_body_frame_transform_) {
            ROS_WARN_THROTTLE(5.0, "Extrinsics not yet available - using camera frame instead of body frame");
            // Continue processing without body frame transform
        }

        ROS_INFO_THROTTLE(5.0, "Converting disparity to depth image...");
        
        // Convert disparity to depth
        cv::Mat depth_image = convertDisparityToDepth(disparity_data);
        
        if (depth_image.empty()) {
            ROS_ERROR("Failed to convert disparity to depth - empty depth image");
            return;
        }
        
        ROS_INFO_THROTTLE(5.0, "Publishing raw depth image (size: %dx%d)", depth_image.cols, depth_image.rows);
        
        // Publish raw depth for debugging
        publishDepthImage(depth_image, raw_depth_pub_, "camera_optical_frame");
        
        if (use_body_frame_transform_ && stereo1_l_extrinsics_.is_valid) {
            ROS_INFO_THROTTLE(5.0, "Transforming depth to body frame...");
            // Transform depth points to body frame
            cv::Mat body_frame_depth = transformDepthToBodyFrame(depth_image);
            
            if (body_frame_depth.empty()) {
                ROS_ERROR("Failed to transform depth to body frame");
                return;
            }
            
            ROS_INFO_THROTTLE(5.0, "Publishing body frame depth for planner");
            publishDepthImage(body_frame_depth, transformed_depth_pub_, base_frame_id_);
            
            // Use body frame depth for planner
            publishDepthImage(body_frame_depth, planner_depth_pub_, base_frame_id_);
        } else {
            if (use_body_frame_transform_ && !stereo1_l_extrinsics_.is_valid) {
                ROS_INFO_THROTTLE(5.0, "Body frame transform requested but extrinsics unavailable - using camera frame");
            }
            ROS_INFO_THROTTLE(5.0, "Publishing camera frame depth for planner");
            // Use camera frame depth directly
            publishDepthImage(depth_image, planner_depth_pub_, "camera_optical_frame");
        }
        
        ROS_INFO_THROTTLE(5.0, "Depth processing pipeline completed successfully [%d processed]", publish_count);
    }

    cv::Mat convertDisparityToDepth(const vkc::Disparity::Reader& disparity_data) {
        static int conversion_count = 0;
        conversion_count++;
        
        int width = disparity_data.getWidth();
        int height = disparity_data.getHeight();
        
        ROS_INFO_THROTTLE(5.0, "Converting disparity to depth [%d] (size: %dx%d)", 
                         conversion_count, width, height);
        
        // Get disparity values
        auto disparity_values = disparity_data.getData();
        auto encoding = disparity_data.getEncoding();
        
        if (!disparity_values.size()) {
            ROS_ERROR("Empty disparity data received");
            return cv::Mat();
        }
        
        ROS_INFO_THROTTLE(5.0, "Disparity data size: %lu bytes, encoding: %d", 
                         disparity_values.size(), static_cast<int>(encoding));
        
        // Interpret raw bytes based on encoding
        const uint16_t* disp_data = nullptr;
        int num_pixels = 0;
        
        if (encoding == vkc::Disparity::Encoding::DISPARITY16) {
            disp_data = reinterpret_cast<const uint16_t*>(disparity_values.begin());
            num_pixels = disparity_values.size() / sizeof(uint16_t);
        } else {
            ROS_ERROR("Unsupported disparity encoding: %d (expected DISPARITY16)", static_cast<int>(encoding));
            return cv::Mat();
        }
        
        if (num_pixels != width * height) {
            ROS_WARN("Pixel count mismatch: got %d, expected %d", num_pixels, width * height);
        }
        
        // Create depth image (16UC1 format in millimeters for planner compatibility)
        cv::Mat depth_image(height, width, CV_16UC1);
        
        int valid_pixels = 0;
        float min_disparity = FLT_MAX, max_disparity = -FLT_MAX;
        float sum_disparity = 0.0f;
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;
                // For DISPARITY16: disparity value is stored as disparity*8
                float disparity = static_cast<float>(disp_data[idx]) / 8.0f;
                
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
        
        ROS_INFO_THROTTLE(5.0, "Disparity stats: valid=%d/%d (%.1f%%), range=[%.2f,%.2f], avg=%.2f", 
                         valid_pixels, width * height, depth_validity_ratio * 100.0f,
                         min_disparity, max_disparity, avg_disparity);
        
        if (valid_pixels == 0) {
            ROS_WARN("No valid disparity pixels found - depth image will be empty");
        }
        
        ROS_INFO_THROTTLE(5.0, "Depth conversion completed: %dx%d, validity=%.1f%%", 
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

    void convertDisparityToPointCloud(const vkc::Shared<vkc::Disparity>& disparity_shared) {
        static int pointcloud_count = 0;
        pointcloud_count++;
        
        ROS_INFO_THROTTLE(5.0, "Converting disparity to point cloud [%d]", pointcloud_count);
        
        // Check if publisher has subscribers
        if (pointcloud_pub_.getNumSubscribers() == 0) {
            ROS_INFO_THROTTLE(10.0, "No subscribers for point cloud topic");
            return; // Skip processing if no one is listening
        }
        
        // Get image data for VK SDK conversion (RGB data used internally by SDK)
        // IMPORTANT: Keep image_copy alive for the entire function to avoid dangling pointer
        cv::Mat image_copy;
        unsigned char* image_data = nullptr;
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            if (!latest_image_.empty()) {
                image_copy = latest_image_.clone();
                ROS_INFO_THROTTLE(10.0, "Using image data: %dx%d for point cloud generation", 
                                 image_copy.cols, image_copy.rows);
            } else {
                ROS_WARN_THROTTLE(10.0, "No image data available, generating point cloud without RGB");
            }
        }
        
        // Set image_data pointer only if we have a valid image
        if (!image_copy.empty()) {
            image_data = image_copy.data;
        }
        
        // Use VK SDK's convertToPointCloud function
        // Pass image data (SDK uses it for RGB, but we won't export RGB to final cloud)
        vkc::Shared<vkc::PointCloud> vk_pointcloud = vkc::convertToPointCloud(disparity_shared, image_data, pc_params_);
        
        auto pc_reader = vk_pointcloud.reader();
        auto points_data = pc_reader.getPoints().asBytes();
        int point_stride = static_cast<int>(pc_reader.getPointStride());
        int points_count = points_data.size() / point_stride;
        
        ROS_INFO_THROTTLE(8.0, "VK SDK point cloud: %d points, stride=%d bytes", 
                         points_count, point_stride);
        
        // Extract field offsets for X, Y, Z (ignore RGB fields)
        std::vector<int> field_offsets;
        for (const auto& field : pc_reader.getFields()) {
            field_offsets.push_back(static_cast<int>(field.getOffset()));
        }
        
        if (field_offsets.size() < 3) {
            ROS_ERROR("VK point cloud has insufficient fields (need at least X,Y,Z)");
            return;
        }
        
        // Create PCL point cloud (PointXYZ - no RGB colors)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.reserve(points_count);
        
        unsigned char* pc_data = const_cast<unsigned char*>(points_data.begin());
        int valid_points = 0;
        
        for (int i = 0; i < points_count; ++i) {
            unsigned char* point_ptr = pc_data + i * point_stride;
            
            float x = *reinterpret_cast<float*>(point_ptr + field_offsets[0]);
            float y = *reinterpret_cast<float*>(point_ptr + field_offsets[1]);
            float z = *reinterpret_cast<float*>(point_ptr + field_offsets[2]);
            // Note: RGB fields (field_offsets[3,4,5]) exist if image_data was provided, but we ignore them
            
            // Skip invalid points (NaN or zero depth)
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && z > 0) {
                pcl::PointXYZ point;
                point.x = x;
                point.y = y;
                point.z = z;
                
                // Transform to body frame if enabled
                if (use_body_frame_transform_ && stereo1_l_extrinsics_.is_valid) {
                    Eigen::Vector4d camera_point_h(x, y, z, 1.0);
                    Eigen::Vector4d body_point_h = stereo1_l_extrinsics_.cam_to_body_transform * camera_point_h;
                    
                    point.x = body_point_h.x();
                    point.y = body_point_h.y();
                    point.z = body_point_h.z();
                }
                
                cloud->points.push_back(point);
                valid_points++;
            }
        }
        
        cloud->width = valid_points;
        cloud->height = 1;
        cloud->is_dense = false;
        
        float validity_ratio = static_cast<float>(valid_points) / points_count;
        ROS_INFO_THROTTLE(8.0, "Point cloud stats: %d/%d valid points (%.1f%%)", 
                         valid_points, points_count, validity_ratio * 100.0f);
        
        // Convert to ROS message and publish
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        
        cloud_msg.header.stamp = ros::Time::now();
        if (use_body_frame_transform_ && stereo1_l_extrinsics_.is_valid) {
            cloud_msg.header.frame_id = base_frame_id_; // Body frame
        } else {
            cloud_msg.header.frame_id = "camera_optical_frame"; // Camera frame
        }
        
        try {
            pointcloud_pub_.publish(cloud_msg);
            ROS_INFO_THROTTLE(8.0, "Successfully published point cloud with %d points in frame: %s", 
                             valid_points, cloud_msg.header.frame_id.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Exception publishing point cloud: %s", e.what());
        }
    }

    void publishDepthImage(const cv::Mat& depth_image, ros::Publisher& pub, const std::string& frame_id) {
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
        
        ROS_INFO_THROTTLE(8.0, "Depth stats for %s: mean=%.2f, std=%.2f, min=%.2f, max=%.2f", 
                         frame_id.c_str(), mean_val[0], std_val[0], min_val, max_val);
        
        try {
            pub.publish(depth_msg);
            ROS_INFO_THROTTLE(8.0, "Successfully published depth image to %s", frame_id.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Exception publishing depth image to %s: %s", frame_id.c_str(), e.what());
        }
    }

    void publishDisparityImage(const vkc::Disparity::Reader& disparity_data) {
        static int disparity_pub_count = 0;
        disparity_pub_count++;
        
        int width = disparity_data.getWidth();
        int height = disparity_data.getHeight();
        
        ROS_INFO_THROTTLE(3.0, "Publishing disparity image [%d] (size: %dx%d)", 
                         disparity_pub_count, width, height);
        
        // Check if publisher has subscribers
        if (disparity_pub_.getNumSubscribers() == 0) {
            ROS_INFO_THROTTLE(10.0, "No subscribers for disparity topic");
            return; // Skip processing if no one is listening
        }
        
        ROS_INFO_THROTTLE(5.0, "%d subscriber(s) for disparity topic", 
                         disparity_pub_.getNumSubscribers());
        
        // Get disparity values
        auto disparity_values = disparity_data.getData();
        
        if (!disparity_values.size()) {
            ROS_ERROR("Empty disparity data - cannot publish");
            return;
        }
        
        sensor_msgs::Image disparity_msg;
        disparity_msg.header.stamp = ros::Time::now();
        disparity_msg.header.frame_id = "camera_optical_frame";
        disparity_msg.height = height;
        disparity_msg.width = width;
        disparity_msg.step = disparity_data.getStep();
        
        // Set encoding based on eCAL disparity encoding (same as ecal_ros_bridge_node.cpp)
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
        
        disparity_msg.is_bigendian = false;
        
        // Copy image data - same as ecal_ros_bridge_node.cpp
        auto data = disparity_data.getData();
        disparity_msg.data.resize(data.size());
        std::memcpy(disparity_msg.data.data(), data.begin(), data.size());
        
        try {
            disparity_pub_.publish(disparity_msg);
            ROS_INFO_THROTTLE(8.0, "Successfully published disparity image");
        } catch (const std::exception& e) {
            ROS_ERROR("Exception publishing disparity image: %s", e.what());
        }
    }
    
    void publishImage(const cv::Mat& image) {
        static int image_pub_count = 0;
        image_pub_count++;
        
        if (image.empty()) {
            ROS_ERROR("Cannot publish empty image");
            return;
        }
        
        ROS_INFO_THROTTLE(5.0, "Publishing RGB image [%d] (size: %dx%d)", 
                         image_pub_count, image.cols, image.rows);
        
        sensor_msgs::Image image_msg;
        image_msg.header.stamp = ros::Time::now();
        image_msg.header.frame_id = "camera_optical_frame";
        image_msg.height = image.rows;
        image_msg.width = image.cols;
        image_msg.encoding = "rgb8";
        image_msg.is_bigendian = false;
        image_msg.step = image.cols * 3;  // 3 bytes per pixel for RGB
        
        image_msg.data.resize(image_msg.step * image_msg.height);
        std::memcpy(image_msg.data.data(), image.data, image_msg.data.size());
        
        try {
            image_pub_.publish(image_msg);
            ROS_INFO_THROTTLE(10.0, "Successfully published RGB image");
        } catch (const std::exception& e) {
            ROS_ERROR("Exception publishing RGB image: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vk180_planner_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    ROS_INFO("Starting VK180 Planner Bridge with Body Frame Transformation");
    
    VK180PlannerBridge integration(nh, pnh);
    
    ros::spin();
    
    return 0;
}