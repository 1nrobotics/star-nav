#include <memory>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <vk_sdk/Sdk.hpp>
#include <vk_sdk/capnp/image.capnp.h>
#include <cmath>
#include <array>
#include <string>
#include <cstring>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct CameraExtrinsics {
    // Body frame transform (camera to body)
    Eigen::Vector3d body_position;
    Eigen::Quaterniond body_rotation;
    
    // IMU frame transform  
    Eigen::Vector3d imu_position;
    Eigen::Quaterniond imu_rotation;
    
    bool is_valid = false;
};

// Global storage for extracted extrinsics
CameraExtrinsics g_stereo1_l_extrinsics;

//print Vector3d and extract to Eigen
void printAndExtractVector3d(const vkc::Vector3d::Reader& vector3d, Eigen::Vector3d& eigen_vec) {
    eigen_vec.x() = vector3d.getX();
    eigen_vec.y() = vector3d.getY(); 
    eigen_vec.z() = vector3d.getZ();
    std::cout << "Position [x=" << eigen_vec.x() 
              << ", y=" << eigen_vec.y() 
              << ", z=" << eigen_vec.z() << "]" << std::endl;
}

//print So3 and extract to Eigen
void printAndExtractSo3(const vkc::So3::Reader& So3, Eigen::Quaterniond& eigen_quat) {
    eigen_quat.w() = So3.getW();
    eigen_quat.x() = So3.getX();
    eigen_quat.y() = So3.getY();
    eigen_quat.z() = So3.getZ();
    std::cout << "Quaternion [w=" << eigen_quat.w() 
              << ", x=" << eigen_quat.x() 
              << ", y=" << eigen_quat.y() 
              << ", z=" << eigen_quat.z() << "]" << std::endl;
}

//print Se3 and extract to Eigen
void printAndExtractSe3(const vkc::Se3::Reader& Se3, Eigen::Vector3d& position, Eigen::Quaterniond& rotation) {
    std::cout << "  Position: ";
    printAndExtractVector3d(Se3.getPosition(), position);
    std::cout << "  Orientation: ";
    printAndExtractSo3(Se3.getOrientation(), rotation);
}

//print SensorExtrinsic and extract to global storage
void extractSensorExtrinsic(const vkc::SensorExtrinsic::Reader& SensorExtrinsic) {
    std::cout << "🔄 EXTRACTING SENSOR EXTRINSICS FOR DEPTH CONVERSION" << std::endl;
    std::cout << "┌─ Sensor Extrinsics ─────────────────────┐" << std::endl;
    
    std::cout << "│ Body Frame Transform (Camera → Body):    │" << std::endl;
    printAndExtractSe3(SensorExtrinsic.getBodyFrame(), 
                      g_stereo1_l_extrinsics.body_position, 
                      g_stereo1_l_extrinsics.body_rotation);
    
    std::cout << "│ IMU Frame Transform:                     │" << std::endl;
    printAndExtractSe3(SensorExtrinsic.getImuFrame(), 
                      g_stereo1_l_extrinsics.imu_position, 
                      g_stereo1_l_extrinsics.imu_rotation);
    
    std::cout << "│ Last Modified: " << std::setw(25) << SensorExtrinsic.getLastModified() << " │" << std::endl;
    std::cout << "└─────────────────────────────────────────┘" << std::endl;
    
    g_stereo1_l_extrinsics.is_valid = true;
    
    // Create transformation matrix for depth conversion
    Eigen::Matrix4d cam_to_body = Eigen::Matrix4d::Identity();
    cam_to_body.block<3,3>(0,0) = g_stereo1_l_extrinsics.body_rotation.toRotationMatrix();
    cam_to_body.block<3,1>(0,3) = g_stereo1_l_extrinsics.body_position;
    
    std::cout << "\n📐 CAMERA TO BODY TRANSFORMATION MATRIX:" << std::endl;
    std::cout << cam_to_body << std::endl;
    
    std::cout << "\n✅ Extrinsics extracted and ready for depth conversion!" << std::endl;
}

// Function to convert a 3D point from camera frame to body frame
Eigen::Vector3d convertCameraToBodyFrame(const Eigen::Vector3d& camera_point) {
    if (!g_stereo1_l_extrinsics.is_valid) {
        std::cerr << "❌ Error: Extrinsics not yet extracted!" << std::endl;
        return camera_point;
    }
    
    // Apply rotation and translation: body_point = R * cam_point + t
    Eigen::Vector3d body_point = g_stereo1_l_extrinsics.body_rotation * camera_point + g_stereo1_l_extrinsics.body_position;
    
    return body_point;
}

// Function to demonstrate depth point conversion for planner integration
void demonstrateDepthConversion() {
    if (!g_stereo1_l_extrinsics.is_valid) {
        std::cout << "⏳ Waiting for extrinsics to be extracted..." << std::endl;
        return;
    }
    
    std::cout << "\n🧮 DEPTH CONVERSION DEMONSTRATION:" << std::endl;
    std::cout << "================================================" << std::endl;
    
    // Example depth points in camera frame (stereo1_l optical frame)
    std::vector<Eigen::Vector3d> camera_points = {
        {0.0, 0.0, 1.0},    // 1m straight ahead
        {0.5, 0.0, 1.0},    // 1m ahead, 0.5m right
        {0.0, 0.3, 2.0},    // 2m ahead, 0.3m up
        {-0.2, -0.1, 0.5}   // 0.5m ahead, 0.2m left, 0.1m down
    };
    
    std::cout << "Converting sample depth points from camera frame to body frame:" << std::endl;
    
    for (size_t i = 0; i < camera_points.size(); ++i) {
        Eigen::Vector3d camera_point = camera_points[i];
        Eigen::Vector3d body_point = convertCameraToBodyFrame(camera_point);
        
        std::cout << "Point " << (i+1) << ":" << std::endl;
        std::cout << "  Camera frame: [" << camera_point.transpose() << "]" << std::endl;
        std::cout << "  Body frame:   [" << body_point.transpose() << "]" << std::endl;
        std::cout << std::endl;
    }
    
    std::cout << "🎯 HOW TO INTEGRATE WITH PLANNER:" << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << "1. Extract depth from VK180 disparity data" << std::endl;
    std::cout << "2. Convert pixel coordinates to 3D camera points using intrinsics" << std::endl;
    std::cout << "3. Apply the above transformation to convert to body frame" << std::endl;
    std::cout << "4. Use body frame points for occupancy grid mapping" << std::endl;
    std::cout << "5. Body frame coordinates work directly with robot odometry" << std::endl;
}

// Receiver for Image data to extract stereo1_l extrinsics
class Stereo1LExtrinsicsReceiver : public vkc::Receiver<vkc::Image> {
public:
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Image>>& message) override {
        auto image = message.payload.reader();
        
        std::cout << "\n=============================================" << std::endl;
        std::cout << "📷 FOUND S1/stereo1_l STREAM!" << std::endl;
        std::cout << "Stream Name: " << image.getStreamName().cStr() << std::endl;
        
        std::cout << "📐 Image Info: " << image.getWidth() << "x" << image.getHeight() 
                  << ", Sensor ID: " << static_cast<int>(image.getSensorIdx()) << std::endl;
        
        // Extract and store sensor extrinsics
        extractSensorExtrinsic(image.getExtrinsic());
        
        // Demonstrate depth conversion
        demonstrateDepthConversion();
        
        std::cout << "=============================================" << std::endl;
        
        return vkc::ReceiverStatus::Open;
    }
};

int main(int argc, char* argv[]) {
    std::cout << "🎯 VK180 Depth Conversion Test" << std::endl;
    std::cout << "===============================================" << std::endl;
    std::cout << "This tool extracts S1/stereo1_l extrinsics and" << std::endl;
    std::cout << "demonstrates camera-to-body frame conversion" << std::endl;
    std::cout << "for planner integration." << std::endl;
    std::cout << "===============================================" << std::endl;
    
    auto visualkit = vkc::VisualKit::create(std::nullopt);
    
    if (visualkit == nullptr) {
        std::cout << "Failed to create visualkit connection." << std::endl;
        return -1;
    }
    
    std::cout << "🔍 Setting up receiver for S1/stereo1_l..." << std::endl;
    
    // Install receiver specifically for S1/stereo1_l
    auto stereo1_l_receiver = std::make_unique<Stereo1LExtrinsicsReceiver>();
    visualkit->source().install("S1/stereo1_l", std::move(stereo1_l_receiver));
    
    std::cout << "\n⏳ Starting data collection..." << std::endl;
    visualkit->source().start();
    
    std::cout << "🎧 Listening for S1/stereo1_l extrinsics data..." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;
    
    // Wait for data for up to 30 seconds
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(30);
    
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // If we got the extrinsics, demonstrate conversion periodically
        if (g_stereo1_l_extrinsics.is_valid) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            std::cout << "\n🔄 Periodic demonstration of depth conversion..." << std::endl;
            demonstrateDepthConversion();
        }
    }
    
    std::cout << "\n✅ Data collection complete." << std::endl;
    
    // Stop the data source
    visualkit->source().stop(false);
    
    return 0;
}