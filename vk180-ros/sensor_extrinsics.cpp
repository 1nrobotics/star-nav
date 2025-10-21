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

//print Vector3d
void printVector3d(const vkc::Vector3d::Reader& vector3d) {
    std::cout << "Position [x=" << vector3d.getX() 
              << ", y=" << vector3d.getY() 
              << ", z=" << vector3d.getZ() << "]" << std::endl;
}

//print So3
void printSo3(const vkc::So3::Reader& So3) {
    std::cout << "Quaternion [w=" << So3.getW() 
              << ", x=" << So3.getX() 
              << ", y=" << So3.getY() 
              << ", z=" << So3.getZ() << "]" << std::endl;
}

//print Se3
void printSe3(const vkc::Se3::Reader& Se3) {
    std::cout << "  Position: ";
    printVector3d(Se3.getPosition());
    std::cout << "  Orientation: ";
    printSo3(Se3.getOrientation());
}

//print SensorExtrinsic
void printSensorExtrinsic(const vkc::SensorExtrinsic::Reader& SensorExtrinsic) {
    std::cout << "┌─ Sensor Extrinsics ─────────────────────┐" << std::endl;
    std::cout << "│ Body Frame Transform:                    │" << std::endl;
    printSe3(SensorExtrinsic.getBodyFrame());
    std::cout << "│ IMU Frame Transform:                     │" << std::endl;
    printSe3(SensorExtrinsic.getImuFrame());
    std::cout << "│ Last Modified: " << std::setw(25) << SensorExtrinsic.getLastModified() << " │" << std::endl;
    std::cout << "└─────────────────────────────────────────┘" << std::endl;
}

//print Header
void printHeader(const vkc::Header::Reader& header) {
    std::cout << "Header - Seq: " << header.getSeq() 
              << ", Timestamp: " << header.getStampMonotonic()
              << ", Frame: " << header.getFrameId().cStr() << std::endl;
}

// Receiver for Image data (contains sensor extrinsics)
class ImageExtrinsicsReceiver : public vkc::Receiver<vkc::Image> {
private:
    std::string stream_name_;
    bool received_data_;
    
public:
    ImageExtrinsicsReceiver(const std::string& stream_name) 
        : stream_name_(stream_name), received_data_(false) {}
    
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Image>>& message) override {
        auto image = message.payload.reader();
        
        std::cout << "\n=============================================" << std::endl;
        std::cout << "📷 IMAGE STREAM: " << stream_name_ << std::endl;
        std::cout << "Stream Name: " << image.getStreamName().cStr() << std::endl;
        
        printHeader(image.getHeader());
        
        std::cout << "📐 Image Info: " << image.getWidth() << "x" << image.getHeight() 
                  << ", Sensor ID: " << static_cast<int>(image.getSensorIdx()) << std::endl;
        
        // Print sensor extrinsics
        std::cout << "\n🔄 SENSOR EXTRINSICS:" << std::endl;
        printSensorExtrinsic(image.getExtrinsic());
        
        std::cout << "=============================================" << std::endl;
        
        received_data_ = true;
        return vkc::ReceiverStatus::Open;
    }
    
    bool hasReceivedData() const { return received_data_; }
};

// Receiver for Disparity data (contains pinhole rotation extrinsics)
class DisparityExtrinsicsReceiver : public vkc::Receiver<vkc::Disparity> {
private:
    std::string stream_name_;
    bool received_data_;
    
public:
    DisparityExtrinsicsReceiver(const std::string& stream_name) 
        : stream_name_(stream_name), received_data_(false) {}
    
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Disparity>>& message) override {
        auto disparity = message.payload.reader();
        
        std::cout << "\n=============================================" << std::endl;
        std::cout << "📊 DISPARITY STREAM: " << stream_name_ << std::endl;
        std::cout << "Stream Name: " << disparity.getStreamName().cStr() << std::endl;
        
        printHeader(disparity.getHeader());
        
        std::cout << "📐 Disparity Info: " << disparity.getWidth() << "x" << disparity.getHeight() 
                  << ", Baseline: " << disparity.getBaseline() << "m" << std::endl;
        
        std::cout << "📷 Camera Intrinsics: fx=" << disparity.getFx() 
                  << ", fy=" << disparity.getFy()
                  << ", cx=" << disparity.getCx() 
                  << ", cy=" << disparity.getCy() << std::endl;
        
        // Print stereo camera rotation (extrinsics)
        std::cout << "\n🔄 STEREO CAMERA EXTRINSICS:" << std::endl;
        std::cout << "Pinhole Rotation (Stereo -> Main Camera): ";
        printSo3(disparity.getPinholeRotation());
        
        std::cout << "=============================================" << std::endl;
        
        received_data_ = true;
        return vkc::ReceiverStatus::Open;
    }
    
    bool hasReceivedData() const { return received_data_; }
};

int main(int argc, char* argv[]) {
    std::cout << "🔍 VK180 Complete Sensor Extrinsics Extractor" << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "This tool will scan ALL available image and disparity streams" << std::endl;
    std::cout << "to extract comprehensive sensor extrinsics information:" << std::endl;
    std::cout << "📷 Image streams: Body frame and IMU frame transforms" << std::endl;
    std::cout << "📊 Disparity streams: Stereo camera rotation matrices" << std::endl;
    std::cout << "=============================================" << std::endl;
    
    auto visualkit = vkc::VisualKit::create(std::nullopt);
    
    if (visualkit == nullptr) {
        std::cout << "Failed to create visualkit connection." << std::endl;
        return -1;
    }
    
    // Create receivers for different streams that contain extrinsics
    std::cout << "Setting up receivers for all image streams..." << std::endl;
    
    // Install receivers for all common VK180 image streams
    // S0 sensor streams
    auto s0_stereo1_l_receiver = std::make_unique<ImageExtrinsicsReceiver>("S0/stereo1_l");
    visualkit->source().install("S0/stereo1_l", std::move(s0_stereo1_l_receiver));
    
    auto s0_stereo1_r_receiver = std::make_unique<ImageExtrinsicsReceiver>("S0/stereo1_r");
    visualkit->source().install("S0/stereo1_r", std::move(s0_stereo1_r_receiver));
    
    auto s0_stereo2_l_receiver = std::make_unique<ImageExtrinsicsReceiver>("S0/stereo2_l");
    visualkit->source().install("S0/stereo2_l", std::move(s0_stereo2_l_receiver));
    
    auto s0_stereo2_r_receiver = std::make_unique<ImageExtrinsicsReceiver>("S0/stereo2_r");
    visualkit->source().install("S0/stereo2_r", std::move(s0_stereo2_r_receiver));
    
    auto s0_panorama_receiver = std::make_unique<ImageExtrinsicsReceiver>("S0/panorama");
    visualkit->source().install("S0/panorama", std::move(s0_panorama_receiver));
    
    auto s0_main_receiver = std::make_unique<ImageExtrinsicsReceiver>("S0/main");
    visualkit->source().install("S0/main", std::move(s0_main_receiver));
    
    // S1 sensor streams
    auto s1_stereo1_l_receiver = std::make_unique<ImageExtrinsicsReceiver>("S1/stereo1_l");
    visualkit->source().install("S1/stereo1_l", std::move(s1_stereo1_l_receiver));
    
    auto s1_stereo1_r_receiver = std::make_unique<ImageExtrinsicsReceiver>("S1/stereo1_r");
    visualkit->source().install("S1/stereo1_r", std::move(s1_stereo1_r_receiver));
    
    auto s1_stereo2_l_receiver = std::make_unique<ImageExtrinsicsReceiver>("S1/stereo2_l");
    visualkit->source().install("S1/stereo2_l", std::move(s1_stereo2_l_receiver));
    
    auto s1_stereo2_r_receiver = std::make_unique<ImageExtrinsicsReceiver>("S1/stereo2_r");
    visualkit->source().install("S1/stereo2_r", std::move(s1_stereo2_r_receiver));
    
    auto s1_panorama_receiver = std::make_unique<ImageExtrinsicsReceiver>("S1/panorama");
    visualkit->source().install("S1/panorama", std::move(s1_panorama_receiver));
    
    auto s1_main_receiver = std::make_unique<ImageExtrinsicsReceiver>("S1/main");
    visualkit->source().install("S1/main", std::move(s1_main_receiver));
    
    std::cout << "Setting up receivers for disparity streams..." << std::endl;
    
    // Install receivers for disparity streams (stereo camera extrinsics)
    auto s0_disparity_l_receiver = std::make_unique<DisparityExtrinsicsReceiver>("S0/stereo1_l/disparity");
    visualkit->source().install("S0/stereo1_l/disparity", std::move(s0_disparity_l_receiver));
    
    auto s0_disparity_r_receiver = std::make_unique<DisparityExtrinsicsReceiver>("S0/stereo2_r/disparity");
    visualkit->source().install("S0/stereo2_r/disparity", std::move(s0_disparity_r_receiver));
    
    auto s1_disparity_l_receiver = std::make_unique<DisparityExtrinsicsReceiver>("S1/stereo1_l/disparity");
    visualkit->source().install("S1/stereo1_l/disparity", std::move(s1_disparity_l_receiver));
    
    auto s1_disparity_r_receiver = std::make_unique<DisparityExtrinsicsReceiver>("S1/stereo2_r/disparity");
    visualkit->source().install("S1/stereo2_r/disparity", std::move(s1_disparity_r_receiver));
    
    std::cout << "\nStarting data collection..." << std::endl;
    visualkit->source().start();
    
    std::cout << "Listening for sensor extrinsics data for 10 seconds..." << std::endl;
    std::cout << "Press Ctrl+C to stop early." << std::endl;
    
    // Wait for data for up to 10 seconds
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(5);
    
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Check if we should exit early (this is a simple approach)
        // In a real implementation, you might want to track received data more carefully
    }
    
    std::cout << "\nData collection complete." << std::endl;
    std::cout << "Check the output above for sensor extrinsics information." << std::endl;
    
    // Stop the data source
    visualkit->source().stop(false);
    
    return 0;
}