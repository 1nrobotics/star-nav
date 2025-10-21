#include <memory>
#include <iostream>
#include <vk_sdk/Sdk.hpp>
#include <vk_sdk/capnp/image.capnp.h>
#include <cmath>
#include <array>
#include <string>
#include <cstring>

//print Vector3d
void printVector3d(const vkc::Vector3d::Reader& vector3d) {
    std::cout << "printVector3d:" << std::endl;
    std::cout << "3dX: " << vector3d.getX() << ", 3dY: " << vector3d.getY() << ", 3dZ: " << vector3d.getZ() << std::endl;
}
//print Header
void printHeader(const vkc::Header::Reader& header) {
    std::cout << "printHeader: " << std::endl;
    std::cout << "getSeq: " << header.getSeq() << std::endl;
    std::cout << "getStampMonotonic: " << header.getStampMonotonic() << std::endl;
    std::cout << "getFrameId: " << header.getFrameId().cStr() << std::endl;
    std::cout << "getClockDomain: ";
    switch (header.getClockDomain()) {
        case vkc::Header::ClockDomain::MONOTONIC:
            std::cout << "Clock domain: monotonic" << std::endl;
            break;
        case vkc::Header::ClockDomain::REALTIME:
            std::cout << "Clock domain: realtime" << std::endl;
            break;
        default:
            std::cout << "Clock domain: unknown" << std::endl;
    }
    std::cout << "getLatencyDevice: " << header.getLatencyDevice() << std::endl;
    std::cout << "getClockOffset: " << header.getClockOffset() << std::endl;
}
//print So3
void printSo3(const vkc::So3::Reader& So3) {
    std::cout << "printSo3: " << std::endl;
    std::cout<<"getW: "<<So3.getW()<<std::endl;
    std::cout<<"getX: "<<So3.getX()<<std::endl;
    std::cout<<"getY: "<<So3.getY()<<std::endl;
    std::cout<<"getZ: "<<So3.getZ()<<std::endl;
}
//print Se3
void printSe3(const vkc::Se3::Reader& Se3) {
    std::cout << "printSe3: " << std::endl;
    std::cout << "getPosition: ";
    printVector3d(Se3.getPosition());
    std::cout << "getOrientation: ";
    printSo3(Se3.getOrientation());
}
//print Twist3
void printTwist3(const vkc::Twist3::Reader& Twist3){
    std::cout << "printTwist3: " << std::endl;
    std::cout << "getLinear: ";
    printVector3d(Twist3.getLinear());
    std::cout << "getAngular: ";
    printVector3d(Twist3.getAngular()); 
}

//print Odometry3d
void printOdometry3d(const vkc::Odometry3d::Reader & Odometry3d){
    std::cout << "printOdometry3d: " << std::endl;
    printHeader(Odometry3d.getHeader());
    std::cout<<"getBodyFrame: ";
    switch (Odometry3d.getBodyFrame()) {
        case vkc::Odometry3d::BodyFrame::NED :
            std::cout << "NED" << std::endl;
            break;
        case vkc::Odometry3d::BodyFrame::NWU:
            std::cout << "NWU" << std::endl;
            break;
        default:
            std::cout << "unknown" << std::endl;
    }
    std::cout<<"getReferenceFrame: ";
    switch (Odometry3d.getReferenceFrame()) {
        case vkc::Odometry3d::ReferenceFrame::ENU :
            std::cout << "ENU" << std::endl;
            break;
        case vkc::Odometry3d::ReferenceFrame::NWU :
            std::cout << "NWU" << std::endl;
            break;
        case vkc::Odometry3d::ReferenceFrame::NED :
            std::cout << "NED" << std::endl;
            break;
        default:
            std::cout << "unknown" << std::endl;
    }
    std::cout<<"getVelocityFrame: ";
    switch (Odometry3d.getVelocityFrame()) {
        case vkc::Odometry3d::VelocityFrame::NONE :
            std::cout << "NONE" << std::endl;
            break;
        case vkc::Odometry3d::VelocityFrame::BODY :
            std::cout << "BODY" << std::endl;
            break;
        case vkc::Odometry3d::VelocityFrame::REFERENCE :
            std::cout << "REFERENCE" << std::endl;
            break;
        default:
            std::cout << "unknown" << std::endl;
    }
    printSe3(Odometry3d.getPose());
    for(const auto & aa: Odometry3d.getPoseCovariance()){
        std::cout<<"getPoseCovariance: "<<aa<<std::endl;
    }
    printTwist3(Odometry3d.getTwist());
    for(const auto & aa: Odometry3d.getTwistCovariance()){
        std::cout<<"getTwistCovariance: "<<aa<<std::endl;
    }
    std::cout<<"getResetCounter: "<<Odometry3d.getResetCounter()<<std::endl;
    std::cout<<"getResetCounter: "<<Odometry3d.getMetricVisionFailureLikelihood()<<std::endl;
    std::cout<<"getResetCounter: "<<Odometry3d.getMetricInertialFailureLikelihood()<<std::endl;
    std::cout<<"getResetCounter: "<<Odometry3d.getEstimatedFailureModeDrift()<<std::endl;
    std::cout<<"getResetCounter: "<<Odometry3d.getMetricFailureVio()<<std::endl;
}

class MyReceiver_Odometry3d: public vkc::Receiver<vkc::Odometry3d> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Odometry3d>>& message) override {
        auto Odometry3d = message.payload.reader();
        printOdometry3d(Odometry3d);
        return vkc::ReceiverStatus::Open;
    }
};

int main(int argc, char* argv[]) {
    auto visualkit = vkc::VisualKit::create(std::nullopt);
    // auto visualkit = vkc::VisualKit::create();

    // Check that the object has been created successfully before proceeding.
    if (visualkit == nullptr) {
        std::cout << "Failed to create visualkit connection." << std::endl;
        return -1;
    }
    
    // Create the receiver that was defined by us.
    auto myReceiver = std::make_unique<MyReceiver_Odometry3d>();

    // Install the receiver into the data source so that the receiver can receive messages.
    visualkit->source().install("S1/vio_odom", std::move(myReceiver));
    // visualkit->source().install("S0/stereo1_l",std::move(myReceiver));
    
    // Start the data source so messages can be received by `myReceiver`.
    visualkit->source().start();

    // Wait for the user to send the CTRL-C signal.
    vkc::waitForCtrlCSignal();

    // Stop the data source.
    //
    // You may pass `true` here if you want the soure to block until all data from it has been sent.
    // In this case, since data from visualkit is infinite, we do not want to block (as it will be forever).
    visualkit->source().stop(false);

    return 0;
}