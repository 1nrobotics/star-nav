#include <memory>
#include <iostream>
#include <vk_sdk/Sdk.hpp>
#include <vk_sdk/capnp/image.capnp.h>
#include <cmath>
#include <array>
#include <string>
#include <cstring>

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

//Disparity
class MyReceiver_Disparity: public vkc::Receiver<vkc::Disparity> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Disparity>>& message) override {
        auto Disparity = message.payload.reader();
        printHeader(Disparity.getHeader());
        std::cout << "getEncoding: ";
        switch (Disparity.getEncoding()) {
            case vkc::Disparity::Encoding::DISPARITY8 :
                std::cout << "DISPARITY8" << std::endl;
                break;
            case vkc::Disparity::Encoding::DISPARITY16:
                std::cout << "DISPARITY16" << std::endl;
                break;
            case vkc::Disparity::Encoding::DEPTH16:
                std::cout << "DEPTH16" << std::endl;
                break;
            case vkc::Disparity::Encoding::DEPTH32:
                std::cout << "DEPTH32" << std::endl;
                break;
            case vkc::Disparity::Encoding::DISTANCE16:
                std::cout << "DISTANCE16" << std::endl;
                break;
            case vkc::Disparity::Encoding::DISTANCE32:
                std::cout << "DISTANCE32" << std::endl;
                break;
            case vkc::Disparity::Encoding::CONFIDENCE8:
                std::cout << "CONFIDENCE8" << std::endl;
                break;
            default:
                std::cout << "unknown" << std::endl;
        }
        std::cout<<"getWidth: "<<Disparity.getWidth()<<std::endl;
        std::cout<<"getHeight: "<<Disparity.getHeight()<<std::endl;
        std::cout<<"getStep: "<<Disparity.getStep()<<std::endl;
        std::cout<<"getDecimationFactor: "<<Disparity.getDecimationFactor()<<std::endl;
        std::cout<<"getData: "<<Disparity.getData().size()<<std::endl;
        std::cout<<"getFx: "<<Disparity.getFx()<<std::endl;
        std::cout<<"getFy: "<<Disparity.getFy()<<std::endl;
        std::cout<<"getCx: "<<Disparity.getCx()<<std::endl;
        std::cout<<"getCy: "<<Disparity.getCy()<<std::endl;
        std::cout<<"getBaseline: "<<Disparity.getBaseline()<<std::endl;
        std::cout<<"getStreamName: "<<Disparity.getStreamName().cStr()<<std::endl;
        printSo3(Disparity.getPinholeRotation());
        std::cout<<"getMaxDisparity: "<<Disparity.getMaxDisparity()<<std::endl;
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
    auto myReceiver = std::make_unique<MyReceiver_Disparity>();

    // Install the receiver into the data source so that the receiver can receive messages.
    visualkit->source().install("S1/stereo1_l/disparity", std::move(myReceiver));
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