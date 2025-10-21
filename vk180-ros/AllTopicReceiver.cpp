#include <memory>
#include <iostream>
#include <vk_sdk/Sdk.hpp>
#include <vk_sdk/capnp/image.capnp.h>
#include <cmath>
#include <array>
#include <string>
#include <cstring>

//print Vector2d
void printVector2d(const vkc::Vector2d::Reader& vector2d) {
    std::cout << "printVector2d:" << std::endl;
    std::cout << "2dX: " << vector2d.getX() << ", 2dY: " << vector2d.getY() << std::endl;
}
//print Vector2f
void printVector2f(const vkc::Vector2f::Reader& vector2f) {
    std::cout << "printVector2f:" << std::endl;
    std::cout << "2fX: " << vector2f.getX() << ", 2fY: " << vector2f.getY() << std::endl;
}
//print Vector3d
void printVector3d(const vkc::Vector3d::Reader& vector3d) {
    std::cout << "printVector3d:" << std::endl;
    std::cout << "3dX: " << vector3d.getX() << ", 3dY: " << vector3d.getY() << ", 3dZ: " << vector3d.getZ() << std::endl;
}
//print Vector3f
void printVector3f(const vkc::Vector3f::Reader& vector3d) {
    std::cout << "printVector3f:" << std::endl;
    std::cout << "3fX: " << vector3d.getX() << ", 3fY: " << vector3d.getY() << ", 3fZ: " << vector3d.getZ() << std::endl;
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
//print MotionMeta
void printMotionMeta(const vkc::MotionMeta::Reader& motionMeta) {
    std::cout << "printMotionMeta: " << std::endl;
    std::cout << "getInstantaneousAngularVelocity: ";
    printVector3d(motionMeta.getInstantaneousAngularVelocity());
    std::cout << "getAverageAngularVelocity: ";
    printVector3d(motionMeta.getAverageAngularVelocity());
    std::cout << "getShockLikelihood: " << motionMeta.getShockLikelihood() << std::endl;
    std::cout << "getDeltaUs: " << motionMeta.getDeltaUs() << std::endl;
}
//print CameraControl
void printCameraControl(const vkc::CameraControl::Reader& CameraControl) {
    std::cout << "printCameraControl: " << std::endl;
    std::cout << "getHeader: ";
    printHeader(CameraControl.getHeader());
    std::cout << "getStreaming: " << CameraControl.getStreaming() << std::endl;
    std::cout << "getExposureUSec: " << CameraControl.getExposureUSec() << std::endl;
    std::cout << "getGain: " << CameraControl.getGain() << std::endl;
    std::cout << "getExposureCompensation: " << CameraControl.getExposureCompensation() << std::endl;
    std::cout << "getSensorIdx: " << CameraControl.getSensorIdx() << std::endl;
    std::cout << "getYoloRequest: " << CameraControl.getYoloRequest()<< std::endl;
}
//print CameraIntrinsic
void printCameraIntrinsic(const vkc::CameraIntrinsic::Reader& CameraIntrinsic) {
    std::cout << "printCameraIntrinsic: " << std::endl;
    std::cout<<"getCx: "<<CameraIntrinsic.getPinhole().getCx()<<std::endl;
    std::cout<<"getCy: "<<CameraIntrinsic.getPinhole().getCy()<<std::endl;
    std::cout<<"getFx: "<<CameraIntrinsic.getPinhole().getFx()<<std::endl;
    std::cout<<"getFy: "<<CameraIntrinsic.getPinhole().getFy()<<std::endl;
    std::cout<<"getDs.getCx: "<<CameraIntrinsic.getDs().getPinhole().getCx()<<std::endl;
    std::cout<<"getDs.getCy: "<<CameraIntrinsic.getDs().getPinhole().getCy()<<std::endl;
    std::cout<<"getDs.getFx: "<<CameraIntrinsic.getDs().getPinhole().getFx()<<std::endl;
    std::cout<<"getDs.getFy: "<<CameraIntrinsic.getDs().getPinhole().getFy()<<std::endl;
    std::cout<<"getDs.getXi: "<<CameraIntrinsic.getDs().getXi()<<std::endl;
    std::cout<<"getDs.getAlpha :"<<CameraIntrinsic.getDs().getAlpha()<<std::endl;
    std::cout<<"getKb4.getCx: "<<CameraIntrinsic.getKb4().getPinhole().getCx()<<std::endl;
    std::cout<<"getKb4.getCy: "<<CameraIntrinsic.getKb4().getPinhole().getCy()<<std::endl;
    std::cout<<"getKb4.getFx: "<<CameraIntrinsic.getKb4().getPinhole().getFx()<<std::endl;
    std::cout<<"getKb4.getFy: "<<CameraIntrinsic.getKb4().getPinhole().getFy()<<std::endl;
    std::cout<<"getKb4.getK1: "<<CameraIntrinsic.getKb4().getK1()<<std::endl;
    std::cout<<"getKb4.getK2: "<<CameraIntrinsic.getKb4().getK2()<<std::endl;
    std::cout<<"getKb4.getK3: "<<CameraIntrinsic.getKb4().getK3()<<std::endl;
    std::cout<<"getKb4.getK4: "<<CameraIntrinsic.getKb4().getK4()<<std::endl;
    std::cout<<"getRadtan8.getCx: "<<CameraIntrinsic.getRadtan8().getPinhole().getCx()<<std::endl;
    std::cout<<"getRadtan8.getCy: "<<CameraIntrinsic.getRadtan8().getPinhole().getCy()<<std::endl;
    std::cout<<"getRadtan8.getFx: "<<CameraIntrinsic.getRadtan8().getPinhole().getFx()<<std::endl;
    std::cout<<"getRadtan8.getFy: "<<CameraIntrinsic.getRadtan8().getPinhole().getFy()<<std::endl;
    std::cout<<"getRadtan8.getK1: "<<CameraIntrinsic.getRadtan8().getK1()<<std::endl;
    std::cout<<"getRadtan8.getK2: "<<CameraIntrinsic.getRadtan8().getK2()<<std::endl;
    std::cout<<"getRadtan8.getK3: "<<CameraIntrinsic.getRadtan8().getK3()<<std::endl;
    std::cout<<"getRadtan8.getK4: "<<CameraIntrinsic.getRadtan8().getK4()<<std::endl;
    std::cout<<"getRadtan8.getK5: "<<CameraIntrinsic.getRadtan8().getK5()<<std::endl;
    std::cout<<"getRadtan8.getK6: "<<CameraIntrinsic.getRadtan8().getK6()<<std::endl;
    std::cout<<"getRectified: "<<CameraIntrinsic.getRectified()<<std::endl;
    std::cout<<"getLastModified: "<<CameraIntrinsic.getLastModified()<<std::endl;
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
//print SensorExtrinsic
void printSensorExtrinsic(const vkc::SensorExtrinsic::Reader& SensorExtrinsic) {
    std::cout << "printSensorExtrinsic: " << std::endl;
    std::cout << "getBodyFrame: ";
    printSe3(SensorExtrinsic.getBodyFrame());
    std::cout << "getImuFrame: ";
    printSe3(SensorExtrinsic.getImuFrame());
    std::cout<<"getLastModified: "<<SensorExtrinsic.getLastModified()<<std::endl;
}
//print Image
void printImage(const vkc::Image::Reader& image) {
    std::cout << "printMotionMeta: " << std::endl;
    std::cout << "getHeader: ";
    printHeader(image.getHeader());
    std::cout << "getEncoding: ";
    switch (image.getEncoding()) {
        case vkc::Image::Encoding::MONO8:
            std::cout << "mono8" << std::endl;
            break;
        case vkc::Image::Encoding::MONO16:
            std::cout << "mono16" << std::endl;
            break;
        case vkc::Image::Encoding::YUV420:
            std::cout << "yuv420" << std::endl;
            break;
        case vkc::Image::Encoding::BGR8:
            std::cout << "bgr8" << std::endl;
            break;
        case vkc::Image::Encoding::JPEG:
            std::cout << "jpeg" << std::endl;
            break;
        case vkc::Image::Encoding::PNG:
            std::cout << "png" << std::endl;
            break;
        case vkc::Image::Encoding::H264:
            std::cout << "h264" << std::endl;
            break;
        case vkc::Image::Encoding::H265:
            std::cout << "h265" << std::endl;
            break;
        default:
            std::cout << "unknown" << std::endl;
    }
    std::cout << "getWidth: " << image.getWidth() << std::endl;
    std::cout << "getHeight: " << image.getHeight() << std::endl;
    std::cout << "getStep: " << image.getStep() << std::endl;
    std::cout << "getData: " << image.getData().size() << " bytes" << std::endl;
    std::cout << "getExposureUSec: " << image.getExposureUSec() << std::endl;
    std::cout << "getGain: " << image.getGain() << std::endl;
    std::cout << "getSensorIdx: " << static_cast<int>(image.getSensorIdx()) << std::endl;
    std::cout << "getStreamName: " << image.getStreamName().cStr() << std::endl;
    std::cout << "getIntrinsic: ";
    printCameraIntrinsic(image.getIntrinsic());
    std::cout << "getExtrinsic: ";
    printSensorExtrinsic(image.getExtrinsic());
    std::cout << "getMipMapLevels: " << static_cast<int>(image.getMipMapLevels()) << std::endl;
    std::cout << "getMipMapBrightness: " << static_cast<int>(image.getMipMapBrightness()) << std::endl;
    std::cout << "getMipMapBrightnessChange: " << image.getMipMapBrightnessChange() << std::endl;
    std::cout << "getMotionMeta: ";
    printMotionMeta(image.getMotionMeta());
}
//print Twist3
void printTwist3(const vkc::Twist3::Reader& Twist3){
    std::cout << "printTwist3: " << std::endl;
    std::cout << "getLinear: ";
    printVector3d(Twist3.getLinear());
    std::cout << "getAngular: ";
    printVector3d(Twist3.getAngular()); 
}
// print TagFamily 枚举值
std::string tagFamilyToString(const vkc::TagFamily family) {
    switch (family) {
        case vkc::TagFamily::TAG16H5: return "tag16h5";
        case vkc::TagFamily::TAG25H7: return "tag25h7";
        case vkc::TagFamily::TAG25H9: return "tag25h9";
        case vkc::TagFamily::TAG36H10: return "tag36h10";
        case vkc::TagFamily::TAG36H11: return "tag36h11";
        case vkc::TagFamily::TAG16H5_BORDER2 : return "tag16h5Border2";
        case vkc::TagFamily::TAG25H7_BORDER2: return "tag25h7Border2";
        case vkc::TagFamily::TAG25H9_BORDER2: return "tag25h9Border2";
        case vkc::TagFamily::TAG36H10_BORDER2: return "tag36h10Border2";
        case vkc::TagFamily::TAG36H11_BORDER2: return "tag36h11Border2";
        default: return "unknown";
    }
}
// print AprilGrid 结构体内容
void printAprilGrid(const vkc::AprilGrid::Reader& AprilGrid) {
    std::cout << "printAprilGrid: " << std::endl;
    std::cout << "getStartId: " << AprilGrid.getStartId() << std::endl;
    std::cout << "getIncrement: " << AprilGrid.getIncrement() << std::endl;
    std::cout << "getFamily: " << tagFamilyToString(AprilGrid.getFamily()) << std::endl;
    std::cout << "getTagCols: " << static_cast<int>(AprilGrid.getTagCols()) << std::endl;
    std::cout << "getTagRows: " << static_cast<int>(AprilGrid.getTagRows()) << std::endl;
    std::cout << "getTagSpacing: " << AprilGrid.getTagSpacing() << std::endl;
    std::cout << "getTagSize: " << AprilGrid.getTagSize() << std::endl;
    std::cout << "getGridId: " << AprilGrid.getGridId() << std::endl;
}
// print TagDetection
void printTagDetection(const vkc::TagDetection::Reader& TagDetection) {
    std::cout << "printTagDetection:" << std::endl;
    std::cout << "getId:" << TagDetection.getId() << std::endl;
    std::cout << "getFamily:" << tagFamilyToString(TagDetection.getFamily()) << std::endl;
    std::cout << "getHammingDistance:" << static_cast<int>(TagDetection.getHammingDistance()) << std::endl;
    std::cout << "getPointsPolygon:";
    for (auto point : TagDetection.getPointsPolygon()) {
        std::cout << point << " ";
    }
    std::cout << std::endl;
    std::cout << "getGridId:" << TagDetection.getGridId() << std::endl;
}
//print Flow2d
void printFlow2d(const vkc::Flow2d::Reader& Flow2d){
    std::cout << "printFlow2d: " << std::endl;
    std::cout << "getDetectorMethod: ";
    switch (Flow2d.getDetectorMethod()) {
        case vkc::Flow2d::DetectorMethod::FAST :
            std::cout << "FAST" << std::endl;
            break;
        case vkc::Flow2d::DetectorMethod::SPARSE_STEREO:
            std::cout << "SPARSE_STEREO" << std::endl;
            break;
        default:
            std::cout << "unknown" << std::endl;
    }
    std::cout<<"getId: "<<Flow2d.getId()<<std::endl;
    std::cout << "getPosition: ";
    printVector2f(Flow2d.getPosition());
    std::cout<<"getRadial: "<<Flow2d.getRadial()<<std::endl;
    std::cout<<"getAzimuth: "<<Flow2d.getAzimuth()<<std::endl;
    std::cout<<"getLevel: "<<Flow2d.getLevel()<<std::endl;
    std::cout<<"getAge: "<<Flow2d.getAge()<<std::endl;
    for(const auto &aa:Flow2d.getHistory()){
        std::cout<<"getSeq: "<<aa.getSeq()<<std::endl;
        std::cout << "getHistory getPosition: ";
        printVector2f(aa.getPosition());
    }
}
//print Imu
void printImu(const vkc::Imu::Reader & Imu){
    std::cout << "printImu: " << std::endl;
    std::cout << "getHeader: ";
    printHeader(Imu.getHeader());
    std::cout << "getLinearAcceleration: ";
    printVector3d(Imu.getLinearAcceleration());
    std::cout << "getAngularVelocity: ";
    printVector3d(Imu.getAngularVelocity());

    std::cout<<"getIntrinsic:"<<std::endl;
    std::cout << "getGyroNoiseStd: ";
    printVector3d(Imu.getIntrinsic().getGyroNoiseStd());
    std::cout << "getAccelNoiseStd: ";
    printVector3d(Imu.getIntrinsic().getAccelNoiseStd());
    std::cout << "getGyroBiasStd: ";
    printVector3d(Imu.getIntrinsic().getGyroBiasStd());
    std::cout << "getHeader: ";
    printVector3d(Imu.getIntrinsic().getAccelBiasStd());
    std::cout<<"getUpdateRate:"<<Imu.getIntrinsic().getUpdateRate()<<std::endl;
    std::cout<<"getTimeOffsetNs:"<<Imu.getIntrinsic().getTimeOffsetNs()<<std::endl;
    std::cout<<"getLastModified:"<<Imu.getIntrinsic().getLastModified()<<std::endl;

    std::cout << "getExtrinsic: ";
    printSensorExtrinsic(Imu.getExtrinsic());
    std::cout<<"getSeqIncrement:"<<Imu.getSeqIncrement()<<std::endl;
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
//print Field
void printField(const vkc::Field::Reader &Field){
    std::cout << "printField: " << std::endl;
    std::cout<<"getName: "<<Field.getName().cStr()<<std::endl;
    std::cout<<"getOffset: "<<Field.getOffset()<<std::endl;
    std::cout << "getType: ";
    switch (Field.getType()) {
        case vkc::Field::NumericType::FLOAT32 :
            std::cout << "FLOAT32" << std::endl;
            break;
        case vkc::Field::NumericType::FLOAT64:
            std::cout << "FLOAT64" << std::endl;
            break;
        case vkc::Field::NumericType::INT8:
            std::cout << "INT8" << std::endl;
            break;
        case vkc::Field::NumericType::INT16:
            std::cout << "INT16" << std::endl;
            break;
        case vkc::Field::NumericType::INT32:
            std::cout << "INT32" << std::endl;
            break;
        case vkc::Field::NumericType::INT64:
            std::cout << "INT64" << std::endl;
            break;
        case vkc::Field::NumericType::UINT8:
            std::cout << "UINT8" << std::endl;
            break;
        case vkc::Field::NumericType::UINT16:
            std::cout << "UINT16" << std::endl;
            break;
         case vkc::Field::NumericType::UINT32:
            std::cout << "UINT32" << std::endl;
            break;
        case vkc::Field::NumericType::UINT64:
            std::cout << "UINT64" << std::endl;
            break;
        default:
            std::cout << "unknown" << std::endl;
    }
}
//print VioPose
void printVioPose(const vkc::VioPose::Reader &VioPose){
    std::cout << "printVioPose: " << std::endl;
    std::cout << "getStamp: " <<VioPose.getStamp()<<std::endl;
    std::cout << "getPose: ";
    printSe3(VioPose.getPose());
    std::cout << "getKf: " <<VioPose.getKf()<<std::endl;
    std::cout << "getGroup: " <<VioPose.getGroup()<<std::endl;
}
//print VioLandmark
void printVioLandmark(const vkc::VioLandmark::Reader &VioLandmark){
    std::cout << "printVioLandmark: " << std::endl;
    std::cout << "getId: " <<VioLandmark.getId()<<std::endl;
    std::cout << "getHostedKfStamp: " <<VioLandmark.getHostedKfStamp()<<std::endl;
    for(const auto& aa: VioLandmark.getObservedStamps()){
        std::cout << "getObservedStamps: " <<aa<<std::endl;
    }
    std::cout << "getInvDist: " <<VioLandmark.getInvDist()<<std::endl;
    std::cout << "getPoint3d: ";
    printVector3f(VioLandmark.getPoint3d());
}



// Our custom receiver to receiver messages.
// ManagerMessage
class MyReceiver_ManagerMessage: public vkc::Receiver<vkc::ManagerMessage> {
    // Override this method to handle messages.

    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::ManagerMessage>>& message) override {
        auto ManagerMessage = message.payload.reader();
        std::cout<<""<<ManagerMessage.getMarker()
                <<""<<ManagerMessage.getVariant().getCameraDriver().getTarget()<<std::endl;
        return vkc::ReceiverStatus::Open;
    }
};
//CameraControl
class MyReceiver_CameraControl : public vkc::Receiver<vkc::CameraControl>{
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::CameraControl>>& message) override{
        auto CameraControl = message.payload.reader();
        printCameraControl(CameraControl);
        return vkc::ReceiverStatus::Open;
    }
};
//CameraIntrinsic
class MyReceiver_CameraIntrinsic : public vkc::Receiver<vkc::CameraIntrinsic>{
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::CameraIntrinsic>>& message) override{
        auto CameraIntrinsic = message.payload.reader();
        printCameraIntrinsic(CameraIntrinsic);
        return vkc::ReceiverStatus::Open;
    }
};
//Detections2d
class MyReceiver_Detections2d: public vkc::Receiver<vkc::Detections2d> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Detections2d>>& message) override {
        auto Detections2d = message.payload.reader();
        printHeader(Detections2d.getHeader());
        printImage(Detections2d.getImage());
        for(const auto &aa : Detections2d.getDetections()){
            std::cout<<"getDetections.getConfidence: "<<aa.getConfidence()<<std::endl;
            std::cout<<"getDetections.getLabelIdx: "<<aa.getLabelIdx()<<std::endl;
            std::cout<<"getDetections.getXmax: "<<aa.getXmax()<<std::endl;
            std::cout<<"getDetections.getXmin: "<<aa.getXmin()<<std::endl;
            std::cout<<"getDetections.getYmax: "<<aa.getYmax()<<std::endl;
            std::cout<<"getDetections.getYmin: "<<aa.getYmin()<<std::endl;
        }
        for(const auto &aa : Detections2d.getLabels()){
            std::cout<<"getLabels: "<<aa.cStr()<<std::endl;
        }
        return vkc::ReceiverStatus::Open;
    }
};
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
//Flow2d
class MyReceiver_Flow2d: public vkc::Receiver<vkc::Flow2d> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Flow2d>>& message) override {
        auto Flow2d = message.payload.reader();
        printFlow2d(Flow2d);
        return vkc::ReceiverStatus::Open;
    }
};
//Flow2d HFOpticalFlowResult
class MyReceiver_Flow2d_HFOpticalFlowResult: public vkc::Receiver<vkc::HFOpticalFlowResult> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::HFOpticalFlowResult>>& message) override {
        auto HFOpticalFlowResult = message.payload.reader();
        printHeader(HFOpticalFlowResult.getHeader());
        std::cout<<"getStreamName: "<<HFOpticalFlowResult.getStreamName().cStr()<<std::endl;
        for(const auto &aa : HFOpticalFlowResult.getMeanFlow()){
            std::cout<<"getMeanFlow: "<<aa<<std::endl;
        }
        for(const auto &aa : HFOpticalFlowResult.getFlowDensity()){
            std::cout<<"getFlowDensity: "<<aa<<std::endl;
        }
        for(const auto &aa : HFOpticalFlowResult.getFlowData()){
            std::cout<<"getFlowData: "<<std::endl;
            printFlow2d(aa);
        }
        printImage(HFOpticalFlowResult.getImage());

        return vkc::ReceiverStatus::Open;
    }
};
//Flow2d HFSparseStereoResult
class MyReceiver_Flow2d_HFSparseStereoResult: public vkc::Receiver<vkc::HFSparseStereoResult> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::HFSparseStereoResult>>& message) override {
        auto HFSparseStereoResult = message.payload.reader();
        printHeader(HFSparseStereoResult.getHeader());
        std::cout<<"getStreamName: "<<HFSparseStereoResult.getStreamName().cStr()<<std::endl;
        for(const auto &aa : HFSparseStereoResult.getFiducialCorners()){
            std::cout<<"getFiducialCorners: "<<std::endl;
            printFlow2d(aa);
        }
        for(const auto &aa : HFSparseStereoResult.getMatchedCorners()){
            std::cout<<"getMatchedCorners: "<<std::endl;
            printFlow2d(aa);
        }
        for(const auto &aa : HFSparseStereoResult.getFilteredCorners()){
            std::cout<<"getFilteredCorners: "<<aa<<std::endl;
        }
        return vkc::ReceiverStatus::Open;
    }
};
//GPS
class MyReceiver_GPS: public vkc::Receiver<vkc::GPS> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::GPS>>& message) override {
        auto GPS = message.payload.reader();
        printHeader(GPS.getHeader());
        std::cout<<"getLat: "<<GPS.getLat()<<std::endl;
        std::cout<<"getLong: "<<GPS.getLong()<<std::endl;
        std::cout<<"getAlt: "<<GPS.getAlt()<<std::endl;
        std::cout<<"getSpeed2d: "<<GPS.getSpeed2d()<<std::endl;
        std::cout<<"getSpeed3d: "<<GPS.getSpeed3d()<<std::endl;
        std::cout<<"getDop: "<<GPS.getDop()<<std::endl;
        return vkc::ReceiverStatus::Open;
    }
};
//Header
class MyReceiver_Header: public vkc::Receiver<vkc::Header> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Header>>& message) override {
        auto Header = message.payload.reader();
        printHeader(Header);
        return vkc::ReceiverStatus::Open;
    }
};
//Image
class MyReceiver_Image: public vkc::Receiver<vkc::Image> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Image>>& message) override {
        auto Image = message.payload.reader();
        printImage(Image);
        return vkc::ReceiverStatus::Open;
    }
};
//Imu
class MyReceiver_Imu: public vkc::Receiver<vkc::Imu> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Imu>>& message) override {
        auto Imu = message.payload.reader();
        printImu(Imu);
        return vkc::ReceiverStatus::Open;
    }
};
//ImuList
class MyReceiver_ImuList: public vkc::Receiver<vkc::ImuList> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::ImuList>>& message) override {
        auto ImuList = message.payload.reader();
        for(const auto & aa : ImuList.getList()){
            std::cout<<"getList: "<<std::endl;
            printImu(aa);
        }
        return vkc::ReceiverStatus::Open;
    }
};
//MavState
// class MyReceiver_MavState: public vkc::Receiver<vkc::MavState> {
//     vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::MavState>>& message) override {
//         auto MavState = message.payload.reader();
        
//         return vkc::ReceiverStatus::Open;
//     }
// };

//Odometry3d
class MyReceiver_Odometry3d: public vkc::Receiver<vkc::Odometry3d> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Odometry3d>>& message) override {
        auto Odometry3d = message.payload.reader();
        printOdometry3d(Odometry3d);
        return vkc::ReceiverStatus::Open;
    }
};
//Odometry3d VioState
class MyReceiver_Odometry3d_VioState: public vkc::Receiver<vkc::VioState> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::VioState>>& message) override {
        auto VioState = message.payload.reader();
        printHeader(VioState.getHeader());
        for(const auto &aa :VioState.getStates()){
            std::cout<<"getStates: "<<std::endl;
            printVioPose(aa);

        }
        for(const auto &aa :VioState.getFrames()){
            std::cout<<"getFrames: "<<std::endl;
            printVioPose(aa);

        }
        for(const auto &aa :VioState.getLandmarks()){
            std::cout<<"getLandmarks: "<<std::endl;
            printVioLandmark(aa);
        }
        std::cout<<"getBiasGyro: "<<std::endl;
        printVector3f(VioState.getBiasGyro());
        std::cout<<"getBiasAccel: "<<std::endl;
        printVector3f(VioState.getBiasAccel());

        return vkc::ReceiverStatus::Open;
    }
};
//PointCloud
class MyReceiver_PointCloud: public vkc::Receiver<vkc::PointCloud> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::PointCloud>>& message) override {
        auto PointCloud = message.payload.reader();
        printHeader(PointCloud.getHeader());
        printOdometry3d(PointCloud.getPose());
        std::cout<<"getPointStride: "<<PointCloud.getPointStride()<<std::endl;
        for(const auto & aa: PointCloud.getFields()){
            std::cout<<"getFields: "<<std::endl;
            printField(aa);
        }
        std::cout<<"getPoints: "<<PointCloud.getPoints().size()<<std::endl;
        return vkc::ReceiverStatus::Open;
    }
};
//Se3
class MyReceiver_Se3: public vkc::Receiver<vkc::Se3> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Se3>>& message) override {
        auto Se3 = message.payload.reader();
        printSe3(Se3);
        return vkc::ReceiverStatus::Open;
    }
};
//SensorExtrinsic
class MyReceiver_SensorExtrinsic: public vkc::Receiver<vkc::SensorExtrinsic> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::SensorExtrinsic>>& message) override {
        auto SensorExtrinsic = message.payload.reader();
        printSensorExtrinsic(SensorExtrinsic);
        return vkc::ReceiverStatus::Open;
    }
};
//So3
class MyReceiver_So3: public vkc::Receiver<vkc::So3> {
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::So3>>& message) override {
        auto So3 = message.payload.reader();
        printSo3(So3);
        return vkc::ReceiverStatus::Open;
    }
};
//system


//TagDetections
class MyReceiver_TagDetections: public vkc::Receiver<vkc::TagDetections>{
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::TagDetections>>& message) override {
        auto TagDetections = message.payload.reader();
        printHeader(TagDetections.getHeader());
        printSensorExtrinsic(TagDetections.getCameraExtrinsic());
        printImage(TagDetections.getImage());
        for(const auto &aa : TagDetections.getTags()){
            std::cout<<"getTags: "<<std::endl;
            printTagDetection(aa);
        }
        for(const auto &aa : TagDetections.getGrids()){
            std::cout<<"getGrids: "<<std::endl;
            printAprilGrid(aa);
        }

        return vkc::ReceiverStatus::Open;
    }

};
//Twist3
class MyReceiver_Twist3: public vkc::Receiver<vkc::Twist3>{
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Twist3>>& message) override {
        auto Twist3 = message.payload.reader();
        printTwist3(Twist3);
        return vkc::ReceiverStatus::Open;
    }

};
//Vector2 Vector2f
class MyReceiver_Vector2_Vector2f: public vkc::Receiver<vkc::Vector2f>{
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Vector2f>>& message) override {
        auto Vector2f = message.payload.reader();
        printVector2f(Vector2f);
        return vkc::ReceiverStatus::Open;
    }

};
//Vector2 Vector2d
class MyReceiver_Vector2_Vector2d: public vkc::Receiver<vkc::Vector2d>{
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Vector2d>>& message) override {
        auto Vector2d = message.payload.reader();
        printVector2d(Vector2d);
        return vkc::ReceiverStatus::Open;
    }

};
//Vector3 Vector3f
class MyReceiver_Vector3_Vector3f: public vkc::Receiver<vkc::Vector3f>{
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Vector3f>>& message) override {
        auto Vector3f = message.payload.reader();
        printVector3f(Vector3f);
        return vkc::ReceiverStatus::Open;
    }

};
//Vector3 Vector3d
class MyReceiver_Vector3_Vector3d: public vkc::Receiver<vkc::Vector3d>{
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Vector3d>>& message) override {
        auto Vector3d = message.payload.reader();
        printVector3d(Vector3d);
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
    auto myReceiver = std::make_unique<MyReceiver_Image>();

    // Install the receiver into the data source so that the receiver can receive messages.
    visualkit->source().install("S0/panorama", std::move(myReceiver));
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