#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "depthai_ros_msgs/TrackedFeatures.h"
#include "ros/ros.h"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/FeatureTracker.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/TrackedFeaturesConverter.hpp"
#include "depthai_ros_msgs/TrackedFeaturesAndImage.h"

dai::rosBridge::TrackedFeaturesConverter* leftConverter ; 
dai::rosBridge::TrackedFeaturesConverter* rightConverter ; 
dai::rosBridge::ImageConverter* left_img_converter;
dai::rosBridge::ImageConverter* right_img_converter ; 


sensor_msgs::Image left_image,right_image ;
depthai_ros_msgs::TrackedFeaturesAndImage left_data, right_data ;  
std::atomic_bool left_img_available(false) , right_img_available(false) ; 
std::mutex left_img_mtx ,right_img_mtx ; 
ros::Publisher left_data_pub , right_data_pub ;  

void left_tracking_callback(std::shared_ptr<dai::ADatatype> data)
{
    // std::cout<<"Entered left_tracking_callback \n" ; 
    auto tracked_points = std::dynamic_pointer_cast<dai::TrackedFeatures>(data) ;
    std::deque<depthai_ros_msgs::TrackedFeatures> featureMsgs ;
    leftConverter->toRosMsg(tracked_points,featureMsgs) ;
    if(featureMsgs.size()>0)
        if(left_img_available)
        {
            left_data.features = featureMsgs.at(0).features ; 
            left_data.image = left_image ; 
            left_data.header = featureMsgs.at(0).header ; 
            left_data_pub.publish(left_data) ; 
            left_img_available = false ; 
            left_img_mtx.unlock() ; 
        }
}

void left_img_callback(std::shared_ptr<dai::ADatatype> data)
{
    auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data) ;
    // cv::Mat cv_frame = img->getCvFrame() ; 
    // auto time = img->getTimestamp() ; 
    std::deque<sensor_msgs::Image> image_msgs ;
    left_img_converter->toRosMsg(img,image_msgs) ; 
    if(image_msgs.size()>0)
        left_image = image_msgs.at(0) ; 
        left_img_available = true ; 
}

void right_img_callback(std::shared_ptr<dai::ADatatype> data)
{
    auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data) ;
    // cv::Mat cv_frame = img->getCvFrame() ; 
    // auto time = img->getTimestamp() ; 
    std::deque<sensor_msgs::Image> image_msgs ;
    left_img_converter->toRosMsg(img,image_msgs) ; 
    if(image_msgs.size()>0)
        right_image = image_msgs.at(0) ; 
        right_img_available = true ; 
}

void right_tracking_callback(std::shared_ptr<dai::ADatatype> data)
{
    // std::cout<<"Entered right_tracking_callback \n" ; 
    auto tracked_points = std::dynamic_pointer_cast<dai::TrackedFeatures>(data) ;
    // std::cout<<"Found "<< tracked_points->trackedFeatures.size() <<" points \n" ; 
    std::deque<depthai_ros_msgs::TrackedFeatures> featureMsgs ;
    rightConverter->toRosMsg(tracked_points,featureMsgs) ;
    if(featureMsgs.size()>0)
        if(right_img_available)
        {
            right_data.features = featureMsgs.at(0).features ; 
            right_data.image = right_image ; 
            right_data.header = featureMsgs.at(0).header ;
            right_data_pub.publish(right_data) ;
            right_img_available = false ;  
        }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "feature_tracker_node");
    ros::NodeHandle pnh("~");

    dai::Pipeline pipeline;
    pipeline.setXLinkChunkSize(0) ;
    left_data_pub = pnh.advertise<depthai_ros_msgs::TrackedFeaturesAndImage>("featureAndImageLeft",2) ;
    right_data_pub = pnh.advertise<depthai_ros_msgs::TrackedFeaturesAndImage>("featureAndImageRight",2) ;  

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto featureTrackerLeft = pipeline.create<dai::node::FeatureTracker>();
    auto featureTrackerRight = pipeline.create<dai::node::FeatureTracker>();
    auto imu = pipeline.create<dai::node::IMU>();
    

    auto xoutTrackedFeaturesLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutImageLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutTrackedFeaturesRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutImageRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();

    xoutTrackedFeaturesLeft->setStreamName("trackedFeaturesLeft");
    xoutImageLeft->setStreamName("imageLeft");
    xoutTrackedFeaturesRight->setStreamName("trackedFeaturesRight");
    xoutImageRight->setStreamName("imageRight");
    xoutImu->setStreamName("imu");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setCamera("right");
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(5);
    imu->setMaxBatchReports(20);

    // Linking
    monoLeft->out.link(featureTrackerLeft->inputImage);
    featureTrackerLeft->outputFeatures.link(xoutTrackedFeaturesLeft->input);
    featureTrackerLeft->passthroughInputImage.link(xoutImageLeft->input) ; 
    monoRight->out.link(featureTrackerRight->inputImage);
    featureTrackerRight->outputFeatures.link(xoutTrackedFeaturesRight->input);
    featureTrackerRight->passthroughInputImage.link(xoutImageRight->input) ; 
    imu->out.link(xoutImu->input);

    // By default the least mount of resources are allocated
    // increasing it improves performance when optical flow is enabled
    auto numShaves = 2;
    auto numMemorySlices = 2;
    featureTrackerLeft->setHardwareResources(numShaves, numMemorySlices);
    featureTrackerRight->setHardwareResources(numShaves, numMemorySlices);

    auto featureTrackerConfig = featureTrackerRight->initialConfig.get();
    featureTrackerConfig.motionEstimator.type = dai::FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW ; // Use Lucas Kanade Opt flow
    featureTrackerConfig.featureMaintainer.enable = true ;
    featureTrackerConfig.featureMaintainer.lostFeatureErrorThreshold = 10.0 ;
    featureTrackerConfig.featureMaintainer.minimumDistanceBetweenFeatures = 20.0 ; 
    featureTrackerConfig.featureMaintainer.trackedFeatureThreshold = 25.0 ; 
    featureTrackerConfig.cornerDetector.cellGridDimension = 4 ; 
    featureTrackerConfig.cornerDetector.enableSobel = true ; 
    featureTrackerConfig.cornerDetector.enableSorting = true ; 
    featureTrackerConfig.cornerDetector.numMaxFeatures = 320 ; 
    featureTrackerConfig.cornerDetector.type = dai::RawFeatureTrackerConfig::CornerDetector::Type::SHI_THOMASI ; 
    featureTrackerRight->initialConfig.set(featureTrackerConfig) ; 
    featureTrackerLeft->initialConfig.set(featureTrackerConfig) ; 
    std::shared_ptr<dai::Device> device;
    std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();

    std::cout << "Listing available devices..." << std::endl;
    device = std::make_shared<dai::Device>(pipeline);
    auto outputFeaturesLeftQueue = device->getOutputQueue("trackedFeaturesLeft", 2, false);
    auto outputFeaturesRightQueue = device->getOutputQueue("trackedFeaturesRight", 2, false);
    auto imuQueue = device->getOutputQueue("imu",30,false) ; 
    auto leftQueue = device->getOutputQueue("imageLeft", 2, false);
    auto rightQueue = device->getOutputQueue("imageRight", 2, false);
    auto calibrationHandler = device->readCalibration();
    std::string tfPrefix = "oak";
    
    dai::rosBridge::ImuConverter imuConverter(tfPrefix + "_imu_frame", dai::ros::ImuSyncMethod::COPY, 0.0, 0.0);
    leftConverter = new dai::rosBridge::TrackedFeaturesConverter(tfPrefix + "_left_camera_optical_frame", false);
    rightConverter = new dai::rosBridge::TrackedFeaturesConverter(tfPrefix + "_right_camera_optical_frame", false);
    left_img_converter = new dai::rosBridge::ImageConverter(tfPrefix + "_left_camera_optical_frame", false);
    right_img_converter = new dai::rosBridge::ImageConverter(tfPrefix + "_right_camera_optical_frame", false);
    leftConverter->updateRosBaseTime() ; 
    rightConverter->updateRosBaseTime() ; 
    left_img_converter->updateRosBaseTime() ; 
    right_img_converter->updateRosBaseTime() ;
    
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::TrackedFeatures, dai::TrackedFeatures> featuresPubL(
        outputFeaturesLeftQueue,
        pnh,
        std::string("features_left"),
        std::bind(&dai::rosBridge::TrackedFeaturesConverter::toRosMsg, leftConverter, std::placeholders::_1, std::placeholders::_2),
        2,
        "",
        "features_left");

    // featuresPubL.addPublisherCallback();

    dai::rosBridge::BridgePublisher<depthai_ros_msgs::TrackedFeatures, dai::TrackedFeatures> featuresPubR(
        outputFeaturesRightQueue,
        pnh,
        std::string("features_right"),
        std::bind(&dai::rosBridge::TrackedFeaturesConverter::toRosMsg, rightConverter, std::placeholders::_1, std::placeholders::_2),
        2,
        "",
        "features_right");

    // featuresPubR.addPublisherCallback();
    
    dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> imuPublish(
        imuQueue,
        pnh,
        std::string("imu"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        2,
        "",
        "imu");

    imuPublish.addPublisherCallback();
    auto leftCameraInfo = left_img_converter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoLeft->getResolutionWidth(), monoLeft->getResolutionHeight());
    auto rightCameraInfo = right_img_converter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoRight->getResolutionWidth(), monoRight->getResolutionHeight());
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(
        leftQueue,
        pnh,
        "left/image_raw",
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, left_img_converter, std::placeholders::_1, std::placeholders::_2),
        2,
        leftCameraInfo,
        "left");
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(
        rightQueue,
        pnh,
        "right/image_raw",
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, right_img_converter, std::placeholders::_1, std::placeholders::_2),
        2,
        rightCameraInfo,
        "right");
    // rightPublish.addPublisherCallback();
    // leftPublish.addPublisherCallback();
    leftQueue->addCallback(left_img_callback) ; 
    rightQueue->addCallback(right_img_callback) ; 
    outputFeaturesLeftQueue->addCallback(left_tracking_callback) ; 
    outputFeaturesRightQueue->addCallback(right_tracking_callback) ; 
    std::cout << "Ready." << std::endl;
    ros::spin() ;   
    return 0;
}
