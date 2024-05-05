#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <deque>
#include <opencv2/highgui.hpp>
#include <unordered_map>
#include <unordered_set>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <depthai_ros_msgs/TrackedFeatures.h>
#include <depthai_ros_msgs/TrackedFeaturesAndImage.h>

static const auto lineColor = cv::Scalar(200, 0, 200);
static const auto pointColor = cv::Scalar(0, 0, 255);
ros::Subscriber left_img_sub , right_img_sub , left_tracker_sub , right_tracker_sub ; 
cv::Mat left_img ,  right_img ; 
dai::TrackedFeatures features_left , features_right ; 

class FeatureTrackerDrawer {
   private:
    static const int circleRadius = 2;
    static const int maxTrackedFeaturesPathLength = 30;
    // for how many frames the feature is tracked
    static int trackedFeaturesPathLength;

    using featureIdType = decltype(dai::Point2f::x);

    std::unordered_set<featureIdType> trackedIDs;
    std::unordered_map<featureIdType, std::deque<dai::Point2f>> trackedFeaturesPath;


    std::string trackbarName;
    std::string windowName;

   public:
    void trackFeaturePath(std::vector<dai::TrackedFeature>& features) {
        std::unordered_set<featureIdType> newTrackedIDs;
        for(auto& currentFeature : features) {
            auto currentID = currentFeature.id;
            newTrackedIDs.insert(currentID);

            if(!trackedFeaturesPath.count(currentID)) {
                trackedFeaturesPath.insert({currentID, std::deque<dai::Point2f>()});
            }
            std::deque<dai::Point2f>& path = trackedFeaturesPath.at(currentID);

            path.push_back(currentFeature.position);
            while(path.size() > std::max<unsigned int>(1, trackedFeaturesPathLength)) {
                path.pop_front();
            }
        }

        std::unordered_set<featureIdType> featuresToRemove;
        for(auto& oldId : trackedIDs) {
            if(!newTrackedIDs.count(oldId)) {
                featuresToRemove.insert(oldId);
            }
        }

        for(auto& id : featuresToRemove) {
            trackedFeaturesPath.erase(id);
        }

        trackedIDs = newTrackedIDs;
    }

    void trackFeaturePath(const depthai_ros_msgs::TrackedFeaturesAndImage::ConstPtr& features)
    {
        std::unordered_set<featureIdType> newTrackedIDs;
        for(auto& currentFeature : features->features) {
            auto currentID = currentFeature.id;
            newTrackedIDs.insert(currentID);

            if(!trackedFeaturesPath.count(currentID)) {
                trackedFeaturesPath.insert({currentID, std::deque<dai::Point2f>()});
            }
            std::deque<dai::Point2f>& path = trackedFeaturesPath.at(currentID);

            path.push_back(dai::Point2f(currentFeature.position.x,currentFeature.position.y));
            while(path.size() > std::max<unsigned int>(1, trackedFeaturesPathLength)) {
                path.pop_front();
            }
        }

        std::unordered_set<featureIdType> featuresToRemove;
        for(auto& oldId : trackedIDs) {
            if(!newTrackedIDs.count(oldId)) {
                featuresToRemove.insert(oldId);
            }
        }

        for(auto& id : featuresToRemove) {
            trackedFeaturesPath.erase(id);
        }

        trackedIDs = newTrackedIDs;
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(features->image, sensor_msgs::image_encodings::MONO8);
            cv::Mat img = cv_ptr->image.clone() ; 
            drawFeatures(img);
            cv::imshow(features->header.frame_id,img) ;
            cv::waitKey(1) ; 
        } catch (cv_bridge::Exception &e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
    }

    void trackFeaturePath(const depthai_ros_msgs::TrackedFeatures::ConstPtr& features) {
        std::unordered_set<featureIdType> newTrackedIDs;
        for(auto& currentFeature : features->features) {
            auto currentID = currentFeature.id;
            newTrackedIDs.insert(currentID);

            if(!trackedFeaturesPath.count(currentID)) {
                trackedFeaturesPath.insert({currentID, std::deque<dai::Point2f>()});
            }
            std::deque<dai::Point2f>& path = trackedFeaturesPath.at(currentID);

            path.push_back(dai::Point2f(currentFeature.position.x,currentFeature.position.y));
            while(path.size() > std::max<unsigned int>(1, trackedFeaturesPathLength)) {
                path.pop_front();
            }
        }

        std::unordered_set<featureIdType> featuresToRemove;
        for(auto& oldId : trackedIDs) {
            if(!newTrackedIDs.count(oldId)) {
                featuresToRemove.insert(oldId);
            }
        }

        for(auto& id : featuresToRemove) {
            trackedFeaturesPath.erase(id);
        }

        trackedIDs = newTrackedIDs;
    }

    void drawFeatures(cv::Mat& img) {
        cv::setTrackbarPos(trackbarName.c_str(), windowName.c_str(), trackedFeaturesPathLength);

        for(auto& featurePath : trackedFeaturesPath) {
            std::deque<dai::Point2f>& path = featurePath.second;
            unsigned int j = 0;
            for(j = 0; j < path.size() - 1; j++) {
                auto src = cv::Point(path[j].x, path[j].y);
                auto dst = cv::Point(path[j + 1].x, path[j + 1].y);
                cv::line(img, src, dst, lineColor, 1, cv::LINE_AA, 0);
            }

            cv::circle(img, cv::Point(path[j].x, path[j].y), circleRadius, pointColor, -1, cv::LINE_AA, 0);
        }
    }

    cv::Mat drawFeatures(const sensor_msgs::Image::ConstPtr& img_msg) {
        cv_bridge::CvImagePtr  img_ptr =  cv_bridge::toCvCopy(img_msg) ; 
        cv::Mat img = img_ptr->image ; 
        cv::setTrackbarPos(trackbarName.c_str(), windowName.c_str(), trackedFeaturesPathLength);

        for(auto& featurePath : trackedFeaturesPath) {
            std::deque<dai::Point2f>& path = featurePath.second;
            unsigned int j = 0;
            for(j = 0; j < path.size() - 1; j++) {
                auto src = cv::Point(path[j].x, path[j].y);
                auto dst = cv::Point(path[j + 1].x, path[j + 1].y);
                cv::line(img, src, dst, lineColor, 1, cv::LINE_AA, 0);
            }

            cv::circle(img, cv::Point(path[j].x, path[j].y), circleRadius, pointColor, -1, cv::LINE_AA, 0);
        }
        return img ; 
    }


    FeatureTrackerDrawer(std::string trackbarName, std::string windowName) : trackbarName(trackbarName), windowName(windowName) {
        cv::namedWindow(windowName.c_str());
        cv::createTrackbar(trackbarName.c_str(), windowName.c_str(), &trackedFeaturesPathLength, maxTrackedFeaturesPathLength, nullptr);
    }
};

int FeatureTrackerDrawer::trackedFeaturesPathLength = 10;
const auto leftWindowName = "left";
auto leftFeatureDrawer = FeatureTrackerDrawer("Feature tracking duration (frames)", leftWindowName);
const auto rightWindowName = "right";
auto rightFeatureDrawer = FeatureTrackerDrawer("Feature tracking duration (frames)", rightWindowName);
void left_image_cb(const sensor_msgs::Image::ConstPtr& img_msg)
{
    auto mat = leftFeatureDrawer.drawFeatures(img_msg) ; 
    cv::imshow(leftWindowName,mat) ;
    cv::waitKey(1) ; 
}

void left_tracker_cb(const depthai_ros_msgs::TrackedFeaturesAndImage::ConstPtr& left_track)
{
    leftFeatureDrawer.trackFeaturePath(left_track) ;
}

void right_image_cb(const sensor_msgs::Image::ConstPtr& img_msg)
{
    auto mat = rightFeatureDrawer.drawFeatures(img_msg) ; 
    cv::imshow(rightWindowName,mat) ;
    cv::waitKey(1) ; 
}

void right_tracker_cb(const depthai_ros_msgs::TrackedFeaturesAndImage::ConstPtr& right_track)
{
    rightFeatureDrawer.trackFeaturePath(right_track) ;
}

int main(int argc , char** argv)
{
    ros::init(argc, argv, "feature_visualizer");
    ros::NodeHandle nh("~") ; 
    printf("Press 's' to switch between Lucas-Kanade optical flow and hardware accelerated motion estimation! \n");
    
    left_img_sub = nh.subscribe("/feature_tracker_node/left/image_raw",3,left_image_cb) ; 
    left_tracker_sub = nh.subscribe("/feature_tracker_node/featureAndImageLeft",3,left_tracker_cb) ; 

    
    right_img_sub = nh.subscribe("/feature_tracker_node/right/image_raw",3,right_image_cb) ;
    right_tracker_sub = nh.subscribe("/feature_tracker_node/featureAndImageRight",3,right_tracker_cb) ; 
    ros::spin() ; 
}