//
// Created by han on 18-11-13.
//

#ifndef BEBOP_SLAM_ORBIMAGE_H
#define BEBOP_SLAM_ORBIMAGE_H

#include <memory>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <System.h>

class ORBImage {
public:
    explicit ORBImage(std::shared_ptr<ORB_SLAM2::System> SLAM_ptr):SLAM_ptr_(SLAM_ptr){};
    void image_process(const sensor_msgs::ImageConstPtr& msg);

private:
    std::shared_ptr<ORB_SLAM2::System> SLAM_ptr_;
    cv::Mat SE3_;

};


#endif //BEBOP_SLAM_ORBIMAGE_H
