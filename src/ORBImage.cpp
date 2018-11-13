//
// Created by han on 18-11-13.
//

#include "include/bebop_slam/ORBImage.h"


void ORBImage::image_process(const sensor_msgs::ImageConstPtr &msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    SE3_ = SLAM_ptr_->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

    if(SE3_.empty()){
        ROS_INFO("empty");
    }else{
        ROS_INFO_STREAM(SE3_);
    }
}
