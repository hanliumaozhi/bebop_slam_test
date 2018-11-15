//
// Created by han on 18-11-13.
//

#ifndef BEBOP_SLAM_ORBIMAGE_H
#define BEBOP_SLAM_ORBIMAGE_H

#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <System.h>
#include <Converter.h>

class ORBImage {
public:
    ORBImage(std::shared_ptr<ORB_SLAM2::System> SLAM_ptr, ros::NodeHandlePtr nh);
    void image_process(const sensor_msgs::ImageConstPtr& msg);
    void orientation_process(const nav_msgs::OdometryConstPtr& msg);

private:
    std::shared_ptr<ORB_SLAM2::System> SLAM_ptr_;
    cv::Mat SE3_;
    ros::NodeHandlePtr nh_;

    tf::Vector3 global_trans_;
    tf::Vector3 global_trans_copy_;
    std::mutex position_data_;
    tf::Transform drone_transform_;
    tf::Quaternion drone_ori_;
};


#endif //BEBOP_SLAM_ORBIMAGE_H
