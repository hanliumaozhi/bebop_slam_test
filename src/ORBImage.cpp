//
// Created by han on 18-11-13.
//

#include "include/bebop_slam/ORBImage.h"

ORBImage::ORBImage(std::shared_ptr<ORB_SLAM2::System> SLAM_ptr, ros::NodeHandlePtr nh):SLAM_ptr_(SLAM_ptr), nh_(nh){
    global_trans_.setZero();
};

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
        // calc se3 inverse
        //cv::Mat Rwc = SE3_.rowRange(0,3).colRange(0,3).t();
        //cv::Mat twc = -Rwc*SE3_.rowRange(0,3).col(3);

        static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
        static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
        // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
        static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
                -1, 1,-1, 1,
                -1,-1, 1, 1,
                1, 1, 1, 1);
        //prev_pose * T = pose
        cv::Mat translation =  (SE3_ * pose_prev.inv()).mul(flipSign);
        world_lh = world_lh * translation;
        pose_prev = SE3_.clone();
        tf::Matrix3x3 cameraRotation_rh(  - world_lh.at<float>(0,0),   world_lh.at<float>(0,1),   world_lh.at<float>(0,2),
                                          - world_lh.at<float>(1,0),   world_lh.at<float>(1,1),   world_lh.at<float>(1,2),
                                          world_lh.at<float>(2,0), - world_lh.at<float>(2,1), - world_lh.at<float>(2,2));
        tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );
        //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
        const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
                                                0, 0, 1,
                                                1, 0, 0);
        //static tf::TransformBroadcaster br;
        //tf::Matrix3x3 globalRotation_rh = cameraRotation_rh*rotation270degXZ;
        position_data_.lock();
        global_trans_ = cameraTranslation_rh*rotation270degXZ;
        position_data_.unlock();
        //globalRotation_rh.setIdentity();
        /*tf::Vector3 globalTranslation_rh;
        globalTranslation_rh.setX(-globalTranslation_rh_tmp.getX());
        globalTranslation_rh.setY(-globalTranslation_rh_tmp.getY());
        globalTranslation_rh.setZ(-globalTranslation_rh_tmp.getZ());*/
        //tf::Transform transform = tf::Transform(globalRotation_rh, globalTranslation_rh);
        //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_optical", "world"));
        //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
    }
}


void ORBImage::orientation_process(const nav_msgs::OdometryConstPtr& msg)
{
    static tf::TransformBroadcaster br;

    position_data_.lock();
    global_trans_copy_ = global_trans_;
    position_data_.unlock();

    drone_transform_.setOrigin(global_trans_copy_);
    drone_ori_.setX(msg->pose.pose.orientation.x);
    drone_ori_.setY(msg->pose.pose.orientation.y);
    drone_ori_.setZ(msg->pose.pose.orientation.z);
    drone_ori_.setW(msg->pose.pose.orientation.w);
    drone_transform_.setRotation(drone_ori_);
    br.sendTransform(tf::StampedTransform(drone_transform_, ros::Time::now(), "world", "base_link"));

}