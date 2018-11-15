//
// Created by han on 18-11-13.
//

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <string>

#include <ros/package.h>

#include "include/bebop_slam/ORBImage.h"

using orb_ptr = std::shared_ptr<ORB_SLAM2::System>;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bebop_slam_node");
    ros::NodeHandlePtr nh = ros::NodeHandlePtr( new ros::NodeHandle( "~" ) );;

    string path_pre = ros::package::getPath("bebop_slam");
    string voc_path = path_pre + "/data/ORBvoc.txt";
    string camera_parameters_path = path_pre + "/data/bebop.yaml";
    orb_ptr SLAM_ptr = std::make_shared<ORB_SLAM2::System>(voc_path, camera_parameters_path, ORB_SLAM2::System::MONOCULAR, false);
    ORBImage igb(SLAM_ptr, nh);

    ros::Subscriber image_sub = nh->subscribe("/bebop/image_raw", 1, &ORBImage::image_process, &igb);
    ros::Subscriber local_sub = nh->subscribe("/bebop/odom", 1, &ORBImage::orientation_process, &igb);

    ros::AsyncSpinner spinner(3);
    spinner.start();

    while(ros::ok()){
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // Stop all threads
    SLAM_ptr->Shutdown();

    spinner.stop();

    return 0;
}
