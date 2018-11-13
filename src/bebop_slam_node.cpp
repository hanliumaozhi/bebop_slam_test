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
    ros::start();

    string path_pre = ros::package::getPath("bebop_slam");

    string voc_path = path_pre + "/data/ORBvoc.txt";
    string camera_parameters_path = path_pre + "/data/bebop.yaml";

    std::cout<<voc_path;
    orb_ptr SLAM_ptr = std::make_shared<ORB_SLAM2::System>(voc_path, camera_parameters_path, ORB_SLAM2::System::MONOCULAR, true);

    ORBImage igb(SLAM_ptr);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ORBImage::image_process, &igb);

    ros::spin();

    // Stop all threads
    SLAM_ptr->Shutdown();


    ros::shutdown();

    return 0;
}
