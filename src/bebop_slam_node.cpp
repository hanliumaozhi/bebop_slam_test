//
// Created by han on 18-11-13.
//

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "include/bebop_slam/ORBImage.h"

using orb_ptr = std::shared_ptr<ORB_SLAM2::System>;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bebop_slam_node");
    ros::start();

    orb_ptr SLAM_ptr = std::make_shared<ORB_SLAM2::System>(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    ORBImage igb(SLAM_ptr);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ORBImage::image_process, &igb);

    ros::spin();

    // Stop all threads
    SLAM_ptr->Shutdown();


    ros::shutdown();

    return 0;
}
