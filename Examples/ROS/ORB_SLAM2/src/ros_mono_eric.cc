/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include <ros/package.h>
#include"../../../include/System.h"
#include "MapPublisher.h" //new added by eric.

using namespace std;
using namespace ORB_SLAM2;
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ORB_SLAM2::MapPublisher* pMapPub)
    :mpSLAM(pSLAM),mpMapPub(pMapPub){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
    ORB_SLAM2::MapPublisher* mpMapPub;//new added by eric.
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_eric");
    ros::start();

    string packagePath = ros::package::getPath("ORB_SLAM2");
    string configPath = packagePath + "//config//ros_mono_eric.yaml";

    cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
      cerr << "ERROR: Wrong path to settings !!" << endl;
      return -1;
    }

    string vocPath = fsSettings["vocPath"];
    string settingPath = fsSettings["settingPath"];

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocPath,settingPath,ORB_SLAM2::System::MONOCULAR,true);

    MapPublisher* mpMapPub_ = new MapPublisher(SLAM.mpMap);

    ImageGrabber igb(&SLAM, mpMapPub_);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    
    mpMapPub->Refresh();
}


