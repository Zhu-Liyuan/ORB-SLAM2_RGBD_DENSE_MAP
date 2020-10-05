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
#include<tf/transform_broadcaster.h>
#include <ros/ros.h>
#include "../../../include/Converter.h"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include<opencv2/core/core.hpp>

#include"System.h"
#include "SlamData.h"
using namespace std;

class ImageGrabber
{
public:

    ORB_SLAM2::System* mpSLAM;

    ORB_SLAM2::SlamData* mpSLAMDATA;
    
    ImageGrabber(ORB_SLAM2::System* pSLAM, ORB_SLAM2::SlamData* pSLAMDATA)
    {
        mpSLAM = pSLAM;
        mpSLAMDATA = pSLAMDATA;
    }

   

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

   
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle nh;
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    ORB_SLAM2::SlamData SLAMDATA(&SLAM, &nh, true);/////////bEnablePublishROSTopic=true

    ImageGrabber igb(&SLAM, &SLAMDATA);  

  

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{


     mpSLAMDATA->SaveTimePoint(ORB_SLAM2::SlamData::TimePointIndex::TIME_BEGIN);
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mpSLAMDATA->SaveTimePoint(ORB_SLAM2::SlamData::TimePointIndex::TIME_FINISH_CV_PROCESS);

    
    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    
    mpSLAMDATA->SaveTimePoint(ORB_SLAM2::SlamData::TimePointIndex::TIME_FINISH_SLAM_PROCESS);

    mpSLAMDATA->CalculateAndPrintOutProcessingFrequency();

    if (Tcw.empty()) {
      return;
    }

    if (mpSLAMDATA->EnablePublishROSTopics())
    {

        mpSLAMDATA->PublishTFForROS(Tcw, cv_ptrRGB);

        mpSLAMDATA->PublishPoseForROS(cv_ptrRGB);

        mpSLAMDATA->PublishPointCloudForROS();
        std::cout<< "0"<<std::endl;
        mpSLAMDATA->PublishDenseMap();
        std::cout<< "9"<<std::endl;
    }
}



