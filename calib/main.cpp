/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include "System.h"
#include "calib.hpp"

using namespace std;

class ImageGrabber
{
private:
    ORB_SLAM3::System* mpSLAM;
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM) : mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB1,const sensor_msgs::ImageConstPtr& msgD1)
    {
        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptrRGB1;
        try
        {
            cv_ptrRGB1 = cv_bridge::toCvShare(msgRGB1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptrD1;
        try
        {
            cv_ptrD1 = cv_bridge::toCvShare(msgD1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        mpSLAM->TrackRGBD(cv_ptrRGB1->image, cv_ptrD1->image, cv_ptrRGB1->header.stamp.toSec());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "doubleRGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "./build/calib path_to_vocabulary path_to_settings_1 path_to_settings_2" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM1(argv[1], argv[2], ORB_SLAM3::System::RGBD, true, 0, "Camera 1");
    ORB_SLAM3::System SLAM2(argv[1], argv[3], ORB_SLAM3::System::RGBD, true, 0, "Camera 2");

    ImageGrabber igb(&SLAM1);
    ImageGrabber igb2(&SLAM2);
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub1(nh, "/usb_front/image", 1000);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub1(nh, "/usb_front/depth/image_raw", 1000);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub2(nh, "/usb_back/image", 1000);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub2(nh, "/usb_back/depth/image_raw", 1000);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub1, depth_sub1);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));
    message_filters::Synchronizer<sync_pol> sync2(sync_pol(10), rgb_sub2, depth_sub2);
    sync2.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb2, _1, _2));

    ros::spin();
    // Stop all threads
    SLAM1.Shutdown();
    SLAM2.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ros::shutdown();
    cout << "SLAM are shutdown" << endl;
    cout << "start to calib..." << endl;

    CalibC2C c2c(&SLAM1, &SLAM2);
    c2c.RunCalib();
    
    cout << "calib finish, exit" << endl;

    return 0;
}



