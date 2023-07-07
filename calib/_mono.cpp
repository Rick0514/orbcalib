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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include "System.h"

using namespace std;

class ImageGrabber
{
private:
    ORB_SLAM3::System* mpSLAM;
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM) : mpSLAM(pSLAM){}

    void GrabMono(const sensor_msgs::ImageConstPtr& msgRGB)
    {
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
        mpSLAM->TrackMonocular(cv_ptrRGB->image, cv_ptrRGB->header.stamp.toSec());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calib_node");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "./build/calib path_to_vocabulary camera_setttings" << endl;
        ros::shutdown();
        return 1;
    }    
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true, 0, "Camera 1");

    ImageGrabber igb(&SLAM);
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber rgb_sub;
    string topic = "/usb_front/image";
    rgb_sub = it.subscribe(topic, 1000, &ImageGrabber::GrabMono, &igb);

    ros::spin();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ros::shutdown();
    cout << "SLAM are shutdown" << endl;

    return 0;
}
