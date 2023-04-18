/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "../../../include/System.h"

// for pubbing
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class ImageGrabber
{
public:
  ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}

  void GrabImage(const sensor_msgs::ImageConstPtr &msg);

  void SetPub(ros::NodeHandle &nh)
  {
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 100);
  }

  ros::Publisher pub_pose, pub_odom;

  ORB_SLAM3::System *mpSLAM;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mono");
  ros::start();
  bool use_viewer = true;

  if (argc < 3)
  {
    cerr << endl
         << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings [use_viewer]" << endl;
    ros::shutdown();
    return 1;
  }
  if (argc == 4)
  {
    use_viewer = std::stoi(argv[3]);
  }
  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, use_viewer);

  ImageGrabber igb(&SLAM);

  ros::NodeHandle nodeHandler;
  ros::Subscriber sub = nodeHandler.subscribe("/left_cam/image_raw", 20, &ImageGrabber::GrabImage, &igb);
  igb.SetPub(nodeHandler);
  ros::spin();

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  ros::shutdown();

  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat T_, R_, t_;

  // std::cout << "image received!" << std::endl;
  Sophus::SE3f T_sophus = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

  cv::eigen2cv(T_sophus.matrix(), T_);
  R_ = T_.rowRange(0, 3).colRange(0, 3).t();
  t_ = -R_ * T_.rowRange(0, 3).col(3);
  vector<float> q = ORB_SLAM3::Converter::toQuaternion(R_);
  float scale_factor = 1.0;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(t_.at<float>(0, 0) * scale_factor, t_.at<float>(0, 1) * scale_factor, t_.at<float>(0, 2) * scale_factor));
  tf::Quaternion tf_quaternion(q[0], q[1], q[2], q[3]);
  transform.setRotation(tf_quaternion);
  static tf::TransformBroadcaster br_;
  br_.sendTransform(tf::StampedTransform(transform, cv_ptr->header.stamp, "world", "ORB_SLAM3_MONO"));

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = cv_ptr->header.stamp;
  pose.header.frame_id = "ORB_SLAM3_MONO";
  tf::poseTFToMsg(transform, pose.pose);
  pub_pose.publish(pose);

  nav_msgs::Odometry odom;
  odom.header = pose.header;
  odom.pose.pose = pose.pose;
  pub_odom.publish(odom);
}