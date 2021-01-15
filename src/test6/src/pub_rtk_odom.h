/*
 * Copyright 2021 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef PUBRTKODOM_H
#define PUBRTKODOM_H
#include <iostream>
#include <gps_common/GPSFix.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PolygonStamped.h>
#include <cartographer/mapping/pose_extrapolator.h>
#include <sensor_msgs/Imu.h>
class PubRtkOdom
{
public:
  PubRtkOdom(const ros::NodeHandle& n);
  ~PubRtkOdom();
  
private:
  void HandleGps(const gps_common::GPSFix::ConstPtr &gps_msg);
  void HandleImu(const sensor_msgs::Imu::ConstPtr& imu_msg);
  void PubTf(const ::ros::WallTimerEvent& unused_timer_event);
  std::unique_ptr<cartographer::sensor::ImuData> ToImuData(
    const sensor_msgs::Imu::ConstPtr& msg);
  Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude);
  const cartographer::transform::Rigid3d ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude, const double altitude);   
  
  ros::NodeHandle nh_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  
  ros::WallTimer wall_timer_;
  ros::Subscriber rtk_sub_;
  ros::Subscriber imu_sub_;
  cartographer::transform::Rigid3d gps2base_, imu2base_, ecef_in_map_,gps2trailer_;
  cartographer::transform::Rigid3d ecef_to_local_frame_,gps_to_base_init_,fix_in_map_;
  bool init_flag_, imu2base_init_, initialised_;
  
  std::unique_ptr<cartographer::mapping::PoseExtrapolator> pose_extrapolator_;
};

#endif // PUBRTKODOM_H
