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

#ifndef CHECKODOMIMU_H
#define CHECKODOMIMU_H
#include <ros/ros.h>

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
class CheckOdomImu
{
public:
  CheckOdomImu(const ros::NodeHandle& n);
  ~CheckOdomImu();
  
private:
  void subOdom(const nav_msgs::Odometry::ConstPtr& msg);
  void subImu(const sensor_msgs::Imu::ConstPtr& msg);
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  float last_imu_z;
};

#endif // CHECKODOMIMU_H
