/*
 * Copyright 2019 <copyright holder> <email>
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

#ifndef PUBSUBMAPPOINTS_H
#define PUBSUBMAPPOINTS_H
#include "dbg.h"
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud.h>
#include <cartographer_ros_msgs/SubmapLasermark.h>

class PubSubmapPoints
{
public:
  PubSubmapPoints(const ros::NodeHandle& n);
private:
  void handleSubmapLasermark(const cartographer_ros_msgs::SubmapLasermark::ConstPtr& msg);
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher points_pub_;
};

#endif // PUBSUBMAPPOINTS_H
