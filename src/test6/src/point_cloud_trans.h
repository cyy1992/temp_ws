/*
 * Copyright 2020 <copyright holder> <email>
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

#ifndef POINTCLOUDTRANS_H
#define POINTCLOUDTRANS_H
#include <iostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <jdrv_msgs/MMWObjectDataStamped.h>

// #include <glog/logging.h>
#include <chrono>
#include <thread>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class PointcloudTrans
{
public:
  PointcloudTrans(const ros::NodeHandle& n);
  ~PointcloudTrans();
  
  void HandleCloudFront(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void HandleCloudBack(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void calibRollPitchZ(const geometry_msgs::TransformStamped& prior_transform, 
                       const sensor_msgs::PointCloud2& cloud,
                       geometry_msgs::TransformStamped& result_transform);
private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  
  ros::Subscriber cloud1_sub_,cloud2_sub_;
  ros::Publisher cloud1_pub_,cloud2_pub_;
};

#endif // POINTCLOUDTRANS_H
