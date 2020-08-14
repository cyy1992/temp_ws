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

#ifndef WHEELODOMTOTF_H
#define WHEELODOMTOTF_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
class WheelOdomToTF
{
public:
  WheelOdomToTF(const ros::NodeHandle& nh_);
  ~WheelOdomToTF();
  
private:
  void HandleWheelOdomtry(const nav_msgs::Odometry::ConstPtr& msg);
  
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  Eigen::Affine3d initial_pose_;
  bool initialised_;
  
};

#endif // WHEELODOMTOTF_H
