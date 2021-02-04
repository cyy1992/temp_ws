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

#ifndef TFTOPATH_H
#define TFTOPATH_H
#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <chrono>
#include <nav_msgs/Path.h>
class TfToPath
{
public:
  TfToPath(const ros::NodeHandle& nh,
           const std::string& frame,
           const std::string& sub_frame);
  
  
private:
  void saveTF(const ros::WallTimerEvent& unused_timer_event);
  
  ros::NodeHandle nh_;
  std::string frame_,sub_frame_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  
  ros::WallTimer timer_;
  
  nav_msgs::Path tf_path_;
  ros::Publisher path_pub_;
  ros::Publisher points_pub_;
  Eigen::Affine3d first_pose_;
  bool initial_;

};

#endif // TFTOPATH_H
