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

#ifndef GETTFPOSES_H
#define GETTFPOSES_H
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <math.h>
#include <algorithm>
class GetTfPoses
{
public:
  GetTfPoses(const ros::NodeHandle& n);
  ~GetTfPoses();
  
private:
  void GetTfTimer(const ros::WallTimerEvent& unused_timer_event);
  
  ros::NodeHandle nh_;
  ros::ServiceClient switch_localization_client_;
  ros::ServiceClient call_initial_client_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  
  ros::WallTimer timer_pub_;
  int cnt_;
  int localization_state_;
  bool need_change_state_;
  float laser_localization_yaw_, cam_loclaization_yaw_;
};

#endif // GETTFPOSES_H
