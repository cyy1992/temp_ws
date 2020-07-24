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

#ifndef ADJUSTIMAGEFREQUENCY_H
#define ADJUSTIMAGEFREQUENCY_H
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sensor_msgs/Image.h>

class AdjustImageFrequency
{
public:
  AdjustImageFrequency(const ros::NodeHandle& nh,const float& frq);
  ~AdjustImageFrequency();
  
private:
  void imgCb(const sensor_msgs::ImageConstPtr& img_msg);
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_;
  ros::Publisher img_pub_;
  float interval_;
  ros::Time last_pub_msg_time_;
  //      nh_.subscribe("/jzhw/jzcamera", 1, &BagToTxt::imgCb, this);
};

#endif // ADJUSTIMAGEFREQUENCY_H
