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

#ifndef DEPTHCOLORREGISTRATION_H
#define DEPTHCOLORREGISTRATION_H
#include <ros/ros.h>
#include <fstream>
#include <algorithm>
#include <math.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

class DepthColorRegistration
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> DepthColorSyncPolicy;
public:
  DepthColorRegistration();
  ~DepthColorRegistration();
  
private:
  void imagesCallback(const sensor_msgs::Image::ConstPtr &depth_msg,
                     const sensor_msgs::Image::ConstPtr &color_msg);
  bool Register(cv::Mat &depth, cv::Mat &color, cv::Mat &out);
  ros::NodeHandle nh_;
    
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::Image> color_sub_;
  message_filters::Synchronizer<DepthColorSyncPolicy> *scansSync_;
  cv::Mat depth_intrinsic_;
  cv::Mat color_intrinsic_;
  cv::Mat depth2color_;
  cv::Mat color2depth_;
  
};

#endif // DEPTHCOLORREGISTRATION_H
