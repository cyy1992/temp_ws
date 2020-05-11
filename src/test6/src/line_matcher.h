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

#ifndef LINEMATCHER_H
#define LINEMATCHER_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ceres/ceres.h>
class LineMatcher
{
public:
  LineMatcher();
  ~LineMatcher();
  
private:
  void init();
  ros::NodeHandle nh_;
  ros::Publisher scan1_pub_,scan2_pub_;
  cv::Mat line_img_;
};

#endif // LINEMATCHER_H
