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

#ifndef PROJECTIMG_H
#define PROJECTIMG_H
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <thread>
#include <opencv2/opencv.hpp>
#include "AprilTagsOld/ImageProjector.h"
#include <ros/ros.h>
class ProjectImg
{
public:
  ProjectImg(const ros::NodeHandle& n);
  ~ProjectImg();
  
private:
  ros::NodeHandle nh_;
  
  ImageProjector* p_ip_ ;
};

#endif // PROJECTIMG_H
