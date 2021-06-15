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

#ifndef UWBFILTER_H
#define UWBFILTER_H
#include <iostream>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <nlink_parser/LinktrackNodeframe2.h>

#include <opencv2/opencv.hpp>

#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
class Kalman{
public:
  Kalman();
  double Filter(const double& det_time, const double& distance);
  
private:
  double Var();
  cv::KalmanFilter kf_;
  cv::Mat state_;
  cv::Mat measurement_;
  bool initialised_;
  std::vector<double> initial_distances_;
  double last_distance_;
  std::vector<double> distance_queue_;
  
  Eigen::Vector2d x_k;
  Eigen::Vector2d z_k;
  Eigen::Matrix2d A;
  Eigen::Vector2d H;

  Eigen::Matrix2d P_k;
  Eigen::Matrix2d Q_k;
  double R_k;
  Eigen::Vector2d K;
  int unpub_cnt_;
};

class UwbFilter
{
public:
  UwbFilter(const ros::NodeHandle& n);
  ~UwbFilter();
  
private:
  void HandleUwbMsg(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg);
  
  ros::NodeHandle nh_;
  ros::Subscriber uwb_sub_;
  ros::Publisher filter_uwb_pub_;
  
  std::map<int, int> unuse_distance_count_;
  std::map<int, std::shared_ptr<Kalman> > kalman_filters_;
  
  // visualization
  std::map<int, ros::Publisher> origin_path_pub_;
  std::map<int, ros::Publisher> filtered_path_pub_;
  std::map<int, ros::Time> last_times_;
  int max_distance_;
  
};

#endif // UWBFILTER_H
