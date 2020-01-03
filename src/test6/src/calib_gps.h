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

#ifndef CALIBGPS_H
#define CALIBGPS_H
#include "dbg.h"
#include <iostream>
#include <gps_common/GPSFix.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>
#include <opencv2/opencv.hpp>
class CalibGps
{
  struct Rigid3d
  {
    Eigen::Vector3d t;
    Eigen::Quaterniond q;
    friend std::ostream & operator << (std::ostream &, Rigid3d &pose)
    {
      std::cout << "t:[" << pose.t(0) <<","<<pose.t(1)<<","<<pose.t(2) 
      <<"]  q:["<<pose.q.w() <<"," <<pose.q.x()<<"," <<pose.q.y()<<"," <<pose.q.z() <<"]"<<std::endl;
      
      return std::cout;
    }
    
    Rigid3d inverse()
    {
      Rigid3d inv;
      inv.q = q.conjugate();
      inv.t = -(inv.q * t);
      return inv;
    }
    
    Rigid3d operator*(const Rigid3d& p2)
    {
      Rigid3d p3;
      p3.t = this->q * p2.t + this->t;
      p3.q = this->q * p2.q;
      return p3;
    }
    
    Eigen::Vector3d operator*(const Eigen::Vector3d& p2)
    {
      Eigen::Vector3d p3;
      p3 = this->q * p2 + this->t;
      return p3;
    }
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  
public:
  CalibGps(const ros::NodeHandle& n);
  ~CalibGps();
  
private:
  Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                   const double altitude);
  const CalibGps::Rigid3d ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude);
  void handleGps(const gps_common::GPSFix::ConstPtr& msg);
  bool updateState(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  void computeExtrinsicParams();
  Rigid3d ecef_to_local_frame_;
  nav_msgs::Path gps_path_;
  ros::Publisher path_pub_;
  
  ros::NodeHandle nh_;
  ros::Subscriber gps_sub_;
  ros::ServiceServer state_srv_;
  bool initialized_;
  int state_;
  std::vector<cv::Point2d> positions_[2];
  double average_dip_;
  int num_dip_;
};

#endif // CALIBGPS_H
