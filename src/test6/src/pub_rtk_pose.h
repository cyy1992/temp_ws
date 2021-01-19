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

#ifndef PUBRTKPOSE_H
#define PUBRTKPOSE_H
#include <iostream>
#include <gps_common/GPSFix.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PolygonStamped.h>
#include "visualization_msgs/MarkerArray.h"
class PubRtkPose
{
struct Rigid3d
{
  Eigen::Vector3d t;
  Eigen::Quaterniond q;

  Rigid3d():t(Eigen::Vector3d(0,0,0)),q(Eigen::Quaterniond(1,0,0,0)){}
  Rigid3d(const Eigen::Vector3d& t1, const Eigen::Quaterniond& q1):t(t1),q(q1){}
//     Rigid3d():t(Eigen::Vector3d(0,0,0)),q(Eigen::Quaterniond(1,0,0,0)){};
  friend std::ostream & operator << (std::ostream &, Rigid3d &pose)
  {
    Eigen::Vector3d euler = pose.q.toRotationMatrix().eulerAngles(2,1,0);
    std::cout <<"\033[1m\033[36m"<< "t:[" << pose.t(0) <<","<<pose.t(1)<<","<<pose.t(2) 
    <<"]  euler:["<<euler(0) <<"," <<euler(1)<<"," <<euler(2) <<"]"<<"\033[0m"<<std::endl;
//       q:["<<pose.q.w() <<"," <<pose.q.x()<<"," <<pose.q.y()<<"," <<pose.q.z() <<"]"<<std::endl;
    return std::cout;
  }
  double getYaw()
  {
    const Eigen::Matrix<double, 3, 1> direction =
    q * Eigen::Matrix<double, 3, 1>::UnitX();
    return atan2(direction.y(), direction.x());
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
  PubRtkPose(const ros::NodeHandle& n);
  ~PubRtkPose();
  
  void HandleGps(const gps_common::GPSFix::ConstPtr &gps_msg);
  void HandleZbPose(const geometry_msgs::Pose::ConstPtr& msg);
private:
  void setParam();
  Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude);
  const Rigid3d ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude, const double altitude);                             
  ros::NodeHandle nh_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Publisher model_pose_pub_;
  ros::Publisher polygon_pub_;
  ros::Publisher zb_polygon_pub_;
  ros::Publisher car_polygon_pub_;
  ros::Publisher markers_pub_;
  ros::Subscriber rtk_sub_;
  ros::Subscriber zb_sub_;
  geometry_msgs::PolygonStamped obj_polygon_;
  geometry_msgs::PolygonStamped zb_polygon_;
  geometry_msgs::PolygonStamped car_polygon_;
  Rigid3d ecef_in_map_,gps2trailer_;
  Rigid3d ecef_to_local_frame_,gps_to_base_init_,fix_in_map_;

  bool has_gps_data_info_;
  geometry_msgs::TransformStamped zb_in_map_tf_;
  visualization_msgs::MarkerArray zb_markers_;
  Rigid3d ear1_in_map_,ear2_in_map_, hook1_in_trailer_, hook2_in_trailer_;
  
};

#endif // PUBRTKPOSE_H
