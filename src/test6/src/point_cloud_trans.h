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

#ifndef POINTCLOUDTRANS_H
#define POINTCLOUDTRANS_H
#include <iostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <jdrv_msgs/MMWObjectDataStamped.h>

// #include <glog/logging.h>
#include <chrono>
#include <thread>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>
#include <test6/SetPose.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_srvs/SetBool.h>
struct Rigid3d
{
  Eigen::Vector3d t;
  Eigen::Quaterniond q;
  Rigid3d():t(Eigen::Vector3d(0,0,0)),q(Eigen::Quaterniond::Identity()){};
  Rigid3d(Eigen::Vector3d t1, Eigen::Quaterniond q1):t(t1),q(q1){};
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
class PointcloudTrans
{
typedef pcl::PointXYZ PointType;

public:
  PointcloudTrans(const ros::NodeHandle& n);
  ~PointcloudTrans();
  
  void HandleCloudBack(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void ICPMatch(const pcl::PointCloud<PointType>::Ptr &cloud_target, 
                 const pcl::PointCloud<PointType>::Ptr &cloud_source, 
                 Eigen::Matrix4d &transform );
  void PubTf(const ros::WallTimerEvent& unused_timer_event);
  bool SetTargetPose(test6::SetPose::Request& request, test6::SetPose::Response& response);
  bool SetModelName(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
private:
  void setPolyGon();
  ros::NodeHandle nh_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Subscriber cloud2_sub_;
  std::vector<ros::ServiceServer> servers_;
  ros::Publisher cloud2_pub_, cloud_all_pub_;
  ros::Publisher target_pub_, source_pub_;
  ros::Publisher polygon_pub_;
  ros::Publisher model_pose_pub_;
  ros::WallTimer wall_timer_;
  geometry_msgs::TransformStamped lidar2base_tf_;
  std::vector<Eigen::Vector3d> cloud_points_; 
  pcl::VoxelGrid<PointType> down_sample_;
  pcl::PointCloud<PointType>::Ptr points_;
  pcl::PointCloud<PointType>::Ptr target_points_;
  sensor_msgs::PointCloud2 target_msg_;
  geometry_msgs::PolygonStamped obj_polygon_;
  float ref_x, ref_y, ref_z;
  float max_x_,max_y_,max_z_;
  Rigid3d ref_odom2odom_;
  Rigid3d obj2ref_odom_;
  bool initialised_;
  geometry_msgs::TransformStamped ref_odom2odom_tf_,obj2ref_odom_tf_;
  std::string model_name;
};

#endif // POINTCLOUDTRANS_H
