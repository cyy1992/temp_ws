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

#ifndef PUBGPSPATH_H
#define PUBGPSPATH_H
#include <queue>
#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>
#include <glog/logging.h>
#include <cartographer/mapping/trajectory_node.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <gps_common/GPSFix.h>
#include <vtr_msgs/GlobalLocalizationPose.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <chrono>
#include <unordered_map>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>

class PubGpsPath
{
public:
  struct Rigid3d
  {
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    Rigid3d():q(Eigen::Quaterniond::Identity()),t(Eigen::Vector3d(0,0,0)){}
    Rigid3d(Eigen::Quaterniond q1,Eigen::Vector3d t1):q(q1),t(t1){}
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
    
    Eigen::Vector3f operator*(const Eigen::Vector3f& p2)
    {
      Eigen::Vector3f p3;
      p3 = this->q.cast<float>() * p2 + this->t.cast<float>();
      return p3;
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
public:
  
  PubGpsPath(const ros::NodeHandle& n, const std::string& map_path);
  ~PubGpsPath();
  void handleGps(const gps_common::GPSFix::ConstPtr &msg);
  void handlePointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
private:
  void setParam();
  Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                   const double altitude);
  Eigen::Quaterniond eul2quat(const Eigen::Vector3d& eul);
  void pathPub(const gps_common::GPSFix& msg);
  
  bool SaveDataSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tfListener_;
  std::vector<ros::ServiceServer> service_servers_; 
  ros::Publisher path_pub_;  
  ros::Subscriber gps_sub_;
  ros::Subscriber lidar_sub_;
  
  Rigid3d ecef_in_map_;
  Rigid3d ecef_to_local_frame_,gps_to_base_init_,fix_in_map_;
  Rigid3d gps2base_;
  
  double fix_frame_in_map_yaw_;
  // gps_type: poslvx INS
  std::queue<gps_common::GPSFix> gps_datas_;
  gps_common::GPSFix valid_msg_;
  
  std::chrono::steady_clock::time_point last_pub_time_;
  bool initialized_;
  
  geometry_msgs::TransformStamped base2odom_tf_;
  float position_covariance_, orientation_covariance_;
  std::string map_path_;
  
  nav_msgs::Path gps_path_;
  bool debug_;
  bool use_lidar_;
  
  
  ros::Publisher path_mapping_pub_;
  nav_msgs::Path gps_mapping_path_;
  bool init_flag_;
  bool use_gps_manual_;
  std::queue<float> altitude_queue_;
  float sum_altitude_;
  bool has_gps_data_info_;
  
  Rigid3d init_gps_pose_;
  std::map<ros::Time, std::vector<Rigid3d>> poses_with_time_;
  std::queue<ros::Time> times_;
  
  std::queue<ros::Time> cloud_times_;
  std::map<ros::Time, cartographer::sensor::PointCloud> origin_cloud_with_time_;
  std::map<ros::Time, cartographer::mapping::TrajectoryNode::Data> cloud_with_time_;
  
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
//   std::vector<cartographer::mapping::TrajectoryNode::Data> trajectory_nodes_;
};

#endif // PUBGPSPATH_H
