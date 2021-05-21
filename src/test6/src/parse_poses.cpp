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

#include "parse_poses.h"
#include <cartographer/io/proto_stream.h>
#include <cartographer/io/proto_stream_deserializer.h>
#include "cartographer/mapping/proto/serialization.pb.h"
#include <cartographer_ros/time_conversion.h>
#include <cartographer_ros/msg_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cartographer/sensor/internal/voxel_filter.h>

using namespace std;
using namespace std::chrono;
using namespace cartographer;
using mapping::proto::SerializedData;

ParsePoses::ParsePoses(const ros::NodeHandle& n):nh_(n)
{
  path1_pub_ = nh_.advertise<nav_msgs::Path>("/parse_poses/path1",1);
  path2_pub_ = nh_.advertise<nav_msgs::Path>("/parse_poses/path2",1);
  gps_path_pub_ = nh_.advertise<nav_msgs::Path>("/parse_poses/gps_path",1);
  cloud1_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/parse_poses/submap1",1);
  cloud2_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/parse_poses/submap2",1);
  ReadData("/home/cyy/1.txt","/home/cyy/2.txt" );
  srvs_.push_back(nh_.advertiseService("/parse_poses/set_step",&ParsePoses::SetStep, this));
  srvs_.push_back(nh_.advertiseService("/parse_poses/set_submap_show_id",&ParsePoses::SetSubmapShowId, this));
  timer_pub_ = nh_.createWallTimer(ros::WallDuration(1), &ParsePoses::PubPath,this);
  
//   string pbstream1 = "/home/cyy/11.pbstream";
//   string pbstream2 = "/home/cyy/22.pbstream";
//   ParsePbstream(pbstream1,pbstream2);
}

ParsePoses::~ParsePoses()
{

}

void ParsePoses::ReadData(const string& file1, const string& file2)
{
  ifstream fi1,fi2;
  fi1.open(file1);
  if(fi1.is_open())
  {
    while(fi1.good())
    {
      long time;
      fi1 >> time;
      Eigen::Vector3d t;
      fi1 >> t(0) >> t(1) >> t(2);
      double q[4];
      fi1 >> q[0] >> q[1] >> q[2] >>q[3];
      Eigen::Quaterniond q_temp(q[0] , q[1] , q[2] ,q[3]);
      Rigid3d gps_pose(q_temp,t);
      
      fi1 >> t(0) >> t(1) >> t(2);
      fi1 >> q[0] >> q[1] >> q[2] >> q[3];
      Eigen::Quaterniond q_temp2(q[0] , q[1] , q[2] ,q[3]);
      Rigid3d lidar_pose(q_temp2,t);
      if(t(0) > 1e8 || t(1) > 1e8 ||t(2) > 1e8)
        continue;
      poses_with_time_[time].push_back(gps_pose);
      poses_with_time_[time].push_back(lidar_pose);
    }
  }
  fi1.close();
  fi2.open(file2);
  if(fi2.is_open())
  {
    while(fi2.good())
    {
      long time;
      fi2 >> time;
      Eigen::Vector3d t;
      fi2 >> t(0) >> t(1) >> t(2);
      double q[4];
      fi2 >> q[0] >> q[1] >> q[2] >>q[3];
      Eigen::Quaterniond q_temp(q[0] , q[1] , q[2] ,q[3]);
      Rigid3d gps_pose(q_temp, t);
      
      fi2 >> t(0) >> t(1) >> t(2);
      fi2 >> q[0] >> q[1] >> q[2] >>q[3];
      Eigen::Quaterniond q_temp2(q[0] , q[1] , q[2] ,q[3]);
      Rigid3d lidar_pose(q_temp2,t);
      if(t(0) > 1e8 || t(1) > 1e8 ||t(2) > 1e8)
        continue;
      poses_with_time_[time].push_back(lidar_pose);
    }
  }
  cout << poses_with_time_.size()<<endl;
  
  for(auto  it : poses_with_time_)
  {
    if(it.second.size() != 3)
    {
      poses_with_time_.erase(poses_with_time_.find(it.first));
    }
  }
  cout << poses_with_time_.size()<<endl;
  saveAsKittiFormat();
  fi2.close();
}

void ParsePoses::saveAsKittiFormat()
{
  vector<string> save_file_names{"kitti_rtk", "kitti_lego", "kitti_lio"};
  for(int i =0; i< save_file_names.size(); i++){
    ofstream outFile1;
    outFile1.open("/home/cyy/" + save_file_names[i] + ".txt", std::ios::out);
    for(auto it : poses_with_time_)
    {
      Rigid3d pose = it.second[i];
      Eigen::Matrix3d rot = pose.q.toRotationMatrix();
      outFile1 << rot(0,0) << " "<< rot(0,1) << " "<< rot(0,2) << " "<< pose.t.x() << " " 
        << rot(1,0) << " "<< rot(1,1) << " "<< rot(1,2) << " "<< pose.t.y() << " " 
        << rot(2,0) << " "<< rot(2,1) << " "<< rot(2,2) << " "<< pose.t.z() << endl;
      
    }
    outFile1.close();
  }
  cout << "write done!" <<endl;
}

sensor_msgs::PointCloud2 ParsePoses::ToPointCloud2Msg(const sensor::PointCloud& cloud)
{
  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "odom";
  cloud_msg.height = 1;
  cloud_msg.width = cloud.size();
//   cout << points_.size() <<endl;
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                "y", 1, sensor_msgs::PointField::FLOAT32, "z",
                                1, sensor_msgs::PointField::FLOAT32, "intensity",
                                1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(cloud.size());
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");
  for(const auto& point : cloud)
  {
    *iter_x = point.position.x();
    *iter_y = point.position.y();
    *iter_z = point.position.z();
    *iter_intensity = point.intensity;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }
  return cloud_msg;
}

bool ParsePoses::SetStep(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  int step_num = request.data;
  cout << "step_num: " << step_num <<endl;
  int k =0 ;
  Eigen::Vector3d lidar2base_t(2.050, 1.160, 0.917);
  Eigen::Quaterniond lidar2base_q(0.383,-0.007, 0.008, 0.924);
  Rigid3d lidar2base(lidar2base_q,lidar2base_t);

  Rigid3d gps_pose_std = poses_with_time_.begin()->second[0];
  Rigid3d odom1_pose_std = poses_with_time_.begin()->second[1];
  Rigid3d odom2_pose_std = poses_with_time_.begin()->second[2];
  path1_.poses.clear();
  path2_.poses.clear();
  gps_path_.poses.clear();
  
  long last_time = poses_with_time_.begin()->first;
  auto temp111 = cloud1_with_time_;
  auto temp222 = cloud2_with_time_;
  map<ros::Time, sensor::PointCloud>::iterator cloud_with_time_it = temp111.begin();
  map<ros::Time, sensor::PointCloud>::iterator cloud2_with_time_it = temp222.begin();
  cartographer::sensor::PointCloud cloud1, cloud2;
//   for(auto  cloud:cloud1_with_time_)
//   {
//     
//     if((kk++) > 100)
//       cloud1.insert(cloud1.end(),cloud.second.begin(),cloud.second.end());
// //     else if((kk++) > 3 * step_num )
// //       break;
//   }
  double last_gps_time = last_time * 1.0 / 1000.0;
  int kk = 0;
  int size_of_poses = poses_with_time_.size();
  path1_.poses.reserve(size_of_poses);
  path2_.poses.reserve(size_of_poses);
  std::vector<Eigen::Vector3d> relative_poses1,relative_poses2;
  relative_poses1.reserve(size_of_poses);
  relative_poses2.reserve(size_of_poses);
  for(auto it : poses_with_time_)
  {
    k++;
    if(k >= step_num || (it.first - last_time) > 50000)
    {
      gps_pose_std = it.second[0];
      odom1_pose_std = it.second[1];
      odom2_pose_std = it.second[2];
      k = 0;
      last_time = it.first;
      last_gps_time = last_time * 1.0 / 1000.0;
      kk++;
    }
    if(k == 0)
    {
      clouds1_.push_back(sensor::VoxelFilter(0.1).Filter(cloud1));
      cout << clouds1_[kk-1].size() <<endl;
      cartographer::sensor::PointCloud().swap(cloud1);
      
      clouds2_.push_back(sensor::VoxelFilter(0.1).Filter(cloud2));
      cout <<kk <<", " << clouds2_[kk-1].size() <<endl;
      cartographer::sensor::PointCloud().swap(cloud2);
    }
    double cur_gps_time = it.first * 1.0 / 1000.0;
//     if(temp111.size() > 0){
//       while(last_gps_time - cloud_with_time_it->first.toSec() > 0)
//       {
//         cloud_with_time_it = temp111.erase(cloud_with_time_it);
//       }
//       while(cur_gps_time - cloud_with_time_it->first.toSec() > 0)
//       {
//         cloud1.insert(cloud1.end(),cloud_with_time_it->second.begin(),cloud_with_time_it->second.end());
//         cloud_with_time_it = temp111.erase(cloud_with_time_it);
//       }
//     }
//     
//     if(temp222.size() > 0){
//       while(last_gps_time - cloud2_with_time_it->first.toSec() > 0)
//       {
//         cloud2_with_time_it = temp222.erase(cloud2_with_time_it);
//       }
//       while(cur_gps_time - cloud2_with_time_it->first.toSec() > 0)
//       {
//         cloud2.insert(cloud2.end(),cloud2_with_time_it->second.begin(),cloud2_with_time_it->second.end());
//         cloud2_with_time_it = temp222.erase(cloud2_with_time_it);
//       }
//     }
    
    Rigid3d pose1 = gps_pose_std * odom1_pose_std.inverse() * it.second[1];
    Rigid3d pose2 = gps_pose_std * odom2_pose_std.inverse() * it.second[2];
    geometry_msgs::PoseStamped temp_pose1,temp_pose2,temp_pose3;
//     temp_pose1.header.frame_id = "base_footprint";
//     temp_pose2.header.frame_id = "base_footprint";
//     temp_pose3.header.frame_id = "base_footprint";
    
    Rigid3d relative_pose1 = pose1.inverse() * it.second[0];
    Rigid3d relative_pose2 = pose2.inverse() * it.second[0];
    relative_pose1.t = Eigen::Vector3d(fabs(relative_pose1.t(0)),fabs(relative_pose1.t(1)),fabs(relative_pose1.t(2)) );
    relative_pose2.t = Eigen::Vector3d(fabs(relative_pose2.t(0)),fabs(relative_pose2.t(1)),fabs(relative_pose2.t(2)) );
    relative_poses1.push_back(relative_pose1.t);
    relative_poses2.push_back(relative_pose2.t);
    temp_pose1.pose.position.x = pose1.t(0);
    temp_pose1.pose.position.y = pose1.t(1);
    temp_pose1.pose.position.z = pose1.t(2);
    temp_pose1.pose.orientation.x = pose1.q.x();
    temp_pose1.pose.orientation.y = pose1.q.y();
    temp_pose1.pose.orientation.z = pose1.q.z();
    temp_pose1.pose.orientation.w = pose1.q.w();
    
    temp_pose2.pose.position.x = pose2.t(0);
    temp_pose2.pose.position.y = pose2.t(1);
    temp_pose2.pose.position.z = pose2.t(2);
    temp_pose2.pose.orientation.x = pose2.q.x();
    temp_pose2.pose.orientation.y = pose2.q.y();
    temp_pose2.pose.orientation.z = pose2.q.z();
    temp_pose2.pose.orientation.w = pose2.q.w();
    
    temp_pose3.pose.position.x = it.second[0].t(0);
    temp_pose3.pose.position.y = it.second[0].t(1);
    temp_pose3.pose.position.z = it.second[0].t(2);
    temp_pose3.pose.orientation.x = it.second[0].q.x();
    temp_pose3.pose.orientation.y = it.second[0].q.y();
    temp_pose3.pose.orientation.z = it.second[0].q.z();
    temp_pose3.pose.orientation.w = it.second[0].q.w();
    
    path1_.poses.push_back(temp_pose1);
    path2_.poses.push_back(temp_pose2);
    gps_path_.poses.push_back(temp_pose3);
//     cout <<k << std::endl;
  }
  
  
  Eigen::Vector3d sum1 = std::accumulate(std::begin(relative_poses1), std::end(relative_poses1),Eigen::Vector3d(0,0,0));
  Eigen::Vector3d mean1 = sum1 / (relative_poses1.size() -1);
  Eigen::Vector3d accum1(0,0,0);
  std::for_each (std::begin(relative_poses1), std::end(relative_poses1), [&](const Eigen::Vector3d d) {
    accum1(0)  += (d-mean1).x()*(d-mean1).x();
    accum1(1)  += (d-mean1).y()*(d-mean1).y();
    accum1(2)  += (d-mean1).z()*(d-mean1).z();
  });
  
  Eigen::Vector3d sum2 = std::accumulate(std::begin(relative_poses2), std::end(relative_poses2),Eigen::Vector3d(0,0,0));
  Eigen::Vector3d mean2 = sum2 / (relative_poses2.size() - 1);
  Eigen::Vector3d accum2(0,0,0);
  std::for_each (std::begin(relative_poses2), std::end(relative_poses2), [&](const Eigen::Vector3d d) {
    accum2(0)  += (d-mean2).x()*(d-mean2).x();
    accum2(1)  += (d-mean2).y()*(d-mean2).y();
    accum2(2)  += (d-mean2).z()*(d-mean2).z();
  });
  
  
  cout << "mean1: " << mean1.transpose() << "\t accum1: " << accum1.transpose() <<endl;
  cout << "mean2: " << mean2.transpose() << "\t accum2: " << accum2.transpose() <<endl;
  clouds1_.push_back(cloud1);
  clouds2_.push_back(cloud2);
//   
  response.success = true;
//   cout << kk <<", " <<clouds1_.size() <<endl;
//   cout << kk <<", " <<clouds2_.size() <<endl;
//   cloud1_ = ToPointCloud2Msg(sensor::VoxelFilter(0.5).Filter(cloud1));
  return true;
}

bool ParsePoses::SetSubmapShowId(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  int show_num = request.data;
  if(show_num >= clouds1_.size()){
    response.success = false;
    return true;
  }
  response.success = true;
  cloud1_ = ToPointCloud2Msg(clouds1_[show_num]);
  cloud2_ = ToPointCloud2Msg(clouds2_[show_num]);
  return true;
}

void ParsePoses::PubPath(const ros::WallTimerEvent& unused_timer_event)
{
  if(gps_path_.poses.size() == 0)
    return;
  path1_.header.frame_id = "odom";
  path2_.header.frame_id = "odom";
  gps_path_.header.frame_id = "odom";
  gps_path_.header.stamp = ros::Time::now();
  path1_.header.stamp = ros::Time::now();
  path2_.header.stamp = ros::Time::now();
  path1_pub_.publish(path1_);
  path2_pub_.publish(path2_);
  gps_path_pub_.publish(gps_path_);
  if(cloud1_.height * cloud1_.width > 0){
    cloud1_.header.stamp = ros::Time::now();
    cloud1_pub_.publish(cloud1_);
    
    cloud2_.header.stamp = ros::Time::now();
    cloud2_pub_.publish(cloud2_);
  }
}

void ParsePoses::ParsePbstream(const string& pbstream_path,const std::string& pbstream2_path)
{
  io::ProtoStreamReader reader(pbstream_path);
  SerializedData proto;
  while (reader.ReadProto(&proto))
  {
    switch (proto.data_case()) {
      case SerializedData::kNode: {
        mapping::TrajectoryNode::Data data = mapping::FromProto(proto.node().node_data()) ;
        ros::Time time = cartographer_ros::ToRos(data.time);
        cloud1_with_time_[time] = data.high_resolution_point_cloud;
        break;
      }
    }
  }
  
  io::ProtoStreamReader reader2(pbstream2_path);
  SerializedData proto2;
  while (reader2.ReadProto(&proto2))
  {
    switch (proto2.data_case()) {
      case SerializedData::kNode: {
        mapping::TrajectoryNode::Data data = mapping::FromProto(proto2.node().node_data()) ;
        ros::Time time = cartographer_ros::ToRos(data.time);
        cloud2_with_time_[time] = data.high_resolution_point_cloud;
        break;
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "parse_poses");
  ros::NodeHandle n;
//   string map_name = "/home/cyy/map/puyan_around";
  ParsePoses parse(n);
  ros::spin();
  return 1;
}
