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
ParsePoses::ParsePoses(const ros::NodeHandle& n,const std::vector<std::string>& files):nh_(n), file_names_(files)
{
  cout << "files size: " <<files.size() <<endl;
  ReadData(files);
  srvs_.push_back(nh_.advertiseService("/parse_poses/set_step",&ParsePoses::SetStep, this));
//   srvs_.push_back(nh_.advertiseService("/parse_poses/set_submap_show_id",&ParsePoses::SetSubmapShowId, this));
  timer_pub_ = nh_.createWallTimer(ros::WallDuration(1), &ParsePoses::PubPath,this);
  paths_.resize(files.size());
}

ParsePoses::~ParsePoses()
{

}

void ParsePoses::ReadPosesFromFile(const string& file1, std::map< long int, std::vector< Rigid3d >>& poses_with_time)
{
  ifstream fi1;
  fi1.open(file1);
  if(fi1.is_open())
  {
    while(fi1.good())
    {
      long time;
      fi1 >> time;
      Eigen::Vector3d t;
      fi1 >> t(0) >> t(1) >> t(2);
//       t(2) = 0;
      double q[4];
      fi1 >> q[0] >> q[1] >> q[2] >>q[3];
      Eigen::Quaterniond q_temp(q[0] , q[1] , q[2] ,q[3]);
      poses_with_time[time].push_back(Rigid3d(t,q_temp));
      fi1 >> t(0) >> t(1) >> t(2);
//       t(2) = 0;
      fi1 >> q[0] >> q[1] >> q[2] >> q[3];
      Eigen::Quaterniond q_temp2(q[0] , q[1] , q[2] ,q[3]);
      
      poses_with_time[time].push_back(Rigid3d(t,q_temp2));
    }
  }
  fi1.close();
}

void ParsePoses::ReadData(const std::vector<std::string>& files)
{
  for(auto file:files)
    ReadPosesFromFile(file,poses_with_time_);
  cout << poses_with_time_.size()<<endl;
  for(auto  it : poses_with_time_)
  {
    if(it.second.size() != files.size() +1)
    {
      poses_with_time_.erase(poses_with_time_.find(it.first));
    }
  }
//   poses_with_time_.erase(poses_with_time_.rbegin());
  cout << poses_with_time_.size()<<endl;
}

bool ParsePoses::SetStep(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  int step_num = request.data;
//   if(step_num == 0)
//     step_num = 1e8;
  cout << "step_num: " << step_num <<endl;
  int k =0 ;
  Eigen::Vector3d lidar2base_t(2.050, 1.160, 0.917);
  Eigen::Quaterniond lidar2base_q(0.383,-0.007, 0.008, 0.924);
  Rigid3d lidar2base(lidar2base_t, lidar2base_q);

  Rigid3d gps_pose_std = poses_with_time_.begin()->second[0];
  int size_of_poses = poses_with_time_.size();
  vector<Rigid3d> odom_poses;
  std::vector<std::vector<Eigen::Vector3d>> relative_poses;
  relative_poses.resize(paths_.size());
  for(unsigned int i =1; i < poses_with_time_.begin()->second.size(); i++){
    odom_poses.push_back(poses_with_time_.begin()->second[i]);
    paths_[i-1].poses.clear();
    paths_[i-1].poses.reserve(size_of_poses);
    relative_poses[i-1].reserve(size_of_poses);
  } 
  gps_path_.poses.clear();
  long last_time = poses_with_time_.begin()->first;
  cartographer::sensor::PointCloud cloud1, cloud2;

  double last_gps_time = last_time * 1.0 / 1000.0;
  int kk = 0;
  for(auto it : poses_with_time_)
  {
    k++;
    if(step_num !=0 &&(k >= step_num || (it.first - last_time) > 50000))
    {
      gps_pose_std = it.second[0];
      for(unsigned int i =0;i < paths_.size(); i++)
        odom_poses[i] = it.second[i + 1];
      k = 0;
      last_time = it.first;
      last_gps_time = last_time * 1.0 / 1000.0;
      kk++;
    }
//     if(k == 0)
//     {
//       clouds1_.push_back(sensor::VoxelFilter(0.1).Filter(cloud1));
//       cout << clouds1_[kk-1].size() <<endl;
//       cartographer::sensor::PointCloud().swap(cloud1);
//       
//       clouds2_.push_back(sensor::VoxelFilter(0.1).Filter(cloud2));
//       cout <<kk <<", " << clouds2_[kk-1].size() <<endl;
//       cartographer::sensor::PointCloud().swap(cloud2);
//     }
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
    for(unsigned int i =0;i < paths_.size(); i++)
    {
      geometry_msgs::PoseStamped temp_pose;
      Rigid3d pose;
      if(step_num!=0)
        pose = gps_pose_std * odom_poses[i].inverse() * it.second[i + 1];
      else
        pose = it.second[i + 1];
      temp_pose.pose.position.x = pose.translation()(0);
      temp_pose.pose.position.y = pose.translation()(1);
      temp_pose.pose.position.z = 0;
      temp_pose.pose.orientation.x = pose.rotation().x();
      temp_pose.pose.orientation.y = pose.rotation().y();
      temp_pose.pose.orientation.z = pose.rotation().z();
      temp_pose.pose.orientation.w = pose.rotation().w();
      paths_[i].poses.push_back(temp_pose);
      Eigen::Vector3d temp = it.second[0].translation() - pose.translation();
      temp(0) = fabs(temp(0));
      temp(1) = fabs(temp(1));
      temp(2) = 0;
      relative_poses[i].push_back(temp);
    }
    geometry_msgs::PoseStamped temp_pose3;
    
    temp_pose3.pose.position.x = it.second[0].translation().x();
    temp_pose3.pose.position.y = it.second[0].translation().y();
    temp_pose3.pose.position.z = 0;
    temp_pose3.pose.orientation.x = it.second[0].rotation().x();
    temp_pose3.pose.orientation.y = it.second[0].rotation().y();
    temp_pose3.pose.orientation.z = it.second[0].rotation().z();
    temp_pose3.pose.orientation.w = it.second[0].rotation().w();
    gps_path_.poses.push_back(temp_pose3);
  }
  
  for(unsigned int i =0;i < relative_poses.size(); i++){
    Eigen::Vector3d sum1 = std::accumulate(std::begin(relative_poses[i]), std::end(relative_poses[i]),Eigen::Vector3d(0,0,0));
    Eigen::Vector3d mean1 = sum1 / (relative_poses[i].size() -1);
    Eigen::Vector3d accum1(0,0,0);
    double max_norm = 0;
    std::for_each (std::begin(relative_poses[i]), std::end(relative_poses[i]), [&](const Eigen::Vector3d d) {
      accum1(0)  += (d-mean1).x()*(d-mean1).x();
      accum1(1)  += (d-mean1).y()*(d-mean1).y();
      accum1(2)  += (d-mean1).z()*(d-mean1).z();
      if(max_norm < d.norm())
        max_norm = d.norm();
    });
    cout << "mean_"<<to_string(i) << ": " << mean1.transpose() << "\t accum_"<<to_string(i) << ": " << accum1.transpose() <<endl;
    cout << "max_norm_"<<to_string(i) << ": " << max_norm <<endl;
  }
   
  response.success = true;
//   cout << kk <<", " <<clouds1_.size() <<endl;
//   cout << kk <<", " <<clouds2_.size() <<endl;
//   cloud1_ = ToPointCloud2Msg(sensor::VoxelFilter(0.5).Filter(cloud1));
  return true;
}

// bool ParsePoses::SetSubmapShowId(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
// {
//   int show_num = request.data;
//   if(show_num >= clouds1_.size()){
//     response.success = false;
//     return true;
//   }
//   response.success = true;
//   cloud1_ = ToPointCloud2Msg(clouds1_[show_num]);
//   cloud2_ = ToPointCloud2Msg(clouds2_[show_num]);
//   return true;
// }

void ParsePoses::PubPath(const ros::WallTimerEvent& unused_timer_event)
{
  if(gps_path_.poses.size() == 0)
    return;
  
  for(unsigned int i =0;i < paths_.size(); i++)
  {
    paths_[i].header.stamp = ros::Time::now();
    paths_[i].header.frame_id = "odom";
    PubMsg("/parse_poses/path"+to_string(i), paths_[i]);
  }
  
  gps_path_.header.frame_id = "odom";
  gps_path_.header.stamp = ros::Time::now();
  PubMsg("/parse_poses/gps_path", gps_path_);
//   path1_pub_.publish(path1_);
//   path2_pub_.publish(path2_);
//   gps_path_pub_.publish(gps_path_);
}

// void ParsePoses::ParsePbstream(const string& pbstream_path,const std::string& pbstream2_path)
// {
//   io::ProtoStreamReader reader(pbstream_path);
//   SerializedData proto;
//   while (reader.ReadProto(&proto))
//   {
//     switch (proto.data_case()) {
//       case SerializedData::kNode: {
//         mapping::TrajectoryNode::Data data = mapping::FromProto(proto.node().node_data()) ;
//         ros::Time time = cartographer_ros::ToRos(data.time);
//         cloud1_with_time_[time] = data.high_resolution_point_cloud;
//         break;
//       }
//     }
//   }
//   
//   io::ProtoStreamReader reader2(pbstream2_path);
//   SerializedData proto2;
//   while (reader2.ReadProto(&proto2))
//   {
//     switch (proto2.data_case()) {
//       case SerializedData::kNode: {
//         mapping::TrajectoryNode::Data data = mapping::FromProto(proto2.node().node_data()) ;
//         ros::Time time = cartographer_ros::ToRos(data.time);
//         cloud2_with_time_[time] = data.high_resolution_point_cloud;
//         break;
//       }
//     }
//   }
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "parse_poses");
  ros::NodeHandle n;
  std::vector<std::string> file_names;
  for(int i =1; i < argc; i++)
  {
    string file_name = std::string(getenv("HOME")) + "/" + argv[i];
    file_names.push_back(file_name);
  }
  ParsePoses parse(n,file_names);
  ros::spin();
  return 1;
}
