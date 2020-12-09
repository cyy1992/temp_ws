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
using namespace std;
ParsePoses::ParsePoses(const ros::NodeHandle& n):nh_(n)
{
  path1_pub_ = nh_.advertise<nav_msgs::Path>("/ParsePoses/path1",1);
  path2_pub_ = nh_.advertise<nav_msgs::Path>("/ParsePoses/path2",1);
  gps_path_pub_ = nh_.advertise<nav_msgs::Path>("/ParsePoses/gps_path",1);
  ReadData("/home/cyy/1.txt","/home/cyy/2.txt" );
  srv_ = nh_.advertiseService("setStep",&ParsePoses::SetStep, this);
  timer_pub_ = nh_.createWallTimer(ros::WallDuration(1), &ParsePoses::PubPath,this);
  
  
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
      fi1 >> q[0] >> q[1] >> q[2] >>q[3];
      Eigen::Quaterniond q_temp2(q[0] , q[1] , q[2] ,q[3]);
      Rigid3d lidar_pose(q_temp2,t);
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
      poses_with_time_[time].push_back(lidar_pose);
    }
  }
  cout << poses_with_time_.size()<<endl;
  
  for(auto  it : poses_with_time_)
  {
    if(it.second.size() < 3)
    {
      poses_with_time_.erase(poses_with_time_.find(it.first));
    }
  }
  cout << poses_with_time_.size()<<endl;
  fi2.close();
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
  Rigid3d odom2_pose_std = lidar2base * poses_with_time_.begin()->second[2] * lidar2base.inverse();
  path1_.poses.clear();
  path2_.poses.clear();
  gps_path_.poses.clear();
  long last_time = poses_with_time_.begin()->first;
  for(auto it : poses_with_time_)
  {
    k++;
    if(k >= step_num || (it.first - last_time) > 5000)
    {
      gps_pose_std = it.second[0];
      odom1_pose_std = it.second[1];
      odom2_pose_std = lidar2base * it.second[2] * lidar2base.inverse();
      k = 0;
      last_time = it.first;
    }
    Rigid3d pose1 = gps_pose_std * odom1_pose_std.inverse() * it.second[1];
    Rigid3d pose2 = gps_pose_std * odom2_pose_std.inverse() *lidar2base * it.second[2] * lidar2base.inverse();
    geometry_msgs::PoseStamped temp_pose1,temp_pose2,temp_pose3;
//     temp_pose1.header.frame_id = "base_footprint";
//     temp_pose2.header.frame_id = "base_footprint";
//     temp_pose3.header.frame_id = "base_footprint";
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
  }
}

void ParsePoses::PubPath(const ros::WallTimerEvent& unused_timer_event)
{
  if(gps_path_.poses.size() == 0)
    return;
  path1_.header.frame_id = "map";
  path2_.header.frame_id = "map";
  gps_path_.header.frame_id = "map";
  gps_path_.header.stamp = ros::Time::now();
  path1_.header.stamp = ros::Time::now();
  path2_.header.stamp = ros::Time::now();
  path1_pub_.publish(path1_);
  path2_pub_.publish(path2_);
  gps_path_pub_.publish(gps_path_);
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
