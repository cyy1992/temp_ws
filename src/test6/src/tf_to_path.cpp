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

#include "tf_to_path.h"
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
using namespace std;
using namespace cv;
TfToPath::TfToPath(const ros::NodeHandle& nh,const std::string& frame,
           const std::string& sub_frame):nh_(nh),frame_(frame),sub_frame_(sub_frame),
           tfBuffer_(ros::Duration(20.)),tfListener_(tfBuffer_),initial_(true)
{
  string path_name = sub_frame+"_to_"+frame +"_path";
  string points_name = sub_frame+"_to_"+frame +"_pose";
  path_pub_ = nh_.advertise<nav_msgs::Path>(path_name,10);
  points_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(points_name,10);
  timer_ = nh_.createWallTimer(ros::WallDuration(0.02),
                               &TfToPath::saveTF,this);
}

void TfToPath::saveTF(const ros::WallTimerEvent& unused_timer_event)
{
  geometry_msgs::TransformStamped base2map;
  try
  {
    base2map = tfBuffer_.lookupTransform(frame_.c_str(), sub_frame_.c_str(), ros::Time(0));
  }
  catch (tf::TransformException ex)
  {
    return;
  }
  geometry_msgs::PoseStamped temp_pose;
  temp_pose.header.stamp = ros::Time::now();
  temp_pose.pose.position.x = base2map.transform.translation.x;
  temp_pose.pose.position.y = base2map.transform.translation.y;
  temp_pose.pose.position.z = base2map.transform.translation.z;
  Eigen::Affine3d base2map_eigen = Eigen::Affine3d::Identity();
  tf::transformMsgToEigen(base2map.transform,base2map_eigen);
  if(initial_)
  {
    initial_ = false;
    first_pose_ = base2map_eigen;
  }
  tf_path_.poses.push_back(temp_pose);
  tf_path_.header.frame_id = frame_;
  if(path_pub_.getNumSubscribers()){
    path_pub_.publish(tf_path_);
  }
  if(points_pub_.getNumSubscribers())
  {
    Eigen::Affine3d relative_pose = first_pose_.inverse() * base2map_eigen;
    geometry_msgs::PoseStamped temp_pose2;
    temp_pose2.header = temp_pose.header;
    tf::poseEigenToMsg(relative_pose,temp_pose2.pose);
    points_pub_.publish(temp_pose2);
  }
}
int main(int argc, char** argv)
{
  string frame,sub_frame;
  frame = argv[1];
  sub_frame = argv[2];
  string node_name = sub_frame+"_to_"+frame +"_node";
  ros::init(argc,argv,node_name);
  ros::NodeHandle n;
  TfToPath tf2path(n,frame,sub_frame);
  ros::spin();
  return 1;
}