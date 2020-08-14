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
using namespace std;
using namespace cv;
TfToPath::TfToPath(const ros::NodeHandle& nh,const std::string& frame,
           const std::string& sub_frame):nh_(nh),frame_(frame),sub_frame_(sub_frame),
           tfBuffer_(ros::Duration(20.)),tfListener_(tfBuffer_)
{
  string path_name = sub_frame+"_to_"+frame +"_path";
  path_pub_ = nh_.advertise<nav_msgs::Path>(path_name,1);
  timer_ = nh_.createWallTimer(ros::WallDuration(0.2),
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
  tf_path_.poses.push_back(temp_pose);
  tf_path_.header.frame_id = frame_;
  if(path_pub_.getNumSubscribers()){
    path_pub_.publish(tf_path_);
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