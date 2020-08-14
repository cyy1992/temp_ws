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

#include "wheel_odom_to_tf.h"

WheelOdomToTF::WheelOdomToTF(const ros::NodeHandle& n):nh_(n),initial_pose_(Eigen::Affine3d::Identity()),initialised_(false)
{
  sub_ = nh_.subscribe("/emma_odom",10,&WheelOdomToTF::HandleWheelOdomtry,this);
}

WheelOdomToTF::~WheelOdomToTF()
{

}


void WheelOdomToTF::HandleWheelOdomtry(const nav_msgs::Odometry::ConstPtr& msg)
{
  geometry_msgs::Pose cur_pose = msg->pose.pose;
  Eigen::Affine3d cur_pose_eigen;
  tf::poseMsgToEigen(cur_pose, cur_pose_eigen);
  if(!initialised_){
    initial_pose_ = cur_pose_eigen;
    initialised_ = true;
  }
  geometry_msgs::TransformStamped pub_msg;
  pub_msg.header.stamp = msg->header.stamp;
  pub_msg.header.frame_id = "odom";
  pub_msg.child_frame_id = "base_footprint";
  tf::transformEigenToMsg( initial_pose_.inverse() * cur_pose_eigen,pub_msg.transform);
  tf_broadcaster_.sendTransform(pub_msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "wheel_odom_to_tf");
  ros::NodeHandle n;
  WheelOdomToTF wtf(n);
  
  ros::spin();
  return 1;
}