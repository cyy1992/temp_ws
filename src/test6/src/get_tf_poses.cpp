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

#include "get_tf_poses.h"
#include <cartographer_ros_msgs/SetSwitch.h>
#include <Eigen/Eigen>
#include <vtr_msgs/SetInitialPose.h>
using namespace std;
GetTfPoses::GetTfPoses(const ros::NodeHandle& n):nh_(n), tfBuffer_(ros::Duration(600.)), tfListener_(tfBuffer_), cnt_(0), 
    localization_state_(0),need_change_state_(true),laser_localization_yaw_(0), cam_loclaization_yaw_(0)
{
  switch_localization_client_ = nh_.serviceClient<cartographer_ros_msgs::SetSwitch>("/mark_localization/set_mark_switch");
  call_initial_client_ = nh_.serviceClient<vtr_msgs::SetInitialPose>("/mark_localization/initial_pose");
  timer_pub_ = nh_.createWallTimer(ros::WallDuration(0.2),
                                            &GetTfPoses::GetTfTimer,this);
}

GetTfPoses::~GetTfPoses()
{
  
}

void GetTfPoses::GetTfTimer(const ros::WallTimerEvent& unused_timer_event)
{
  if(need_change_state_){
    
    cartographer_ros_msgs::SetSwitch srv;
    cout << "Change state: " << localization_state_ <<endl;
    if(localization_state_ == 0)
    {
      geometry_msgs::TransformStamped base2map;
      try
      {
        base2map =
            tfBuffer_.lookupTransform("map", "base_footprint", ros::Time(0));
        cnt_++;
      }
      catch (tf2::TransformException &ex)
      {
        return;
      }
      srv.request.flag = false;
      srv.request.type = "Visualmarks";
      switch_localization_client_.call(srv);
      
      srv.request.flag = true;
      srv.request.type = "LaserScanOdom";
      switch_localization_client_.call(srv);
      
      vtr_msgs::SetInitialPose initial_pose_srv;
      initial_pose_srv.request.base2map.header.stamp = ros::Time::now();
      initial_pose_srv.request.base2map.pose.pose.position.x = base2map.transform.translation.x;
      initial_pose_srv.request.base2map.pose.pose.position.y = base2map.transform.translation.y;
      initial_pose_srv.request.base2map.pose.pose.position.z = base2map.transform.translation.z;
      
      initial_pose_srv.request.base2map.pose.pose.orientation.x = base2map.transform.rotation.x;
      initial_pose_srv.request.base2map.pose.pose.orientation.y = base2map.transform.rotation.y;
      initial_pose_srv.request.base2map.pose.pose.orientation.z = base2map.transform.rotation.z;
      initial_pose_srv.request.base2map.pose.pose.orientation.w = base2map.transform.rotation.w;
      call_initial_client_.call(initial_pose_srv);
      localization_state_++;
    }
    else if(localization_state_ == 1)
    {
      srv.request.flag = true;
      srv.request.type = "Visualmarks";
      switch_localization_client_.call(srv);
      
      srv.request.flag = false;
      srv.request.type = "LaserScanOdom";
      switch_localization_client_.call(srv);
      localization_state_++;
    }
    else{
      float laser_yaw = laser_localization_yaw_ / cnt_;
      float cam_yaw = cam_loclaization_yaw_ / cnt_;
      cout <<"\033[33m" <<  "laser_yaw: " << laser_yaw <<endl;
      cout <<"\033[33m" << "cam_yaw: " << cam_yaw <<endl;
      cout <<"\033[33m" << "laser_yaw - cam_yaw: " << (laser_yaw - cam_yaw) <<endl;
      exit(1);
    }
    cnt_ = 0;
    need_change_state_ = false;
    ros::Duration(3).sleep();
  }
  else
  {
    if(cnt_ == 9)
      need_change_state_ = true;
    geometry_msgs::TransformStamped base2map;
    try
    {
      base2map =
          tfBuffer_.lookupTransform("map", "base_footprint", ros::Time(0));
      cnt_++;
    }
    catch (tf2::TransformException &ex)
    {
      return;
    }
    float yaw;
    const Eigen::Matrix<float, 3, 1> direction =
      Eigen::Quaternionf(base2map.transform.rotation.w, 
                         base2map.transform.rotation.x, 
                         base2map.transform.rotation.y, 
                         base2map.transform.rotation.z) 
        * Eigen::Matrix<float, 3, 1>::UnitX();
    yaw = atan2(direction.y(), direction.x());
    cout << "localization_state_: " << localization_state_ <<"\t yaw: " <<yaw <<endl;

    if(localization_state_ == 1)
    {
      laser_localization_yaw_ += yaw;
    }
    else if(localization_state_ == 2)
    {
      cam_loclaization_yaw_ += yaw;
    }
    
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_tf_poses");
  ros::NodeHandle n;
  GetTfPoses temp(n);
  ros::spin();
  
  return 1;
}
