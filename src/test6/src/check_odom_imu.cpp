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

#include "check_odom_imu.h"
#include <eigen_conversions/eigen_msg.h>
using namespace std;
CheckOdomImu::CheckOdomImu(const ros::NodeHandle& n):nh_(n),tfBuffer_(ros::Duration(10.)),
      tfListener_(tfBuffer_)
{
  odom_sub_ = nh_.subscribe("/emma_odom", 10, &CheckOdomImu::subOdom,this);
  imu_sub_ = nh_.subscribe("/jzhw/imu", 10, &CheckOdomImu::subImu,this);

    
}

CheckOdomImu::~CheckOdomImu()
{
  
}

void CheckOdomImu::subOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(fabs(msg->twist.twist.angular.z - last_imu_z) > 0.01)
  {
    cout << msg->twist.twist.angular.z <<", " << last_imu_z <<endl;
  }
}

void CheckOdomImu::subImu(const sensor_msgs::Imu::ConstPtr& msg)
{
  static bool imu_initial = false;
  static Eigen::Affine3d imu2base;
  if(!imu_initial){
    try
    {
      
      geometry_msgs::TransformStamped imu2base_tf = tfBuffer_.lookupTransform("base_footprint", "imu_link", ros::Time(0));
      tf::transformMsgToEigen(imu2base_tf.transform,imu2base);
      imu_initial = true;
//       break;
    }
    catch(tf::TransformException ex)
    {
      std::cout << "can not get tf"<<std::endl;
    }
  }
  
  Eigen::Vector3d origin_angle_velocity(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
  Eigen::Vector3d bias(0.0162261,0.0266933,0.0109272);
  Eigen::Vector3d correct_velocity = imu2base * (origin_angle_velocity - bias);
  last_imu_z = correct_velocity(2);
}

int main(int argc, char** argv)
{
  ros::init( argc,argv,"check_odom_imu");
  ros::NodeHandle n;
  CheckOdomImu h(n);
  ros::spin();
  return 1;
}
