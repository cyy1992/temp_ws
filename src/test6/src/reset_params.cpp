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

#include <ros/ros.h>
#include <algorithm>
#include <iostream>
#include <Eigen/Eigen>
#include <math.h>
using namespace std;

Eigen::Quaterniond ToEigen(const Eigen::Vector3d& eulur)
{
  Eigen::Matrix3d matrix_tmp;
  matrix_tmp = (Eigen::AngleAxisd(eulur[0],Eigen::Vector3d::UnitZ()) 
        * Eigen::AngleAxisd(eulur[1],Eigen::Vector3d::UnitY()) 
        * Eigen::AngleAxisd(eulur[2],Eigen::Vector3d::UnitX())).toRotationMatrix();
  Eigen::Quaterniond q(matrix_tmp);
  return q;
}


int main(int argc, char** argv)
{
  
  ros::init(argc,argv,"reset_params");
  ros::NodeHandle n;
  string base2cam_yaw_s;
  base2cam_yaw_s = argv[1];
  double base2cam_yaw = stod(base2cam_yaw_s);
  Eigen::Vector3d base2cam_p;
  Eigen::Vector3d base2cam_eul;
  ::ros::param::get("/jzhw/calib/camera/down/px_base2cam", base2cam_p(0));
  ::ros::param::get("/jzhw/calib/camera/down/py_base2cam", base2cam_p(1));
  ::ros::param::get("/jzhw/calib/camera/down/pz_base2cam", base2cam_p(2));
  ::ros::param::get("/jzhw/calib/camera/down/yaw_base2cam", base2cam_eul(0));
  ::ros::param::get("/jzhw/calib/camera/down/pitch_base2cam", base2cam_eul(1));
  ::ros::param::get("/jzhw/calib/camera/down/roll_base2cam", base2cam_eul(2));
  cout << "base2cam(x,y,z,yaw,pitch,roll): \n" <<  base2cam_p.transpose() <<"\n" << base2cam_eul.transpose() <<endl;
  base2cam_eul(0) = base2cam_yaw;
  Eigen::Quaterniond base2cam_q = ToEigen(base2cam_eul);
  
  Eigen::Quaterniond cam2base_q = base2cam_q.conjugate();
  Eigen::Vector3d cam2base_eul = cam2base_q.toRotationMatrix().eulerAngles(2,1,0);
  Eigen::Vector3d cam2base_p = -(cam2base_q * base2cam_p) / 1000;
  
  ::ros::param::set("/jzhw/calib/camera/down/px", cam2base_p(0));
  ::ros::param::set("/jzhw/calib/camera/down/py", cam2base_p(1));
  ::ros::param::set("/jzhw/calib/camera/down/pz", cam2base_p(2));
  ::ros::param::set("/jzhw/calib/camera/down/yaw", cam2base_eul(0));
  ::ros::param::set("/jzhw/calib/camera/down/pitch", cam2base_eul(1));
  ::ros::param::set("/jzhw/calib/camera/down/roll", cam2base_eul(2));
  cout << "cam2base(x,y,z,yaw,pitch,roll): \n" <<  cam2base_p.transpose() <<"\n" << cam2base_eul.transpose() <<endl;
  string  cmd_set = "rosservice call /jzhw/param_update \"request: 'down'\" ";
  int status = system(cmd_set.c_str()); 
  return 1;
}