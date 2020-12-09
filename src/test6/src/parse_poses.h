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

#ifndef PARSEPOSES_H
#define PARSEPOSES_H
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Path.h>
class ParsePoses
{
struct Rigid3d
  {
    Eigen::Vector3d t;
    Eigen::Quaterniond q;
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
public:
  ParsePoses(const ros::NodeHandle& n);
  ~ParsePoses();
  void ReadData(const std::string& file1, const std::string& file2);
  void PubPath(const ros::WallTimerEvent& unused_timer_event);
  bool SetStep(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
private:
  ros::NodeHandle nh_;
  ros::Publisher path1_pub_,path2_pub_,gps_path_pub_;
  ros::ServiceServer srv_;
  ros::WallTimer timer_pub_;
  std::map<long, std::vector<Rigid3d>> poses_with_time_;
  nav_msgs::Path path1_,path2_,gps_path_;
};

#endif // PARSEPOSES_H
