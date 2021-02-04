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
#include "unity.hpp"
#include <std_srvs/SetBool.h>
#include <nav_msgs/Path.h>
class ParsePoses:public Unity
{
public:
  ParsePoses(const ros::NodeHandle& n,const std::vector<std::string>& files);
  ~ParsePoses();
  void ReadPosesFromFile(const std::string& file1, std::map<long, std::vector<Rigid3d>>& poses);
  void ReadData(const std::vector<std::string>& files);
  void PubPath(const ros::WallTimerEvent& unused_timer_event);
  bool SetStep(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  bool SetSubmapShowId(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  void ParsePbstream(const std::string& pbstream_path,const std::string& pbstream2_path);
private:
  sensor_msgs::PointCloud2 ToPointCloud2Msg(const cartographer::sensor::PointCloud& cloud);
  ros::NodeHandle nh_;
//   ros::Publisher path1_pub_,path2_pub_,gps_path_pub_;
  ros::ServiceServer srv_;
  ros::WallTimer timer_pub_;
  std::map<long, std::vector<Rigid3d>> poses_with_time_;
  nav_msgs::Path path1_,path2_,gps_path_;
  std::vector<std::string> file_names_;
};

#endif // PARSEPOSES_H
