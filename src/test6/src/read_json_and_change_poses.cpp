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

#include "read_json_and_change_poses.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>

using namespace std;
ReadJsonAndChangePoses::ReadJsonAndChangePoses(const string& map_path)
{
  getMapInfo(map_path);
  getAndChangePoints(map_path);
}

ReadJsonAndChangePoses::~ReadJsonAndChangePoses()
{

}
Eigen::Vector2i ReadJsonAndChangePoses::worldPose2ImagePose(const Eigen::Vector3d& world_pose)
{
  Eigen::Vector2i img_pose;
  Eigen::Vector3d box(maxes_[0],maxes_[1] ,0);
  cout << "world_pose:" << world_pose[0] / resolution_<<", " << world_pose[1] / resolution_<<endl;
  img_pose(1) =   (box[0]- world_pose(0)) / resolution_ ;
  img_pose(0) =   (box[1]- world_pose(1)) / resolution_;
  return img_pose;
}

void ReadJsonAndChangePoses::getMapInfo(const string& map_path)
{
  {
    boost::property_tree::ptree pt1;
//     string submap_filepath = map_path + "/show_map_data.xml";
    boost::property_tree::read_xml(map_path + "/show_map_data.xml", pt1);
    if (pt1.empty())
    {
      return;
    }
    std::string data;
    
    resolution_ = pt1.get<double>("resolution");
    data = pt1.get<std::string>("max_box");
    sscanf(data.c_str(), "%lf %lf", &maxes_[0], &maxes_[1]);
//     data = pt1.get<std::string>("probability_grid.num_cells");
//     sscanf(data.c_str(), "%d %d", &size_of_map_[0], &size_of_map_[1]);
    cout << "box:" << maxes_[0] / resolution_<<", " << maxes_[1] / resolution_<<endl;
//     cout << "size_of_map_:" << size_of_map_[0]<<", " << size_of_map_[1]<<endl;
    
    Eigen::Quaterniond q(0,-0.707107,0.707107,0.000000);
    cout << q.toRotationMatrix() <<endl;
  }
}

void ReadJsonAndChangePoses::getAndChangePoints(const std::string& map_path)
{

  
  Json::Reader reader;
  Json::Value root;
  ifstream in(map_path+ "/info_ImageRefer.json", ios::in);
  
  if( !in.is_open() )  
  { 
    cout << "Error opening file\n"; 
  }
  if(reader.parse(in,root, true))
  {
    for(auto& node:root["nodes"])
    {
      Eigen::Vector3d position;
      position(0) = node["worldPose"]["position"]["x"].asDouble();
      position(1) = node["worldPose"]["position"]["y"].asDouble();
      position(2) = node["worldPose"]["position"]["z"].asDouble();
      Eigen::Vector2i img_position = worldPose2ImagePose(position);
      node["pose"]["x"] = img_position(0);
      node["pose"]["y"] = img_position(1);
      Eigen::Quaterniond q_temp(node["worldPose"]["orientation"]["w"].asDouble(),
                                node["worldPose"]["orientation"]["x"].asDouble(),
                                node["worldPose"]["orientation"]["y"].asDouble(),
                                node["worldPose"]["orientation"]["z"].asDouble());
      const Eigen::Matrix<double, 3, 1> direction = q_temp * Eigen::Matrix<double, 3, 1>::UnitX();
      node["pose"]["theta"] =M_PI /2 + atan2(direction.y(), direction.x());
    }
  }
  Json::StyledWriter sw;
  ofstream os;
  os.open(map_path+ "/info_ImageRefer.json");
  os << sw.write(root);
  os.close();
  cout << "success!";
}

int main(int argc, char** argv)
{
  string path = std::string(getenv("HOME"))+"/map/" + argv[1];
  cout << "path:" << path<<endl;
  ReadJsonAndChangePoses read(path);
  return 1;
}
