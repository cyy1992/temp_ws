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

#ifndef READJSONANDCHANGEPOSES_H
#define READJSONANDCHANGEPOSES_H
#include <string>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <Eigen/Eigen>

class ReadJsonAndChangePoses
{
public:
  ReadJsonAndChangePoses(const std::string& map_path);
  ~ReadJsonAndChangePoses();
  Eigen::Vector2i worldPose2ImagePose(const Eigen::Vector3d& world_pose);
  void getAndChangePoints(const std::string& map_path);
  void getMapInfo(const std::string& map_path);
  
  double maxes_[2];
  int size_of_map_[2];
  
  double resolution_;

};

#endif // READJSONANDCHANGEPOSES_H
