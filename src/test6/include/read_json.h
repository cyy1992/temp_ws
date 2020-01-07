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

#ifndef READJSON_H
#define READJSON_H
#include <string>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <Eigen/Eigen>
class ReadJson
{
struct Params{
  double px,py,pz,roll,pitch,yaw;
  Eigen::Vector3d t;
  Eigen::Quaterniond q;
  Params():t(Eigen::Vector3d(0,0,0)),q(Eigen::Quaterniond::Identity()){};
  Params(Eigen::Vector3d _t, Eigen::Vector3d _eul)
  {
    t = _t;
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(_eul(2),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(_eul(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(_eul(0),Eigen::Vector3d::UnitZ())); 
    q = yawAngle*pitchAngle * rollAngle;
    
    px = t(0);
    py = t(1);
    pz = t(2);
    roll = _eul(2);
    pitch = _eul(1);
    yaw = _eul(0);
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
public:
  ReadJson(const std::string& floder_path);
  std::vector<std::string> getFilesList(const std::string& dirpath);
  std::vector<double> getAverage(const std::vector<Params>& params);
private:
  bool readFromFile(const std::string& file_name);
  std::map<std::string,std::vector<Params>> params_with_type_;
};

#endif // READJSON_H
