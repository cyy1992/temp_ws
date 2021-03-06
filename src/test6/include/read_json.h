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
    
    Eigen::Quaterniond q_temp = q.normalized();
    q = q_temp;
    double aSinInput = -2 * (q_temp.x() * q_temp.z() - q_temp.w() * q_temp.y());
    if(aSinInput > 1)
      aSinInput = 1;
    Eigen::Vector3d euler;
    euler(0) = atan2(2*(q_temp.x()*q_temp.y()+q_temp.w()*q_temp.z()), 
                      q_temp.w() * q_temp.w() + q_temp.x() * q_temp.x()- q_temp.y()*q_temp.y() - q_temp.z() *q_temp.z());
    euler(1) = asin(aSinInput);
    euler(2) = atan2(2*(q_temp.y() *q_temp.z()+q_temp.w()*q_temp.x()), 
                      q_temp.w() * q_temp.w() - q_temp.x() * q_temp.x() - q_temp.y()*q_temp.y() + q_temp.z() *q_temp.z() );
    for(int i =0 ; i < 3; i ++)
    {
      if(euler(i) < -2.9)
        euler(i) = euler(i) + 2 * M_PI;
    }
    px = t(0);
    py = t(1);
    pz = t(2);
    roll = euler(2);
    pitch = euler(1);
    yaw = euler(0);
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
public:
  ReadJson(const std::string& floder_path);
  std::vector<std::string> getFilesList(const std::string& dirpath);
  std::vector<double> getAverage(const std::vector<Params>& params);
  void checkResult(const Eigen::Vector3d& position, 
                    const Eigen::Quaterniond& rotation);
private:
  bool readFromFile(const std::string& file_name);
  
  std::map<std::string,std::vector<Params>> params_with_type_;
};

#endif // READJSON_H
