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

#include "read_yaml.h"
using namespace std;

ReadYaml::ReadYaml(const std::string& path)
{
  YAML::Node params_root = YAML::LoadFile(path);params_root.Type();
  cout << params_root["emma2.1.zte-std"]["extrinsic"]["camera_arm"].Type() <<endl;
  cout << params_root["emma2.1.zte-std"]["extrinsic"]["camera_front"].Type() <<endl;
  string param = params_root["emma2.1.zte-std"]["extrinsic"]["camera_front1"].as<std::string>();
  cout << param <<endl;
  std::stringstream strStr;
  std::vector<double> temp(3);
  for (int k = 0; k < 3; k++)
    strStr >> temp[k];
}

ReadYaml::~ReadYaml()
{

}

int main(int argc, char** argv)
{
  ReadYaml yaml_test("/home/cyy/jz_project/catkin_ws/src/caliber/aprilslam/aprilslam/config/prior_calib_params.yaml");
  return 1;
}
