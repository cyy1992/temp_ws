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
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace std;

ReadYaml::ReadYaml(const std::string& path)
{
  writeYaml("/home/cyy/test.yaml");
//   YAML::Node params_root = YAML::LoadFile(path);params_root.Type();
//   cout << params_root["emma2.1.zte-std"]["extrinsic"]["camera_arm"].Type() <<endl;
//   cout << params_root["emma2.1.zte-std"]["extrinsic"]["camera_front"].Type() <<endl;
//   string param = params_root["emma2.1.zte-std"]["extrinsic"]["camera_front1"].as<std::string>();
//   cout << param <<endl;
//   std::stringstream strStr;
//   std::vector<double> temp(3);
//   for (int k = 0; k < 3; k++)
//     strStr >> temp[k];
}

ReadYaml::~ReadYaml()
{

}
string vec2string(vector<float> &feature ){
  string f_data;
  std::stringstream ss;

  cout << "v.size() = " << feature.size() << endl;
  for(size_t i = 0; i < feature.size(); ++i)
  {
    if(i != 0)
      ss << " ";
    ss << feature[i];
  }
  
  f_data = ss.str();
  cout << "ss = " << ss.str() << endl;
  return f_data;
}

vector<float> string_to_vector_float(const std::string& str){
//   string str = "-0.0647031 -0.0206785 -0.0439941 -0.0323903 -0.0139745 -0.0208939 0.0289745 0.0277657 -0.0807781 -0.0288254";
  stringstream ss(str);
  string buf;
  vector<float> vec;
  //vector<float> vec(1024);//可以初始化时指定vecrot容器大小
  while(ss >> buf)
    vec.push_back(atof(buf.c_str()));

  return vec;
}

void ReadYaml::writeYaml(const string& path)
{
  ofstream of;
  of.open(path);
  
  YAML::Node yaml_node;
  
  for(int i =0; i < 2; i++)
  {
    YAML::Node sub_node;
    sub_node["test"] = 1;
    sub_node["id"] = 2;
    std::vector<float> vec;
    vec.push_back(1.0);
    vec.push_back(2.1);
    string data = vec2string(vec);
    
    vector<float> fl = string_to_vector_float(data);
    cout << fl[0] <<"," <<fl[1] <<endl;
    sub_node["data"] = data;
    
    yaml_node["laser_data"].push_back(sub_node);
    cout << "here"<<endl;
  }
  std::cout << yaml_node <<endl;
  of << yaml_node ;
  of.close();
  
  YAML::Node node_read = YAML::LoadFile(path);
  
  cout << node_read.size() <<endl;
  cout << node_read["laser_data"].size() <<endl;
  for(auto it = node_read["laser_data"].begin(); it != node_read["laser_data"].end();it++)
  {
    YAML::Node temp = *it;
    cout << temp["data"] << endl;
  }
  
//   cv::FileStorage fs_write(path, cv::FileStorage::WRITE);
// 
//   time_t rawtime;
//   time(&rawtime);
//   fs_write << "Time" << asctime(localtime(&rawtime));
//   
//   std::vector<int> image_size = {1280, 800};
//   fs_write << "imageSize" << image_size; 
// 
//   cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 190, 0, 640, 
//                                                      0, 190, 400, 
//                                                      0, 0, 1);
//   
//   cv::Mat distort_coefficient = (cv::Mat_<double>(5, 1) << 0.1, 0.01, -0.001, 0, 0);
//   std::vector<int> vec;
//   vec.push_back(1);
//   vec.push_back(2);
//   fs_write << "cameraMatrix" << vec << "distCoeffs" << distort_coefficient;
//     
//   fs_write.release();

 

  
}

int main(int argc, char** argv)
{
  ReadYaml yaml_test("/home/cyy/jz_project/catkin_ws/src/caliber/aprilslam/aprilslam/config/prior_calib_params.yaml");
  return 1;
}
