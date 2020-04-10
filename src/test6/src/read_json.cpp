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

#include "read_json.h"
#include <memory.h>
#include <dirent.h>
#include <dbg.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

using namespace std;

ReadJson::ReadJson(const std::string& floder_path)
{
  std::vector<std::string> files = getFilesList(floder_path);

//   dbg(files);
  for(auto file:files)
  {
    cout << file <<endl;
    readFromFile(file);
  }
  
  for(auto it: params_with_type_)
  {
    cout << it.first << endl;
    vector< double > average = getAverage(it.second);
    Params temp(Eigen::Vector3d(average[0],average[1],average[2]),Eigen::Vector3d(average[5],average[4],average[3]));
    cout <<"average result: \n" <<temp.t.transpose() << " " << temp.q.coeffs().transpose() <<endl;
    cout << temp.px <<" " << temp.py<<" " << temp.pz<<" " << temp.q.x()<<" " << temp.q.y()<<" " << temp.q.z() <<" " <<temp.q.w()<<endl;
  }
}

std::vector<std::string> ReadJson::getFilesList(const std::string& dirpath)
{
  DIR *dir = opendir(dirpath.c_str());
  if (dir == NULL)
  {
    cout << "opendir error" << endl;
  }
  vector<string> allPath;
  struct dirent *entry;
  while ((entry = readdir(dir)) != NULL)
  {
    if (entry->d_type == DT_DIR){//It's dir
      if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
          continue;
      string dirNew = dirpath + "/" + entry->d_name;
      vector<string> tempPath = getFilesList(dirNew);
      allPath.insert(allPath.end(), tempPath.begin(), tempPath.end());

    }else {
      //cout << "name = " << entry->d_name << ", len = " << entry->d_reclen << ", entry->d_type = " << (int)entry->d_type << endl;
      string name = entry->d_name;
      string imgdir = dirpath +"/"+ name;
      //sprintf("%s",imgdir.c_str());
      allPath.push_back(imgdir);
    }

  }
  closedir(dir);
  //system("pause");
  return allPath;
}

bool ReadJson::readFromFile(const string& file_name)
{
  Json::Reader reader;
  Json::Value root;
  ifstream in(file_name, ios::binary);
  if( !in.is_open() )  
  { 
    cout << "Error opening file\n"; 
    return false; 
  }

 
  if(reader.parse(in,root))
  {
  //读取根节点信息
    Json::Value camera = root["Camera"];
    Json::Value laser = root["Laser"];
    cout << camera.size() << endl;
    for(auto sub_cam : camera)
    {
      string cam_type = sub_cam["name"].toStyledString();
//       
        
      Json::Value calib_param = sub_cam["calib_param"];
      Eigen::Vector3d t,eul;
      t(0) = calib_param["px"].asFloat();
      t(1) = calib_param["py"].asFloat();
      t(2) = calib_param["pz"].asFloat();
      eul(2) = calib_param["roll"].asFloat();
      eul(1) = calib_param["pitch"].asFloat();
      eul(0) = calib_param["yaw"].asFloat();
      Params param(t,eul);
      if(params_with_type_.find(cam_type) == params_with_type_.end())
      {
        vector<Params> params;
        params.push_back(param);
        params_with_type_[cam_type] = params;
      }
      else
        params_with_type_[cam_type].push_back(param);
    }
    
    for(auto sub_cam : laser)
    {
      string cam_type = sub_cam["name"].toStyledString();
//       
        
      Json::Value calib_param = sub_cam["calib_param"];
      Eigen::Vector3d t,eul;
      t(0) = calib_param["px"].asFloat();
      t(1) = calib_param["py"].asFloat();
      t(2) = calib_param["pz"].asFloat();
      eul(2) = calib_param["roll"].asFloat();
      eul(1) = calib_param["pitch"].asFloat();
      eul(0) = calib_param["yaw"].asFloat();
      Params param(t,eul);
      if(params_with_type_.find(cam_type) == params_with_type_.end())
      {
        vector<Params> params;
        params.push_back(param);
        params_with_type_[cam_type] = params;
      }
      else
        params_with_type_[cam_type].push_back(param);
    }
  }
  in.close();
  return true;
}

vector< double > ReadJson::getAverage(const vector< ReadJson::Params >& params)
{
  double px = 0,py = 0,pz = 0;
//   double qw = 0, qx = 0,qy = 0, qz =0;
  double roll = 0, pitch = 0,yaw = 0;
  for(auto param:params)
  {
    px += param.px;
    py += param.py;
    
    pz += param.pz;
    roll += param.roll;
    pitch += param.pitch;
    yaw += param.yaw;
    cout << param.px <<"  \t  " << param.py<<"  \t  " << param.pz<<"  \t  " << param.roll<<"  \t  " << param.pitch<<"  \t  " << param.yaw <<endl;
//     qw += param.q.w();
//     qx += param.q.x();
//     qy += param.q.y();
//     qz += param.q.z();
  }
  vector< double > average;
  average.push_back(px/params.size());
  average.push_back(py/params.size());
  average.push_back(pz/params.size());
  average.push_back(roll/params.size());
  average.push_back(pitch/params.size());
  average.push_back(yaw/params.size());
//   average.push_back(qw/params.size());
//   average.push_back(qx/params.size());
//   average.push_back(qy/params.size());
//   average.push_back(qz/params.size());
  return average;
}


void ReadJson::checkResult(const Eigen::Vector3d& position, 
                             const Eigen::Quaterniond& rotation)
{
  std::string yaml_path = ros::package::getPath("aprilslam") + "/config/prior_calib_params.yaml";
  YAML::Node params_root = YAML::LoadFile(yaml_path);
  string sub_robot_type; 
  string robot_type = "emma2";
  ::ros::param::get("/agent/robot_assembler_type",robot_type);
  if(!::ros::param::get("/agent/robot_assembler_sub_type",sub_robot_type))
  {
    cout << "Can't get param of /agent/robot_assembler_sub_type!" <<endl;
  }
  string type = robot_type + "." + sub_robot_type;
  cout << "yaml_path: " <<yaml_path <<endl;
  cout << "type: " <<type <<endl;
  string cam_type = "camera_arm";
  if(params_root[type.c_str()]["extrinsic"][cam_type.c_str()].Type() == 0){
    cout << "Warning: " << type << " undefined!" << endl;
    return ;
  }
  cout << params_root[type.c_str()]["extrinsic"][cam_type.c_str()].Type();
  string prior_extrinsic_str = params_root[type.c_str()]["extrinsic"][cam_type.c_str()].as<string>();
  cout << "prior_extrinsic_str: " <<prior_extrinsic_str <<endl;
  std::stringstream strStr;
  strStr << prior_extrinsic_str;
  std::vector<double> temp(7);
  for (int k = 0; k < 7; k++)
    strStr >> temp[k];
  Eigen::Vector3d prior_position(temp[0],temp[1],temp[2]);
  Eigen::Quaterniond prior(temp[6],temp[3],temp[4],temp[5]);
  
  Eigen::Vector3d err_position = prior_position - position;
  Eigen::Quaterniond err_q = prior.conjugate() * rotation ;
  double err_position_norm = err_position.norm();
//   Eigen::Vector3d err_eul = err_q.toRotationMatrix().eulerAngles();
  cout << "prior_position: " <<prior_position.transpose() <<endl;
  if(err_position_norm > 0.01)
  {
    cout << "\033[31m ERROR: err position norm is bigger than 0.07, it is " << err_position_norm << "\033[37m"<<endl; 
  }
  
  if(fabs(err_q.w() -1.0) > 0.1 ||  fabs(err_q.x()> 0.1) || fabs(err_q.y()> 0.1) || fabs(err_q.z()> 0.1))
  {
    cout << "\033[31m ERROR: err rotation is too bigger! err_eul is " <<err_q.toRotationMatrix().eulerAngles(2,1,0).transpose() << "\033[37m"<<endl; 
  }
  exit(1);
}
int main(int argc, char *argv[]) 
{
  ros::init(argc, argv, "read_json");
  ros::NodeHandle nh;
  const string file = "/home/cyy/Documents/tmp_tm_json";
  ReadJson json(file);
  json.checkResult(Eigen::Vector3d(0,0,0), Eigen::Quaterniond(1,0,0,0));
  return 0;
}