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
    dbg(it.first);
    vector< double > average = getAverage(it.second);
    dbg(average);
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
  double px = 0,py = 0,pz = 0,qw = 0, qx = 0,qy = 0, qz =0;
  for(auto param:params)
  {
    px += param.px;
    py += param.py;
    
    pz += param.pz;
    qw += param.q.w();
    qx += param.q.x();
    qy += param.q.y();
    qz += param.q.z();
  }
  vector< double > average;
  average.push_back(px/params.size());
  average.push_back(py/params.size());
  average.push_back(pz/params.size());
  average.push_back(qw/params.size());
  average.push_back(qx/params.size());
  average.push_back(qy/params.size());
  average.push_back(qz/params.size());
  return average;
}

int main(int argc, char *argv[]) 
{
  const string file = "/home/cyy/Documents/json_files";
  ReadJson json(file);
  
  return 0;
}