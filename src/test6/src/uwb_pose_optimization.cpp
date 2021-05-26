/*
 * Copyright 2021 <copyright holder> <email>
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

#include "uwb_pose_optimization.h"
#include <cartographer/transform/timestamped_transform.h>
#include <cartographer/transform/transform_interpolation_buffer.h>
using namespace std;
using namespace cartographer;
uwbPoseOptimization::uwbPoseOptimization(const std::string& path)
{
  loadCloud(path);
  wall_timer_ = nh_.createWallTimer(::ros::WallDuration(0.2),
                          &uwbPoseOptimization::display, this);
}

uwbPoseOptimization::~uwbPoseOptimization()
{

}
void uwbPoseOptimization::display(const ros::WallTimerEvent& unused_timer_event)
{
  
}

void uwbPoseOptimization::loadCloud(const std::string& filename)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  //*打开点云文件
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename + "/map.pcd", *cloud) == -1) {
      PCL_ERROR("Couldn't read file map.pcd\n");
  }
  
  ifstream fi;
  fi.open(filename+ "/uwb_datas.txt");
  if(fi.is_open())
  {
    while(fi.good())
    {
      int64 time;
      fi >> time;
      Eigen::Vector4d data;
      fi >> data(0) >> data(1) >> data(2)>>data(3);
      uwb_datas_[common::FromUniversal(time)].push_back(data);
    }
  }
  fi.close();
  cout << uwb_datas_.begin()->second[0] <<endl;
  ifstream fi2;
  fi2.open(filename+ "/node_poses.txt");
  if(fi2.is_open())
  {
    while(fi2.good())
    {
      int64 time;
      fi2 >> time;
      double px,py,pz,qw,qx,qy,qz;
      fi2 >> px >> py >> pz>>qw >> qx >> qy>>qz;
      node_datas_[common::FromUniversal(time)] = transform::Rigid3d({px,py,pz},{qw,qx,qy,qz});
    }
  }
  fi2.close();
  cout << node_datas_.begin()->second <<endl;
  
  for(auto it:uwb_datas_)
  {
    map<common::Time, cartographer::transform::Rigid3d>::iterator it1 = node_datas_.begin();
    map<common::Time, cartographer::transform::Rigid3d>::iterator it2 = node_datas_.begin();
    ++it2;
    common::Time cur_t = it.first;
    for(; it2 != node_datas_.end(); ++it1,++it2)
    {
      common::Time t11 = it1->first;
      common::Time t22 = it2->first;
      if(cur_t > t11 && cur_t < t22){
        cartographer::transform::Rigid3d transform2 = cartographer::transform::Interpolate(  
            cartographer::transform::TimestampedTransform{t11, it1->second},
            cartographer::transform::TimestampedTransform{t22, it2->second},
            cur_t).transform;
        uwb_poses_[cur_t] = transform2;
        break;
      }
    }
  }
  cout << uwb_datas_.size() << ", " <<  uwb_poses_.size() <<endl;
  cout << uwb_poses_.begin()->second <<endl;
  optimizePoses(uwb_datas_,uwb_poses_);
  
}
void uwbPoseOptimization::optimizePoses(
  const map< common::Time, vector< Eigen::Vector4d > >& uwb_datas,
  const map< common::Time, transform::Rigid3d >& uwb_poses)
{
  std::map<int, Eigen::Vector3d> ids;
  ceres::Problem problem;
  for(const auto& it:uwb_poses)
  {
    const common::Time time = it.first;
    transform::Rigid3d global_pose = it.second;
    vector<Eigen::Vector4d> uwb_data = uwb_datas.find(time)->second;

    for(Eigen::Vector4d node:uwb_data)
    {
      int id = static_cast<int>(node(0));
      if(ids.find(id) == ids.end())
        ids[id] = Eigen::Vector3d(0,0,0);
      
      problem.AddResidualBlock(
          DistanceMarkJzCostFunction::CreateAutoDiffCostFunction(node(1), global_pose.translation()),
          nullptr, ids[id].data());
    }
  }
  ceres::Solver::Options options;
  options.max_num_iterations = 2000;
  options.linear_solver_type = ceres::DENSE_QR;
  options.function_tolerance = 1e-8;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout <<"final_cost:  " <<summary.final_cost <<std::endl;
  for(auto id:ids)
    cout <<id.first <<", " << id.second <<endl;
  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "uwb_pose_optimization");
  string path = std::string(getenv("HOME"))+"/map/" + argv[1];
  uwbPoseOptimization pcl(path);
  ros::spin();
  return 1;
}