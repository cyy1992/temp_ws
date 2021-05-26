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

#ifndef UWBPOSEOPTIMIZATION_H
#define UWBPOSEOPTIMIZATION_H
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <limlog/Log.h>

#include "transform.h"
#include <cartographer/common/time.h>
#include <cartographer/transform/rigid_transform.h>
#include <ceres/ceres.h>
class uwbPoseOptimization
{
public:
  uwbPoseOptimization(const std::string& path);
  ~uwbPoseOptimization();
  void loadCloud(const std::string& filename);
private:
  void optimizePoses(
    const std::map<cartographer::common::Time, std::vector<Eigen::Vector4d>>& uwb_datas,
    const std::map<cartographer::common::Time, cartographer::transform::Rigid3d>& uwb_poses);
  void display(const ::ros::WallTimerEvent& unused_timer_event);
  ros::NodeHandle nh_;
  sensor_msgs::PointCloud2 all_cloud_;
  std::map<cartographer::common::Time, std::vector<Eigen::Vector4d>> uwb_datas_;
  std::map<cartographer::common::Time, cartographer::transform::Rigid3d> node_datas_;
  std::map<cartographer::common::Time, cartographer::transform::Rigid3d> uwb_poses_;
  ros::WallTimer wall_timer_;
  ros::Publisher cloud_pub_,uwb_pub_;
  
};

class DistanceMarkJzCostFunction {
public:

  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double& observation, 
      const Eigen::Vector3d& sensor2bpre) {
    return new ceres::AutoDiffCostFunction<
        DistanceMarkJzCostFunction, 1 /* residuals */,
        3 /* landmark translation variables */>(
        new DistanceMarkJzCostFunction(observation, sensor2bpre));
  }

  template <typename T>
  bool operator()(const T* const landmark_translation, T* const e) const {
    const Eigen::Matrix<T, 3, 1> global_t(T(global_pose_t_(0)),
                                                T(global_pose_t_(1)),
                                                T(global_pose_t_(2)));
    
    e[0] = (global_t[0] - landmark_translation[0]) * (global_t[0] - landmark_translation[0]) + 
                    (global_t[1] - landmark_translation[1]) * (global_t[1] - landmark_translation[1]) + 
                    (global_t[2] - landmark_translation[2]) * (global_t[2] - landmark_translation[2]) - 
                    T(sensor2mark_distance_);//,
    return true;
  }

private:
  DistanceMarkJzCostFunction(const double observation,
                         const Eigen::Vector3d& global_pose_t)
      : sensor2mark_distance_(observation * observation),
        global_pose_t_(global_pose_t){}
  const double sensor2mark_distance_;
  const Eigen::Vector3d global_pose_t_;
};
#endif // UWBPOSEOPTIMIZATION_H