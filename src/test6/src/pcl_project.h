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

#ifndef PCLPROJECT_H
#define PCLPROJECT_H
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <math.h>
#include <algorithm>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <limlog/Log.h>

class PclProject
{
public:
  PclProject();
  ~PclProject();
  
  void HandleDepthPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
  
private:
  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_sub_;
  
  void ToPCL(const sensor_msgs::PointCloud2::ConstPtr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& temp_cloud);
  void GetPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                 std::vector<std::vector<float>> &Coffis, 
                 std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> &result_clouds,
                 int threshold);
  void PclShow(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  int FindGround(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds);
  void FitPlane(std::vector<Eigen::Vector3f>& plane_points, float* plane12);
  std::vector<Eigen::Vector3f> ToEigenPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};

#endif // PCLPROJECT_H
