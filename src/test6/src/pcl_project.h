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
class PclProject
{
public:
  typedef pcl::PointNormal PointT;
  PclProject();
  ~PclProject();
  
  void HandleDepthPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void HandlePointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
private:
  void read_pcd(std::string pcd_path,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  Eigen::Matrix4d PointToPlaneIcpMatcher (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2);
  void ToPCL(const sensor_msgs::PointCloud2::ConstPtr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& temp_cloud);
  void GetPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                 std::vector<std::vector<float>> &Coffis, 
                 std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> &result_clouds,
                 const unsigned int& threshold);
  void PclShow(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  int FindGround(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds);
  std::vector<int> FindWalls(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds, const int& ground_index);
  void cvFitPlane(const CvMat* points, float* plane);
  void FitLine(std::vector<cv::Point2f>& fit_pts,cv::Mat& line);
  void FitPlane(std::vector<Eigen::Vector3f>& plane_points, float* plane12);
  void GetCamToPlane(const cv::Point3f& vector_x, const float* plane_index, cv::Mat& rotation, cv::Mat& translation,const bool& is_reverse);
  std::vector<Eigen::Vector3f> ToEigenPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void display(const ::ros::WallTimerEvent& unused_timer_event);
  void pubPointCloud2(const ros::Publisher& pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_sub_, cloud_sub_;
  ros::Publisher src_pub_,tgt_pub_,final_pub_,transformed_pub_;
  float max_dst_error_plane_,max_dst_error_line_;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_,src_cloud_,final_cloud_,transformed_cloud_;
  ros::WallTimer wall_timer_;
};

#endif // PCLPROJECT_H
