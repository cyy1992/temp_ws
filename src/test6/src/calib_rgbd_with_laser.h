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

#ifndef CALIBRGBDWITHLASER_H
#define CALIBRGBDWITHLASER_H
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Eigen>

#include <std_srvs/Empty.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
typedef pcl::PointXYZ PointType;
class CalibRgbdWithLaser
{
public:
  CalibRgbdWithLaser(const ros::NodeHandle& n);
  void HandleDepthImage(const sensor_msgs::Image::ConstPtr& msg);
  void HandleLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg);
  bool SetState(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  void ComputeResult();
  
private:
  void FitPlane(std::vector<Eigen::Vector3f>& plane_points, float* plane12);
  void FitLine(std::vector<cv::Point2f>& fit_pts,cv::Mat& line);
  void cvFitPlane(const CvMat* points, float* plane);
  void pubPointCloud(const std::vector<Eigen::Vector3f>& plane_points);
  void ICPMatch(const pcl::PointCloud<PointType>::Ptr &cloud_target, 
                 const pcl::PointCloud<PointType>::Ptr &cloud_source, 
                 Eigen::Matrix4d &transform );
  void print4x4Matrix(const Eigen::Matrix4d & matrix);
  pcl::PointCloud<PointType>::Ptr ToPCL(const std::vector<Eigen::Vector3f>& points);
  ros::NodeHandle nh_;
  ros::Subscriber depth_img_sub_;
  ros::Subscriber laser_sub_;
  ros::Publisher cloud_pub_;
  ros::ServiceServer state_srv_;
  Eigen::Matrix3f K_inv;
  int offset_;
  int state_;
  std::vector<Eigen::Vector3f> plane_points_[3];
  std::vector<Eigen::Vector3f> laser_point_;
  int plane_cnt_, corner_cnt_, laser_cnt_;
};

#endif // CALIBRGBDWITHLASER_H
