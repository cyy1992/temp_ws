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

#include "pcl_project.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace limlog;
using namespace cv;
PclProject::PclProject()
{
  point_cloud_sub_ = nh_.subscribe("/front/depth/color/points",1,&PclProject::HandleDepthPointCloud,this);
}

PclProject::~PclProject()
{

}

void PclProject::ToPCL(const sensor_msgs::PointCloud2::ConstPtr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& temp_cloud)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
}

void PclProject::PclShow(const pcl::PointCloud< pcl::PointXYZ >::Ptr& cloud)
{
  pcl::visualization::PCLVisualizer p;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (cloud, abs(rand() % 255), abs(rand() % 255), abs(rand() % 255));
  p.addPointCloud(cloud, tgt_h, "target" + to_string(abs(rand())));
  p.spinOnce();
}

void PclProject::GetPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                           std::vector<std::vector<float>> &Coffis, 
                           std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> &result_clouds,
                           int threshold)
{
  while (cloud->points.size() >= threshold){
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); 
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0){
      PCL_ERROR("Could not estimate a planar model for the given dataset.");
      return;
    }
    if (inliers->indices.size()<1000){
      break;
    }
    vector<float> tmp;
    tmp.push_back(coefficients->values[0]);
    tmp.push_back(coefficients->values[1]);
    tmp.push_back(coefficients->values[2]);
    tmp.push_back(coefficients->values[3]);
    Coffis.push_back(tmp);
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "
      << coefficients->values[2] << " " << coefficients->values[3] << std::endl;
 
    //提取平面
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*output);
    std::cerr << "output point size : " << output->points.size() << std::endl;
    result_clouds.push_back(output);
//     PclShow(output);
    // 移去平面局内点，提取剩余点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative(true);
    extract.filter(*cloud_other);
    std::cerr << "other point size : " << cloud_other->points.size() << std::endl;
    cloud = cloud_other;
  }
}

void PclProject::HandleDepthPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  cout << "coming!" <<endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ToPCL(msg, temp_cloud);
  std::vector<std::vector<float>> Coffis;
  std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> result_clouds;
  int threshold = temp_cloud->size() / 10;
  GetPlanes(temp_cloud,Coffis,result_clouds,threshold);
  
  //get ground
  int ground_index = FindGround(result_clouds);
  float plane_params[4];
  FitPlane(ToEigenPoints(result_clouds[ground_index]), plane_params);
  float A = plane_params[0];
  float B = plane_params[1];
  float C = plane_params[2];
  float D = plane_params[3];
  cout << "plane index: " << A << ","<< B << ","<< C << ","<< D <<endl;
  float sum_squar = A * A + B * B + C * C;
  Point3f p0(0, 0, D / C);
  Point3f p1(result_clouds[ground_index].points.rbegin()->x, result_clouds[ground_index].points.rbegin()->y, 
                              (D - result_clouds[ground_index].points.rbegin()->x * A - result_clouds[ground_index].points.rbegin()->y * B) / C);
  Point3f vx = p1 - p0;
  float sum = sqrt(vx.x*vx.x + vx.y*vx.y + vx.z*vx.z);
  vx = vx * (1. / sum);
  Point3f vz(A, B, C);
  sum = sqrt(vz.x*vz.x + vz.y*vz.y + vz.z*vz.z);
  vz = vz * (1. / sum);
  Point3f vy;
  vy = vz.cross(vx);
  sum = sqrt(vy.x*vy.x + vy.y*vy.y + vy.z*vy.z);
  vy = vy * (1. / sum);
  
  Mat plane2camera_rot = (Mat_<float>(3, 3) <<
    vx.x, vy.x, vz.x,
    vx.y, vy.y, vz.y,
    vx.z, vy.z, vz.z); 
  Eigen::Matrix3f plane2cam_rot_eigen;
  plane2cam_rot_eigen <<
    vx.x, vy.x, vz.x,
    vx.y, vy.y, vz.y,
    vx.z, vy.z, vz.z;
    
    
  
  

  pcl::PointCloud<pcl::PointXYZ>::Ptr all_extract_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  cout << "planes size: " << result_clouds.size() <<endl;
  
  for(auto cloud_target: result_clouds){
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::ExtractIndices<pcl::PointXYZ> ei;
//     ei.setIndices(inliers);
//     ei.setInputCloud(temp_cloud);
//     ei.filter(*cloud_target);
    *all_extract_cloud += *cloud_target;
  }

  
  // display
  
  PclShow(all_extract_cloud);
  pcl::visualization::PCLVisualizer p2;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (temp_cloud, 0, 255, 0);
  p2.addPointCloud(temp_cloud, src_h, "source");
  p2.spin();
  
}

void PclProject::FitPlane(std::vector<Eigen::Vector3f>& plane_points, float* plane12)
{
  CvMat* points_mat = cvCreateMat(plane_points.size(), 3, CV_32FC1);
  for (size_t i = 0; i < plane_points.size(); ++i)
  {
    points_mat->data.fl[i * 3 + 0] = plane_points[i](0);
    points_mat->data.fl[i * 3 + 1] = plane_points[i](1);
    points_mat->data.fl[i * 3 + 2] = plane_points[i](2);
  }
  size_t  length = sizeof(plane12) / sizeof(plane12[0]);
  for(size_t i =0; i < length; i++)
    plane12[i] = 0;
  cvFitPlane(points_mat,plane12);
  float A = plane12[0];
  float B = plane12[1];
  float C = plane12[2];
  float D = plane12[3];
  float sum_squar = A * A + B * B + C * C;
  
  int cnt = 0;
  for (vector<Eigen::Vector3f>::iterator it = plane_points.begin(); it!=plane_points.end(); )
  {
    float dst = fabs(A * it->x() + B * it->y() + C * it->z() - D)/sqrt(sum_squar);
    if(dst > max_dst_error_plane_)
    {
      cnt++;
      it = plane_points.erase(it);
    }
    else
      ++it;
  }
  cout << "cnt: " << cnt <<"\t plane:" << plane12[0] << ","<< plane12[1] << ","<< plane12[2] << ","<< plane12[3] <<endl;
  if(cnt > 1)
    FitPlane(plane_points,plane12);
}

vector< Eigen::Vector3f > PclProject::ToEigenPoints(const pcl::PointCloud< pcl::PointXYZ >::Ptr& cloud)
{
  vector<Eigen::Vector3f> eigen_points;
  for(auto point:cloud->points)
    eigen_points.push_back(Eigen::Vector3f(point.x,point.y,point.z));
  return eigen_points;
}

int PclProject::FindGround(const vector< pcl::PointCloud< pcl::PointXYZ >::Ptr >& clouds)
{
  int k = 0;
  float min_dist = 1e9;
  for(int i =0; i < clouds.size(); i++)
  {
    for(pcl::PointXYZ point: clouds[i])
    {
      float temp_norm = point.x * point.x + point.y * point.y + point.z * point.z;
      if(temp_norm < min_dist)
      {
        k = i;
        min_dist = temp_norm;
      }
    }
  }
  return k;
}

int main(int argc, char** argv){
  limlog::setLogFile("/home/cyy/11222222.log");
  ros::init(argc, argv, "pcl_test");
  limlog::setLogLevel(limlog::LogLevel::INFO);
  LOG_INFO << "hello ros!";
  PclProject pcl;
  ros::spin();
  return 1;
}

