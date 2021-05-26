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
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <algorithm>
#include "pcl/registration/correspondence_estimation_normal_shooting.h"
#include "pcl/registration/correspondence_rejection_surface_normal.h"
#include "pcl/registration/correspondence_rejection_features.h"
#include "pcl/visualization/cloud_viewer.h"
using namespace std;
using namespace limlog;
using namespace cv;
using namespace pcl;
Rigid3d global_submap_pose({-5.91763, 203.921, 0.871792}, {0.980257, 0.0067101, -0.00144523, 0.197608});
Rigid3d global_node_pose({-1.42752, 207.395, 0.980616},{0.102625, 0.00182981, 0.00670749, -0.994696});
Eigen::Quaterniond gravity_alignment(-0.141539, 0,0,-0.989933);
PclProject::PclProject()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  //*打开点云文件
  if (pcl::io::loadPCDFile<pcl::PointXYZI>("/home/cyy/map/pcd3d/pcd_frames/0/submap.pcd", *cloud) == -1) {
      PCL_ERROR("Couldn't read file rabbit.pcd\n");
  }
  pcl::visualization::CloudViewer viewer("cloud viewer");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped()) {

  }
  system("pause");
  point_cloud_sub_ = nh_.subscribe("/front2/depth/color/points",1,&PclProject::HandleDepthPointCloud,this);
  cloud_sub_ = nh_.subscribe("/pointcloud_front",1,&PclProject::HandlePointCloud,this);
  src_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("src_cloud", 1);
  tgt_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("tgt_cloud", 1);
  final_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("final_cloud", 1);
  
  transformed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1);
  max_dst_error_plane_ = 0.005;
  max_dst_error_line_ = 0.005;
  wall_timer_ = nh_.createWallTimer(::ros::WallDuration(0.2),
                          &PclProject::display, this);
  target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  src_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  transformed_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  if(1)
  {
    read_pcd("/home/cyy/submap_37.pcd",target_cloud_);
    read_pcd("/home/cyy/node_1820.pcd",src_cloud_);
    cout << "submap points size: " << target_cloud_->size() <<endl;
    
    Rigid3d gravity({0,0,0},gravity_alignment);
    Rigid3d node2submap_pose = global_submap_pose.inverse() * global_node_pose;
    
    
    cout << global_submap_pose <<endl;
    cout << global_node_pose <<endl;
    cout << node2submap_pose <<endl;
    PointToPlaneIcpMatcher(src_cloud_,target_cloud_);
    Eigen::Matrix4d trans;
    trans << 0-0.991356, 0-0.126272, -0.0356134, 0001.62393,
      0000.12381, 00-0.99021 ,00.0644582, 00-2.44006,
      0-0.043404, 00.0594917, 000.997285, 00.0712701,
      0000000000, 0000000000, 0000000000 ,0000000001;
    for(auto p: src_cloud_->points)
    {
      Eigen::Vector4d p1(p.x,p.y,p.z,1);
      Eigen::Vector4d p2 = trans * p1;
      pcl::PointXYZ p3;
      p3.x = p2(0);p3.y = p2(1);p3.z = p2(2);
      transformed_cloud_->points.push_back(p3);
    }
    
  }
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
                           const unsigned int& threshold)
{
  while (cloud->points.size() >= threshold){
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); 
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.003);
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
  const unsigned int threshold = temp_cloud->size() / 10;
  GetPlanes(temp_cloud,Coffis,result_clouds,threshold);
  
  //get ground
  int ground_index = FindGround(result_clouds);
  cout << "ground_index: " << ground_index <<endl;
  
  vector<int> walls_index = FindWalls(result_clouds, ground_index);
  cout << "walls_index: " << walls_index[0] <<endl;
  float plane_params[4];
  auto planes_points = ToEigenPoints(result_clouds[ground_index]);
  FitPlane(planes_points, plane_params);
  Point3f vector_p1(result_clouds[ground_index]->points.rbegin()->x, 
             result_clouds[ground_index]->points.rbegin()->y, 
             result_clouds[ground_index]->points.rbegin()->z);
  
  Mat cam2plane_rot,cam2plane_t;

  GetCamToPlane(vector_p1,plane_params, cam2plane_rot, cam2plane_t, false);
  vector<Point2f> fit_pts[2];
  float sum_squar = plane_params[0] * plane_params[0] + plane_params[1] * plane_params[1] + plane_params[2] * plane_params[2];
  
  for (size_t i = 0; i < walls_index.size(); i++)
  {
    int positive_cnt = 0;
    int negtive_cnt = 0;
    if( i == 0){
      for (pcl::PointXYZ point : result_clouds[i]->points)
      {
        Mat temp_point = (Mat_<float>(3, 1) << point.x, point.y, point.z);
        Mat newPoint_temp = cam2plane_rot * temp_point + cam2plane_t;
        if(newPoint_temp.at<float>(2) > 0)
          positive_cnt++;
        else if(newPoint_temp.at<float>(2) < 0)
          negtive_cnt++;
      }
      if(positive_cnt < negtive_cnt)
      {
        GetCamToPlane(vector_p1,plane_params, cam2plane_rot, cam2plane_t, true);
        cout << "is reserve: true" << endl; 
      }
      else
        cout << "is reserve: false" << endl;
      cout << "all points size: " << result_clouds[i]->points.size() << "\t positive_cnt: " << positive_cnt <<", " <<negtive_cnt << endl; 
    }
    
    for (pcl::PointXYZ point : result_clouds[i]->points)
    {
      float t = (plane_params[0] * point.x + plane_params[1] * point.y + plane_params[2] * point.z - plane_params[3]) / sum_squar;
      float x = point.x - plane_params[0] * t;
      float y = point.y - plane_params[1] * t;
      float z = point.z - plane_params[2] * t;
      Mat cameraPoint = (Mat_<float>(3, 1) << x, y, z);
      //point in new
      Mat newPoint = cam2plane_rot * cameraPoint + cam2plane_t;
      
      fit_pts[i].push_back(Point2f(newPoint.at<float>(0), newPoint.at<float>(1)));
    }
  }
  
  cv::Mat show_img(1000,1000,CV_8UC3,Scalar(255,255,255));
  for(size_t i =0; i < 2; i++)
    for(size_t j =0; j < fit_pts[i].size(); j++)
    {
      if(j == 0)
        cout << fit_pts[i][j].x << "," <<fit_pts[i][j].y <<endl;
      circle(show_img,Point(fit_pts[i][j].x * 100 + 500, fit_pts[i][j].y * 100+ 500),1, cv::Scalar(255, 0, 255));
    }
  
  Mat line[2];
  for(size_t i =0; i < 2; i++)
  {
    cout << "fit_pts[" << i<<"] size: " << fit_pts[i].size() <<endl;
    FitLine(fit_pts[i],line[i]);
    cout << "fit_pts[" << i<<"] size: " << fit_pts[i].size() <<endl;
  }
  for(int i =0; i < 2; i++)
  {
    for(unsigned int j =0; j < fit_pts[i].size(); j++)
    {
      circle(show_img,Point(fit_pts[i][j].x * 100 + 500, fit_pts[i][j].y * 100 +500),1, cv::Scalar(255, i * 255, 0));
    }
  }
  
  
  
  imshow("temp1", show_img);
  waitKey(0);
    
//   Mat line[2];
//   for(size_t i =0; i < 2; i++)
//   {
//     cout << "fit_pts[" << i<<"] size: " << fit_pts[i].size() <<endl;
//     FitLine(fit_pts[i],line[i]);
//     cout << "fit_pts[" << i<<"] size: " << fit_pts[i].size() <<endl;
//   }
//   
//   for(int i =0; i < 2; i++)
//   {
//     for(int j =0; j < fit_pts[i].size(); j++)
//     {
//       circle(show_img,Point(fit_pts[i][j].x * 100 + 500, fit_pts[i][j].y * 100 +500),1, cv::Scalar(255, i * 255, 0));
//     }
// //     cv::line(show_img2,Point(fit_pts[i].begin()->x* 100 + 500,fit_pts[i].begin()->y* 100 + 500),
// //              Point(fit_pts[i].rbegin()->x* 100 + 500,fit_pts[i].rbegin()->y* 100 + 500),cv::Scalar(255, 0, 0));
//   }
//   for(int j =0; j < laser_point_.size(); j++)
//   {
//     circle(show_img,Point(static_cast<int>(laser_point_[j].x() * 100 + 500), static_cast<int>(laser_point_[j].y() * 100 +500)),1, cv::Scalar(0, 255, 0));
//   }
  
  

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

void PclProject::read_pcd(string pcd_path,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
    PCL_ERROR("Couldn't read file rabbit.pcd\n");
    //return(-1);
  }
  std::cout << cloud->points.size() << std::endl;
}


inline void PointCloud2ToPointXYZ(const sensor_msgs::PointCloud2::ConstPtr& msg, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  sensor_msgs::PointCloud2 cloud_msg = *msg;
  int range_size = cloud_msg.width * cloud_msg.height;
  if(range_size > 0)
  {
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1()
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
//     PointCloud2ToPointXYZ(msg, tmp);
    
    tmp->points.reserve(range_size);

    for(int i = 0; i < range_size; i++)
    {
      pcl::PointXYZ point;
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      if(isnan(point.x) || isnan(point.y) || isnan(point.z))
        continue;
      if(point.x * point.x + point.y * point.y +point.z * point.z > 20*20)
        continue;
      
      tmp->points.push_back(point);
    }
    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(tmp);
    downSizeFilter.filter(*cloud);
  }
}
void PclProject::HandlePointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  static int k=0;
  if(k == 0)
  {
    cout << "set target cloud"<<endl;
    
    PointCloud2ToPointXYZ(msg, target_cloud_);
  }
  else if(k%5 == 0)
  {
    cout << "set src cloud"<<endl;
    PointCloud2ToPointXYZ(msg, src_cloud_);
    Eigen::Matrix4d src2tgt = PointToPlaneIcpMatcher(src_cloud_,target_cloud_);
    Eigen::Matrix3d rotation = src2tgt.block(0,0,3,3);
    Eigen::Vector3d translation(src2tgt(0, 3), src2tgt(1, 3), src2tgt(2, 3));
    cout << src2tgt <<endl;
    cout << rotation <<endl;
    cout << translation <<endl;
    for(const auto &it:src_cloud_->points)
    {
      Eigen::Vector3d p1(it.x,it.y,it.z);
      Eigen::Vector3d p2 = rotation * p1 + translation;
      pcl::PointXYZ p3;
      p3.x = p2(0);p3.y = p2(1);p3.z = p2(2);
      transformed_cloud_->points.push_back(p3);
    }
  }
  if(!src_cloud_->points.empty()){
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*target_cloud_, tempCloud);
    tempCloud.header.frame_id = "base_footprint";
    tempCloud.header.stamp = ros::Time::now();
    tgt_pub_.publish(tempCloud);
    
    sensor_msgs::PointCloud2 tempCloud1;
    pcl::toROSMsg(*src_cloud_, tempCloud1);
    tempCloud1.header.frame_id = "base_footprint";
    tempCloud1.header.stamp = ros::Time::now();
    src_pub_.publish(tempCloud1);
    
    sensor_msgs::PointCloud2 tempCloud2;
    pcl::toROSMsg(*final_cloud_, tempCloud2);
    tempCloud2.header.frame_id = "base_footprint";
    tempCloud2.header.stamp = ros::Time::now();
    final_pub_.publish(tempCloud2);
    
    sensor_msgs::PointCloud2 tempCloud22;
    pcl::toROSMsg(*transformed_cloud_, tempCloud22);
    tempCloud22.header.frame_id = "base_footprint";
    tempCloud22.header.stamp = ros::Time::now();
    transformed_pub_.publish(tempCloud22);
    
  }
  if(k <7)
    k++;
  
}

void PclProject::pubPointCloud2(const ros::Publisher& pub, PointCloud< PointXYZ >::Ptr cloud)
{
  sensor_msgs::PointCloud2 tempCloud22;
    pcl::toROSMsg(*cloud, tempCloud22);
    tempCloud22.header.frame_id = "base_footprint";
    tempCloud22.header.stamp = ros::Time::now();
    pub.publish(tempCloud22);
}

void PclProject::GetCamToPlane(const Point3f& vector_x, const float* plane_index, 
                               Mat& cam2plane_rot, Mat& cam2plane_t, const bool& is_reverse)
{
  float A = plane_index[0];
  float B = plane_index[1];
  float C = plane_index[2];
  float D = plane_index[3];
  cout << "plane index: " << A << ","<< B << ","<< C << ","<< D <<endl;
  
  Point3f p0(0, 0, D / C);
  Point3f p1(vector_x.x, vector_x.y, (D - vector_x.x * A - vector_x.y * B) / C);
  Point3f vx = p1 - p0;
  float sum = sqrt(vx.x*vx.x + vx.y*vx.y + vx.z*vx.z);
  vx = vx * (1. / sum);
  Point3f vz(A, B, C);
  sum = sqrt(vz.x*vz.x + vz.y*vz.y + vz.z*vz.z);
  vz = vz * (1. / sum);
  if(is_reverse)
    vz = -vz;
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
    
  Eigen::Quaternionf plane2cam_q_eigen(plane2cam_rot_eigen);
  Mat plane2camera_t = (Mat_<float>(3, 1) << p0.x, p0.y, p0.z);
  Eigen::Vector3f plane2cam_t_eigen(p0.x, p0.y, p0.z);
  
  cout << "plane2camera_rot: " << plane2camera_rot <<endl;
  cout << "plane2camera_t: " << plane2camera_t.t() <<endl;
  cam2plane_rot = plane2camera_rot.t();
  cam2plane_t = -(cam2plane_rot * plane2camera_t);
}

void PclProject::cvFitPlane(const CvMat* points, float* plane)
{
  int nrows = points->rows;
  int ncols = points->cols;
  int type = points->type;
  CvMat* centroid = cvCreateMat(1, ncols, type);
  cvSet(centroid, cvScalar(0));
  for (int c = 0; c<ncols; c++){
    for (int r = 0; r < nrows; r++)
    {
      centroid->data.fl[c] += points->data.fl[ncols*r + c];
    }
    centroid->data.fl[c] /= nrows;
  }
  // Subtract geometric centroid from each point.  
  CvMat* points2 = cvCreateMat(nrows, ncols, type);
  for (int r = 0; r<nrows; r++)
  for (int c = 0; c<ncols; c++)
    points2->data.fl[ncols*r + c] = points->data.fl[ncols*r + c] - centroid->data.fl[c];
  // Evaluate SVD of covariance matrix.  
  CvMat* A = cvCreateMat(ncols, ncols, type);
  CvMat* W = cvCreateMat(ncols, ncols, type);
  CvMat* V = cvCreateMat(ncols, ncols, type);
  cvGEMM(points2, points, 1, NULL, 0, A, CV_GEMM_A_T);
  cvSVD(A, W, NULL, V, CV_SVD_V_T);
  // Assign plane coefficients by singular vector corresponding to smallest singular value.  
  plane[ncols] = 0;
  for (int c = 0; c<ncols; c++){
    plane[c] = V->data.fl[ncols*(ncols - 1) + c];
    plane[ncols] += plane[c] * centroid->data.fl[c];
  }
  // Release allocated resources.  
  cvReleaseMat(&centroid);
  cvReleaseMat(&points2);
  cvReleaseMat(&A);
  cvReleaseMat(&W);
  cvReleaseMat(&V);
}

void PclProject::FitLine(vector<Point2f>& fit_pts,Mat& line)
{
  fitLine(fit_pts, line, CV_DIST_L2, 0, 0.01, 0.01);
  
  float cos_theta = line.at<float>(0, 0);
  float sin_theta = line.at<float>(1, 0);
  float x0 = line.at<float>(2, 0), y0 = line.at<float>(3, 0);

  float k = sin_theta / cos_theta;
  float b = y0 - k * x0;
  float sum_quar = (k * k + 1);
  int temp_cnt = 0;
  for(vector<Point2f>::iterator it = fit_pts.begin(); it !=fit_pts.end(); )
  {
    float dst = fabs(k * it->x - it->y + b)/sqrt(sum_quar);
    if(dst > max_dst_error_line_)
    {
      it = fit_pts.erase(it);
      ++temp_cnt;
    }
    else
      ++it;
  }
  cout << "cnt: " << temp_cnt <<"\t line:" << k << ","<< b <<endl;
  if(temp_cnt > 1)
    FitLine(fit_pts,line);
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
  unsigned int k = 0;
  float min_dist = 1e9;
  for(unsigned int i =0; i < clouds.size(); i++)
  {
    for(pcl::PointXYZ point: clouds[i]->points)
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

vector< int > PclProject::FindWalls(const vector< pcl::PointCloud< pcl::PointXYZ >::Ptr >& clouds,
                                    const int& ground_index)
{
  int i =0;
  vector<int> walls_index;
  
  for(auto cloud:clouds)
  {
    if(ground_index != i)
    {
      walls_index.push_back(i);
    }
    if(walls_index.size() == 2)
      break;
  }
  return walls_index;
}

Eigen::Matrix4d PclProject::PointToPlaneIcpMatcher (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, 
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2) 
{
  
  cout << "start matching..." <<endl;
  pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*cloud1, *src);
//   pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
//   pcl::copyPointCloud(*cloud2, *tgt);
  
//   pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
//   norm_est.setSearchMethod (pcl::search::KdTree<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
//   norm_est.setKSearch (10);
//   norm_est.setInputCloud (tgt);
//   norm_est.compute (*tgt);
//   
//   pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est2;
//   norm_est2.setSearchMethod (pcl::search::KdTree<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
//   norm_est2.setKSearch (10);
//   norm_est2.setInputCloud (src);
//   norm_est2.compute (*src);
//   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp1;
//   icp1.setMaximumIterations (10);
//   icp1.setInputSource (cloud1);
//   icp1.setInputTarget (cloud2);
//   pcl::PointCloud<pcl::PointXYZ> output1;
//   icp1.align(output1);
//   icp1.getFinalTransformation();
//   icp1.getFitnessScore();
//   final_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::copyPointCloud(output1, *final_cloud_);
//   return icp1.getFinalTransformation().cast<double>();;
  
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  //建立kdtree来进行近邻点集搜索
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  //为kdtree添加点云数据
  //点云法向计算时，需要搜索的近邻点大小
  n.setKSearch(20);
  //开始进行法向计算
  
//     tree->setInputCloud(cloud1);
// 
//   n.setInputCloud(cloud1);
//   n.setSearchMethod(tree);
// 
//   n.compute(*normals);
  //* normals should not contain the point normals + surface curvatures
  //将点云数据与法向信息拼接
//   pcl::PointCloud<pcl::PointNormal>::Ptr init_trans_cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
//   pcl::concatenateFields(*cloud1, *normals, *init_trans_cloud_normals);

  pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
  tree->setInputCloud(cloud2);
  n.setInputCloud(cloud2);
  n.setSearchMethod(tree);
  n.compute(*normals1);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_model_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud2, *normals1, *cloud_model_normals);

  pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
  typedef pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal> PointToPlane;
//   typedef pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal> SymmPointToPlane;

  boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
  icp.setTransformationEstimation(point_to_plane);
  icp.setInputSource(src);
  icp.setInputTarget(cloud_model_normals);
//   icp.setRANSACOutlierRejectionThreshold(0.5);
//   icp.setRANSACIterations(100);
  icp.setMaximumIterations(10);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(0.1);
  pcl::PointCloud<pcl::PointNormal> output;
    Rigid3d gravity({0,0,0},gravity_alignment);
    Rigid3d node2submap_pose = global_submap_pose.inverse() * global_node_pose;
  Eigen::Matrix4d prior(Eigen::Matrix4d::Identity());
  prior.topLeftCorner<3,3>() = (node2submap_pose.rotation()).toRotationMatrix();
  prior(0,3) =  node2submap_pose.translation().x();
  prior(1,3) =  node2submap_pose.translation().y();
  prior(2,3) =  node2submap_pose.translation().z(); 
  cout << "prior : \n" << prior <<endl;
  pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal>::Ptr 
    ce(new pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal,pcl::PointNormal,pcl::PointNormal>);
//   icp.setCorrespondenceEstimation(ce);

  // Add rejector
//   pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr 
//     rej(new pcl::registration::CorrespondenceRejectorSurfaceNormal);
// //   rej->setThreshold (0); //Could be a lot of rotation -- just make sure they're at least within 0 degrees
// //   rej->set
// //     rej->setInputNormals(cloud_model_normals);
//     rej->setThreshold (0); 
//   icp.addCorrespondenceRejector (rej);

  cout << "111" <<endl;
  icp.align(output,prior.cast<float>());
//   for (int iter = 0; iter < 4; iter++)
//   {
//     bool force_cache = (bool) iter/2;
//     bool force_cache_reciprocal = (bool) iter%2;
//     pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
//     // Ensure that, when force_cache is not set, we are robust to the wrong input
//     if (force_cache)
//       tree->setInputCloud (cloud_model_normals);
//     icp.setSearchMethodTarget (tree, force_cache);
// 
//     pcl::search::KdTree<PointT>::Ptr tree_recip (new pcl::search::KdTree<PointT>);
//     if (force_cache_reciprocal)
//       tree_recip->setInputCloud (src);
//     icp.setSearchMethodSource (tree_recip, force_cache_reciprocal);
// 
//     // Register
//     icp.align (output);
// //     EXPECT_EQ (int (output.points.size ()), int (cloud_source.points.size ()));
// //     EXPECT_LT (reg.getFitnessScore (), 0.005);
//   }
  cout << "111" <<endl;
  final_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(output, *final_cloud_);
  std::cout <<icp.getFitnessScore() <<endl;
//   if(icp.getFitnessScore() > 0.2)
//     return nullptr;
  std::cout <<icp.hasConverged()  << ", " << icp.getFinalTransformation() << std::endl; 
  return icp.getFinalTransformation().cast<double>();;
  //     
}

void PclProject::display(const ros::WallTimerEvent& unused_timer_event)
{
  if(!transformed_cloud_->points.empty())
  {
    pubPointCloud2(src_pub_, src_cloud_);
    pubPointCloud2(tgt_pub_, target_cloud_);
    pubPointCloud2(transformed_pub_, transformed_cloud_);
//     pubPointCloud2(transformed_pub_, final_cloud_);
  }
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

