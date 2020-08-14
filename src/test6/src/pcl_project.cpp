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

using namespace std;
using namespace limlog;
using namespace cv;
PclProject::PclProject()
{
  point_cloud_sub_ = nh_.subscribe("/front2/depth/color/points",1,&PclProject::HandleDepthPointCloud,this);
  max_dst_error_plane_ = 0.005;
  max_dst_error_line_ = 0.005;
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

int main(int argc, char** argv){
  limlog::setLogFile("/home/cyy/11222222.log");
  ros::init(argc, argv, "pcl_test");
  limlog::setLogLevel(limlog::LogLevel::INFO);
  LOG_INFO << "hello ros!";
  PclProject pcl;
  ros::spin();
  return 1;
}

