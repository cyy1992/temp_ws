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

#include "calib_rgbd_with_laser.h"
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace std;
using namespace cv;
CalibRgbdWithLaser::CalibRgbdWithLaser(const ros::NodeHandle& n):nh_(n),state_(0),plane_cnt_(0),corner_cnt_(0),laser_cnt_(0)
{
  state_srv_ = nh_.advertiseService("/calib_rgbd_with_laser/set_state",&CalibRgbdWithLaser::SetState,this);
  depth_img_sub_ = nh_.subscribe("/front/depth/image_rect_raw",1,&CalibRgbdWithLaser::HandleDepthImage,this);
  laser_sub_ = nh_.subscribe("/scan_emma_nav_front",1,&CalibRgbdWithLaser::HandleLaserScan,this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/temp_pointcloud2",1);
  Eigen::Matrix3f K;
  K.setIdentity();
//   ros::param::get("/jzhw/calib/camera/front/fx_depth", K.matrix()(0, 0));
//   ros::param::get("/jzhw/calib/camera/front/fy_depth", K.matrix()(1, 1));
//   ros::param::get("/jzhw/calib/camera/front/cx_depth", K.matrix()(0, 2));
//   ros::param::get("/jzhw/calib/camera/front/cy_depth", K.matrix()(1, 2));
  K.matrix()(0, 0) = 209.2635498046875;
  K.matrix()(1, 1) = 209.2635498046875;
  K.matrix()(0, 2) = 208.588623046875;
  K.matrix()(1, 2) = 116.7160415649414;
  K_inv = K.inverse();
  offset_ = 40;
}

bool CalibRgbdWithLaser::SetState(std_srvs::Empty::Request& request, 
                                  std_srvs::Empty::Response& response)
{
  state_ +=1;
  if(state_ == 2)
    computeResult();
  return true;

}

void CalibRgbdWithLaser::computeResult()
{
  float plane[4];
  cout << "plane_points_[0] size: " <<plane_points_[0].size() <<endl;
  FitPlane(plane_points_[0],plane);
  float A = plane[0];
  float B = plane[1];
  float C = plane[2];
  float D = plane[3];
  cout << "plane index: " << A << ","<< B << ","<< C << ","<< D <<endl;
  float sum_squar = A * A + B * B + C * C;
  Point3f p0(0, 0, D / C), p1(plane_points_[1].rbegin()->x(), plane_points_[1].rbegin()->y(), 
                              (D - plane_points_[1].rbegin()->x() * A - plane_points_[1].rbegin()->y() * B) / C);
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
  Eigen::Quaternionf plane2cam_q_eigen(plane2cam_rot_eigen);
  Mat plane2camera_t = (Mat_<float>(3, 1) << p0.x, p0.y, p0.z);
  Eigen::Vector3f plane2cam_t_eigen(p0.x, p0.y, p0.z);
  
  cout << "plane2camera_rot: " << plane2camera_rot <<endl;
  cout << "plane2camera_t: " << plane2camera_t.t() <<endl;
  Mat cam2plane_rot = plane2camera_rot.t();
  Mat cam2plane_t = -(cam2plane_rot * plane2camera_t);
  vector<Point2f> fit_pts[2];
  for (size_t i = 1; i < 3; i++)
  {
    for (size_t j = 0; j < plane_points_[i].size(); ++j)
    {
      float t = (A * plane_points_[i][j].x() + B * plane_points_[i][j].y() + C * plane_points_[i][j].z() - D) / sum_squar;
      float x = plane_points_[i][j].x() - A * t;
      float y = plane_points_[i][j].y() - B * t;
      float z = plane_points_[i][j].z() - C * t;
      Mat cameraPoint = (Mat_<float>(3, 1) << x, y, z);
      //point in new
      Mat newPoint = cam2plane_rot * cameraPoint + cam2plane_t;
      
      fit_pts[i-1].push_back(Point2f(newPoint.at<float>(0), newPoint.at<float>(1)));
      if(j < 10)
        cout << x <<"," <<y <<"," <<z <<endl;
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
    
  imshow("temp_img",show_img);
  waitKey(0);
  Mat line[2];
  for(size_t i =0; i < 2; i++)
  {
    cout << "fit_pts[" << i<<"] size: " << fit_pts[i].size() <<endl;
    FitLine(fit_pts[i],line[i]);
    cout << "fit_pts[" << i<<"] size: " << fit_pts[i].size() <<endl;
  }
  
  for(size_t i =0; i < 2; i++)
  {
    for(size_t j =0; j < fit_pts[i].size(); j++)
    {
      circle(show_img,Point(fit_pts[i][j].x * 100 + 500, fit_pts[i][j].y * 100 +500),1, cv::Scalar(255, i * 255, 0));
    }
//     cv::line(show_img2,Point(fit_pts[i].begin()->x* 100 + 500,fit_pts[i].begin()->y* 100 + 500),
//              Point(fit_pts[i].rbegin()->x* 100 + 500,fit_pts[i].rbegin()->y* 100 + 500),cv::Scalar(255, 0, 0));
  }
  for(size_t j =0; j < laser_point_.size(); j++)
  {
    circle(show_img,Point(static_cast<int>(laser_point_[j].x() * 100 + 500), static_cast<int>(laser_point_[j].y() * 100 +500)),1, cv::Scalar(0, 255, 0));
  }
  imshow("temp_img2",show_img);
  waitKey(0);
  
  vector<Eigen::Vector3f> temp;
  for(size_t i =0; i < 2; i++)
  {
    for(size_t j =0; j < fit_pts[i].size(); j++)
    {
      temp.push_back(Eigen::Vector3f(fit_pts[i][j].x,fit_pts[i][j].y,0));
    }
  }
  pcl::PointCloud<PointType>::Ptr src = ToPCL(temp);
  pcl::PointCloud<PointType>::Ptr target = ToPCL(laser_point_);
  Eigen::Matrix4d transform;
  ICPMatch(src,target,transform);
  Eigen::Matrix3d rotation = transform.block(0,0,3,3);
  Eigen::Quaterniond q_temp(rotation);
  Eigen::Quaternionf q = q_temp.cast<float>();
  Eigen::Vector3f translation(transform(0, 3), transform(1, 3), transform(2, 3));
  for(auto &it:laser_point_)
  {
    it = q * it + translation;
  }
  cv::Mat show_img2(1000,1000,CV_8UC3,Scalar(255,255,255));
  for(size_t j =0; j < laser_point_.size(); j++)
  {
    circle(show_img2,Point(static_cast<int>(laser_point_[j].x() * 100 + 500), static_cast<int>(laser_point_[j].y() * 100 +500)),1, cv::Scalar(0, 255, 0));
  }
  for(size_t i =0; i < 2; i++)
  {
    for(size_t j =0; j < fit_pts[i].size(); j++)
    {
      circle(show_img2,Point(fit_pts[i][j].x * 100 + 500, fit_pts[i][j].y * 100 +500),1, cv::Scalar(255, i * 255, 0));
    }
  }
  imshow("temp_img3",show_img2);
  waitKey(0);
  Eigen::Quaternionf laser2cam_q = plane2cam_q_eigen * q;
  Eigen::Vector3f laser2cam_t = plane2cam_q_eigen * translation + plane2cam_t_eigen;
  cout << "laser2cam: R:\n" << laser2cam_q.toRotationMatrix() <<"\n t: " <<laser2cam_t.transpose() <<endl;
}


void CalibRgbdWithLaser::FitLine(vector<Point2f>& fit_pts,Mat& line)
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
    if(dst > 0.05)
    {
      it = fit_pts.erase(it);
      ++temp_cnt;
    }
    else
      ++it;
  }
  cout << "cnt: " << temp_cnt <<"\t line:" << k << ","<< b <<endl;
  if(temp_cnt > 5)
    FitLine(fit_pts,line);
}
 
void CalibRgbdWithLaser::cvFitPlane(const CvMat* points, float* plane)
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

void CalibRgbdWithLaser::FitPlane(std::vector<Eigen::Vector3f>& plane_points, float* plane12)
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
    if(dst > 0.01)
    {
//       cout << "dst: " << dst <<endl;
      cnt++;
      it = plane_points.erase(it);
      
    }
    else
      ++it;
  }
  cout << "cnt: " << cnt <<"\t plane:" << plane12[0] << ","<< plane12[1] << ","<< plane12[2] << ","<< plane12[3] <<endl;
  if(cnt > 10)
    FitPlane(plane_points,plane12);
}


void CalibRgbdWithLaser::HandleDepthImage(const sensor_msgs::Image::ConstPtr& msg)
{
  Mat depth_image = cv_bridge::toCvCopy(msg)->image;
  depth_image.convertTo(depth_image, CV_32F);
  
  int point_cnt = 0;
  
  int nr = depth_image.rows;
  int nc = depth_image.cols;
  if(state_ == 0) // add plane points  
  {
    plane_cnt_++;
    if(plane_cnt_ > 3)
      return;
    for (int i = nr/2; i < nr - offset_; i++)
    {
      const float* data = depth_image.ptr<float>(i);
      for (int j = 1 ; j < nc ; j++)
      {
        if(j > offset_ *2  && j < nc - offset_*2 ){
          float depth = (*data) * 0.001f;
          
          if (depth > 1e-9 && depth < 2.f)
          {
            Eigen::Vector3f vec;
            vec << depth * j, depth * i, depth;
            Eigen::Vector3f point_in_sensor = K_inv * vec;

            plane_points_[0].push_back(point_in_sensor);
            point_cnt++;
          }
        }
        data++;
      }
    }
  }
  else if(state_ == 1)
  {
    corner_cnt_++;
    if(corner_cnt_ > 3)
      return;
    float max_depth = 0;
    int divid_num = 1;
    const float* temp_data = depth_image.ptr<float>(0);
    for (int i = 1; i < nc; i++)
    {
      float depth = (*temp_data);
      temp_data++;
      if(max_depth < depth)
      {
        max_depth = depth;
        divid_num = i;
      }
    }
    //TODO for every row to search max_depth and relative divid_num
    cout << "max_depth: " <<max_depth << ", divid_num: " <<divid_num <<endl;
    for (int i = offset_; i < nr / 3; i++)
    {
      const float* data = depth_image.ptr<float>(i);
      for (int j = 1 ; j < nc ; j++)
      {
        if(j > offset_ && j < nc - offset_ ){
          float depth = (*data) * 0.001f;
          Eigen::Vector3f vec;
          vec << depth * j, depth * i, depth;

          Eigen::Vector3f point_in_sensor = K_inv * vec;
          if(divid_num > j)
          {
            plane_points_[1].push_back(point_in_sensor);
          }
          if (divid_num < j)
          {
            plane_points_[2].push_back(point_in_sensor);          
          }
        }
        
        data++;
      }
    }
    

  }
  vector<Eigen::Vector3f> temp;
  temp.insert(temp.end(),plane_points_[0].begin(),plane_points_[0].end());
  temp.insert(temp.end(),plane_points_[1].begin(),plane_points_[1].end());
  temp.insert(temp.end(),plane_points_[2].begin(),plane_points_[2].end());
//   pubPointCloud(temp);
}

void CalibRgbdWithLaser::HandleLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
  if(state_ == 1)
  {
    laser_cnt_++;
    if(laser_cnt_ > 3)
      return;
    sensor_msgs::LaserScan msg = *laser_msg;
    float angle = msg.angle_min;
    float seg_angle = 30.0 * M_PI / 180;
    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      const float first_echo = msg.ranges[i];
      if (msg.range_min <= first_echo && first_echo <= msg.range_max && angle < seg_angle &&angle > -seg_angle) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
       
        Eigen::Vector3f position = rotation * (first_echo * Eigen::Vector3f::UnitX());
        laser_point_.push_back(position);
       
      }
      angle += msg.angle_increment;
    }
    
    pubPointCloud(laser_point_);
  }
  
}

void CalibRgbdWithLaser::pubPointCloud(const std::vector<Eigen::Vector3f>& plane_points)
{
  
  sensor_msgs::PointCloud2 cloud_in_sensor;
  cloud_in_sensor.header.stamp = ros::Time::now();
  cloud_in_sensor.header.frame_id = "camera_link3";
  cloud_in_sensor.height = 1;
  cloud_in_sensor.width = 3;
  sensor_msgs::PointCloud2Modifier modifier(cloud_in_sensor);
  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
                                "y", 1, sensor_msgs::PointField::FLOAT32, "z",
                                1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(plane_points.size());
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_in_sensor, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_in_sensor, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_in_sensor, "z");
  // std::ofstream fout("/home/miow/depth.txt");
  for (size_t i = 0; i < plane_points.size(); i++)
  {
    Eigen::Vector3f point_in_sensor = plane_points[i];
    *iter_x = point_in_sensor(0);
    *iter_y = point_in_sensor(1);
    *iter_z = point_in_sensor(2);
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  cloud_pub_.publish(cloud_in_sensor);
}

pcl::PointCloud< PointType >::Ptr CalibRgbdWithLaser::ToPCL(const vector< Eigen::Vector3f >& points)
{
  pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>);
  for(auto it : points)
  {
    PointType point;
    point.x = it(0);
    point.y = it(1);
    point.z = it(2);
    output->points.push_back(point);
  }
  return std::move(output);
}
void CalibRgbdWithLaser::print4x4Matrix(const Eigen::Matrix4d & matrix)
{
  cout << ("Rotation matrix :\n");
  cout << ( matrix(0, 0), matrix(0, 1), matrix(0, 2));
  cout << ( matrix(1, 0), matrix(1, 1), matrix(1, 2));
  cout << ( matrix(2, 0), matrix(2, 1), matrix(2, 2));
  cout << ("Translation vector :\n");
  cout << (matrix(0, 3), matrix(1, 3), matrix(2, 3)) <<endl;
}

void CalibRgbdWithLaser::ICPMatch(const pcl::PointCloud<PointType>::Ptr &cloud_target, 
                 const pcl::PointCloud<PointType>::Ptr &cloud_source, 
                 Eigen::Matrix4d &transform )
{
  transform = Eigen::Matrix4d::Identity();
  pcl::PointCloud<PointType>::Ptr src(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr tgt(new pcl::PointCloud<PointType>);

  tgt = cloud_target;
  src = cloud_source;
  cout << "src size: " << src->size() <<"\t target size: " << tgt->size() <<endl;
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(0.8);
  icp.setTransformationEpsilon(1e-10);
  icp.setEuclideanFitnessEpsilon(0.01);
  icp.setMaximumIterations(1000);

  icp.setInputSource (src);
  icp.setInputTarget (tgt);
  icp.align (*src);
  if(icp.hasConverged())
  {
    cout << "icp success" <<endl;
    transform = icp.getFinalTransformation().cast<double>();
    print4x4Matrix(transform);
    cout << "transform: " << transform <<endl;
  }
  else
     cout << "icp failed" <<endl;
//   output->resize(tgt->size()+output->size());
//   for (int i=0;i<tgt->size();i++)
//   {
//     output->push_back(tgt->points[i]);
//   }
//   cout<<"After registration using ICP:"<<output->size()<<endl;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "calib_rgbd_with_laser");
  ros::NodeHandle n;
  CalibRgbdWithLaser calib(n);
  ros::spin();
  return 1;
}