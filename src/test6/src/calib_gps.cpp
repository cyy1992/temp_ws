/*
 * Copyright 2019 <copyright holder> <email>
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

#include "calib_gps.h"
#include <thread>
using namespace std;
using namespace cv;
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }


CalibGps::CalibGps(const ros::NodeHandle& n):nh_(n)
{
  gps_sub_ = nh_.subscribe("/jzhw/gps/fix",10,&CalibGps::handleGps,this);
  state_srv_ = nh_.advertiseService("/gps/update_state",&CalibGps::updateState,this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("calib_gps_path",1);
  initialized_ = false;
  state_ = 0;
  num_dip_ = 0;
  average_dip_ = 0;
}

CalibGps::~CalibGps()
{

}
Eigen::Vector3d CalibGps::LatLongAltToEcef(const double latitude, const double longitude,
                                               const double altitude) {
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(DegToRad(latitude));
  const double cos_phi = std::cos(DegToRad(latitude));
  const double sin_lambda = std::sin(DegToRad(longitude));
  const double cos_lambda = std::cos(DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + altitude) * cos_phi * cos_lambda;
  const double y = (N + altitude) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + altitude) * sin_phi;
  
  return Eigen::Vector3d(x, y, z);
}

const CalibGps::Rigid3d CalibGps::ComputeLocalFrameFromLatLong(
  const double latitude, const double longitude) {
  
  const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, 0.);
  const Eigen::Quaterniond 
    rotation =  Eigen::AngleAxisd(DegToRad(latitude - 90.),
                         Eigen::Vector3d::UnitY()) * 
                         Eigen::AngleAxisd(DegToRad(-longitude),
                                           Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond 
    rotation2 =  Eigen::AngleAxisd(DegToRad(latitude - 90.),
                         Eigen::Vector3d::UnitY()) * 
                         Eigen::AngleAxisd(DegToRad(-longitude),
                                           Eigen::Vector3d::UnitZ());
//   dbg(rotation.toRotationMatrix());
//   dbg(rotation.toRotationMatrix().inverse().eulerAngles(2,1,0).transpose());
  return Rigid3d({rotation * -translation, rotation});
}
  
void CalibGps::handleGps(const gps_common::GPSFix::ConstPtr& msg)
{
  if(!initialized_)
  {
    initialized_ = true;
    ecef_to_local_frame_ = ComputeLocalFrameFromLatLong(msg->latitude, msg->longitude);
    cout << "ecef_to_local_frame:" << ecef_to_local_frame_;
  }
  Rigid3d fix2
  = ComputeLocalFrameFromLatLong(msg->latitude, msg->longitude);
  Rigid3d fix_pose1 = ecef_to_local_frame_ * fix2.inverse();
  geometry_msgs::PoseStamped temp_pose;
  temp_pose.header.stamp = msg->header.stamp;
  temp_pose.pose.position.x = fix_pose1.t(0);
  temp_pose.pose.position.y = fix_pose1.t(1);
  temp_pose.pose.position.z = fix_pose1.t(2);
  gps_path_.poses.push_back(temp_pose);
  gps_path_.header.frame_id = "base_footprint";
  cout << atan2(fix_pose1.t(1),fix_pose1.t(0)) * 180 / M_PI <<endl;
  path_pub_.publish(gps_path_);
  
  
  if(state_ == 0){
    positions_[0].push_back(cv::Point2d(fix_pose1.t(0),fix_pose1.t(1)));
    if(msg->err_dip !=0 && msg->err_dip < 1.0)
    {
      num_dip_ ++;
      average_dip_ = (average_dip_ * (num_dip_-1) + msg->dip) /num_dip_;
    }
  }
  else if(state_ == 1)
  {
    positions_[1].push_back(cv::Point2d(fix_pose1.t(0),fix_pose1.t(1)));
  
    
  }
//   cout << fix_pose1 <<endl;
}
bool CalibGps::updateState(std_srvs::Empty::Request& request, 
                           std_srvs::Empty::Response& response)
{
  state_++;
  initialized_ = false;
  cout << ("switch state, now the state is "+ to_string(state_)) <<endl;
  if(state_ == 2){
    std::thread th(&CalibGps::computeExtrinsicParams,this);
    th.detach();
  }
  return true;
}
void CalibGps::computeExtrinsicParams()
{
  vector<cv::Point2d> line_pts;
  vector<cv::Point> circle_pts;
  Mat img(1000,1000,CV_8UC3,Scalar(255,255,255));
  for(int i= 0; i < positions_[0].size(); i ++ )
  {
    cv::Point2d pt = positions_[0][i];
    circle(img,cv::Point2d(positions_[0][i].x * 100,positions_[0][i].y * 100)+Point2d(500,500),1,Scalar(0,255,0));
    line_pts.push_back(pt);
  }
  double vx,vy/*,A,B,C*/;
  Vec4d line2;
  fitLine(line_pts, line2, CV_DIST_L2, 0, 0.01, 0.01);
  vx = line2[0];
  vy = line2[1];
  average_dip_ = average_dip_ * M_PI /180;
  double theta = 2 * M_PI - atan2(vy, vx) - average_dip_;
  dbg(atan2(vy, vx));
  dbg(average_dip_);
  dbg(theta);
  
  for(int i = 0; i < positions_[1].size(); i++)
  {
    double x,y;
    x= positions_[1][i].x* 100;
    y = positions_[1][i].y* 100;
    
    cv::Point pt( x , y);
    cout << "x:" << x <<", y:" <<y<<endl;
    circle(img,cv::Point2d(positions_[1][i].x * 100,positions_[1][i].y * 100)+Point2d(500,500),1,Scalar(255,255,0));
    circle_pts.push_back(pt);
  }
  imshow("calib_gps_img1",img);
  waitKey(0);
  dbg(circle_pts.size());
//   putText(img,"lidar trajectory",Point2d(100,50),cv::FONT_HERSHEY_COMPLEX,1,Scalar(255,0,0));
  RotatedRect rect = fitEllipse(circle_pts);
  dbg(rect.center);
  circle(img,Point2d(rect.center.x ,rect.center.y)+Point2d(500,500),3,Scalar(0,0,0));
  {
    RotatedRect box = rect;
    
    box.center = cv::Point2f( rect.center.x,rect.center.y )+ Point2f(500,500);
    ellipse(img, box, Scalar(0,0,255), 1, CV_AA);
//     putText(img,"fitEllipse",Point2d(100,350),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,0,255));
  }
  double dx,dy;
//   double center_x = rect.center.x *0.0001;
//   double center_y = rect.center.y *0.0001;
  dx = -(rect.center.x *0.01 * cos(theta) + rect.center.y *0.01 * sin(theta));
  dy = -(-(rect.center.x *0.01 * sin(theta)) + rect.center.y *0.01 * cos(theta));
//   double dx1,dy1;
  Mat centerPt = (Mat_<double>(3, 1) << dx, dy , 0);
  dbg(centerPt);
  imshow("calib_gps_img",img);
  waitKey(0);
  exit(1);
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"calib_gps_node");
  ros::NodeHandle n;
  CalibGps gps(n);
  ros::spin();
  return 1;
}


