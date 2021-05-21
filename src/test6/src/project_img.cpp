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

#include "project_img.h"
using namespace std;
using namespace cv;
ProjectImg::ProjectImg(const ros::NodeHandle& n):nh_(n)
{
  cv::Mat intrinsic;
  cv::Mat distortion;
  
  intrinsic = Mat::zeros(3, 3, CV_64FC1);
  distortion = Mat::zeros(4, 1, CV_64FC1);
  intrinsic.at<double>(0, 0) = 503.2694 ;
  intrinsic.at<double>(1, 1) = 503.0114 ;
  intrinsic.at<double>(0, 2) = 370.2925;
  intrinsic.at<double>(1, 2) = 205.2823;
  intrinsic.at<double>(2, 2) = 1.0;
  distortion.at<double>(0) = -0.3785;
  distortion.at<double>(1) = 0.1464;
  distortion.at<double>(2) = 0;
  distortion.at<double>(3) = 0;
  
  ros::param::get("/jzhw/camera/jz/down/calib_param/fx", intrinsic.at<double>(0, 0));
  ros::param::get("/jzhw/camera/jz/down/calib_param/fy", intrinsic.at<double>(1, 1));
  ros::param::get("/jzhw/camera/jz/down/calib_param/cx", intrinsic.at<double>(0, 2));
  ros::param::get("/jzhw/camera/jz/down/calib_param/cy", intrinsic.at<double>(1, 2));
  ros::param::get("/jzhw/camera/jz/down/calib_param/k1", distortion.at<double>(0));
  ros::param::get("/jzhw/camera/jz/down/calib_param/k2", distortion.at<double>(1));
  cout <<"\033[34m" << "intrinsic_: " <<  intrinsic <<endl;
  cout <<"\033[34m" <<"distortion_: " <<  distortion <<endl;
  
  Mat extrinsicR = (Mat_<double>(3, 3) << 0.9998 ,   0.0002 ,  -0.0176,
                     -0.0000 ,   0.9999  ,  0.0124,
                     0.0176 ,  -0.0124  ,  0.9998  );
  
  Mat extrinsicT = (Mat_<double>(3, 1) << -33.730323291904966,-55.01432000164834,56.10216083784688);
  
  
  Eigen::Vector3d cam2base_t;
  Eigen::Vector3d cam2base_eul;
  ros::param::get("/jzhw/camera/jz/down/calib_param/px", cam2base_t(0));
  ros::param::get("/jzhw/camera/jz/down/calib_param/py", cam2base_t(1));
  ros::param::get("/jzhw/camera/jz/down/calib_param/pz", cam2base_t(2));
  ros::param::get("/jzhw/camera/jz/down/calib_param/yaw", cam2base_eul(0));
  ros::param::get("/jzhw/camera/jz/down/calib_param/pitch", cam2base_eul(1));
  ros::param::get("/jzhw/camera/jz/down/calib_param/roll", cam2base_eul(2));
  Eigen::Matrix3d base2cam_rot;
  base2cam_rot = (Eigen::AngleAxisd(cam2base_eul[0],Eigen::Vector3d::UnitZ()) 
        * Eigen::AngleAxisd(cam2base_eul[1],Eigen::Vector3d::UnitY()) 
        * Eigen::AngleAxisd(cam2base_eul[2],Eigen::Vector3d::UnitX())).toRotationMatrix().transpose();
  Eigen::Vector3d base2cam_t = -1000*(base2cam_rot * cam2base_t);
  cout <<"base2cam_t : " << base2cam_t <<endl;
  cout << "base2cam_eul: " << base2cam_rot.eulerAngles(2,1,0).transpose() <<endl;
  for(int i=0; i <3; i++){
    for(int j =0; j < 3; j++)
      extrinsicR.at<double>(i,j) = base2cam_rot(i,j);
    extrinsicT.at<double>(i) = base2cam_t(i);
  }
  cout <<"extrinsicT : " << extrinsicT <<endl;
  int projWidth =  300;
  int projHeight = 300;
//   objWidth_ = projWidth_ *1.0 / 2.78;
//   objHeight_ = projHeight_*1.0  / 2.78;
  double objWidth = projWidth *1.0 / 2.78;
  double objHeight = projHeight*1.0  / 2.78;
  int width_ = 752;
  int height_ = 480;
  
  p_ip_ = new ImageProjector(intrinsic, distortion, extrinsicR, extrinsicT,
                             width_, height_, projWidth, projHeight, objWidth,
                             objHeight);
  
  Mat img = imread("/home/cyy/image.png",CV_LOAD_IMAGE_GRAYSCALE);
  Mat projImg;
  p_ip_->projectImage(img, projImg);
  imshow("project_img", projImg);
  waitKey(0);
  exit(1);
}

ProjectImg::~ProjectImg()
{

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "project_img");
  ros::NodeHandle nh;
  ProjectImg ProjectImg(nh);
  ros::spin();

  return 1;
}
