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

#include "depth_color_registration.h"
#include <cv_bridge/cv_bridge.h>
#include "depth2colorAlign.h"
using namespace cv;
using namespace std;
DepthColorRegistration::DepthColorRegistration()
{
  
  depth_sub_.subscribe(nh_, "/front/depth/image_rect_raw", 3);
  color_sub_.subscribe(nh_, "/front/color/image_raw", 3);
  DepthColorSyncPolicy policy_scans(20);
  scansSync_ = new message_filters::Synchronizer<DepthColorSyncPolicy>(DepthColorSyncPolicy(policy_scans), depth_sub_, color_sub_);
  scansSync_->registerCallback(boost::bind(&DepthColorRegistration::imagesCallback, this, _1, _2));
  
  depth_intrinsic_ = Mat::zeros(3, 3, CV_64FC1);
  color_intrinsic_ = Mat::zeros(3, 3, CV_64FC1);
  depth2color_ = Mat::zeros(4, 4, CV_64FC1);
  
  depth_intrinsic_.at<double>(0, 0) = 212.54563903808594;
  depth_intrinsic_.at<double>(1, 1) = 212.54563903808594;
  depth_intrinsic_.at<double>(0, 2) = 212.65614318847656;
  depth_intrinsic_.at<double>(1, 2) = 124.0171127319336;
  depth_intrinsic_.at<double>(2, 2) = 1;
  
  color_intrinsic_.at<double>(0, 0) = 899.851 ;
  color_intrinsic_.at<double>(1, 1) = 897.192;
  color_intrinsic_.at<double>(0, 2) = 642.591;
  color_intrinsic_.at<double>(1, 2) = 366.916;
  color_intrinsic_.at<double>(2, 2) = 1;
  
  depth2color_.at<double>(0,0) = 0.9998875260353088;
  depth2color_.at<double>(0,1) = 0.014821221120655537;
  depth2color_.at<double>(0,2) = 0.002307532588019967;
  depth2color_.at<double>(1,0) = -0.014824875630438328;
  depth2color_.at<double>(1,1) = 0.9998888373374939;
  depth2color_.at<double>(1,2) = 0.0015745549462735653;
  depth2color_.at<double>(2,0) = -0.0022839391604065895;
  depth2color_.at<double>(2,1) = -0.0016085866373032331;
  depth2color_.at<double>(2,2) = 0.9999961256980896;
  depth2color_.at<double>(0,3) = 0.014732059091329575;
  depth2color_.at<double>(1,3) = 0.0003794773365370929;
  depth2color_.at<double>(2,3) = 0.0003817194083239883;
//   depth2color_.at<double>(0,0) = 0.999453404256005;
//   depth2color_.at<double>(0,1) = -0.00956534772566204;
//   depth2color_.at<double>(0,2) = -0.0316448549367741;
//   depth2color_.at<double>(1,0) = 0.00966546346485642;
//   depth2color_.at<double>(1,1) = 0.999948751209193;
//   depth2color_.at<double>(1,2) = 0.00301227010506918;
//   depth2color_.at<double>(2,0) = 0.0316144197652247;
//   depth2color_.at<double>(2,1) = -0.00331648580029223;
//   depth2color_.at<double>(2,2) = 0.999494636996540;
//   depth2color_.at<double>(0,3) = -0.00928060839271329;
//   depth2color_.at<double>(1,3) = 0.000687241533046751;
//   depth2color_.at<double>(2,3) = -0.00156753173132629;
  depth2color_.at<double>(3,3) = 1;
  cv::invert(depth2color_,color2depth_);
  cout << "depth_intrinsic: " << depth_intrinsic_ <<endl;
  cout << "color_intrinsic: " << color_intrinsic_ <<endl;
  cout << "depth2color: " << depth2color_ <<endl;
  cout << "color2depth: " << color2depth_ <<endl;
}

DepthColorRegistration::~DepthColorRegistration()
{

}

void DepthColorRegistration::imagesCallback(const sensor_msgs::Image::ConstPtr& depth_msg, 
                                            const sensor_msgs::Image::ConstPtr& color_msg)
{
  Mat im_depth = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
  Mat im_color = cv_bridge::toCvCopy(color_msg)->image;
//   Mat im_color;
//   resize(color_img1,im_color,Size(im_depth.cols,im_depth.rows));
  
  im_color.convertTo(im_color, CV_16UC3);
  im_color = im_color * 255;

  cv::namedWindow("color", CV_WINDOW_NORMAL);
  imshow("color", im_color);
  cv::namedWindow("depth", CV_WINDOW_NORMAL);
  imshow("depth", im_depth);
  cv::waitKey(2);

  cv::Mat im_registrated_depth;

  cv::Mat Transform = color2depth_;

  cv::Mat K_color = color_intrinsic_;

  cv::Mat K_depth = depth_intrinsic_;



  cv::Size color_size(im_color.cols, im_color.rows);
  cv::Size depth_size(im_depth.cols, im_depth.rows);

  Depth2ColorAlign app(color_size, depth_size, color2depth_, K_color, K_depth);

  //while(1)
  {
    const clock_t begin_time = clock();
    app.align(im_depth, im_registrated_depth);
//     std::cout << float(clock() - begin_time) / CLOCKS_PER_SEC * 1000.0 << "ms" << std::endl;
    cv::waitKey(2);
  }

  // std::cout << im_registrated_depth << std::endl;

  cv::namedWindow("registrated depth", CV_WINDOW_NORMAL);
  imshow("registrated depth", im_registrated_depth);
  cvtColor(im_registrated_depth, im_registrated_depth, CV_GRAY2BGR);

  cv::Mat colorAddDepth = im_color/2  + im_registrated_depth*16 ;
  cv::namedWindow("add", CV_WINDOW_NORMAL);
  
  imshow("add", colorAddDepth);
 
//   Mat out;
//   if (0 && Register(depth_img,color_img,out) )
//     imshow("out",out);  
//   imshow("depth", depth_img);
//   imshow("color", color_img);
//   imshow("color1", color_img1);
  waitKey(2);
}

bool DepthColorRegistration::Register(cv::Mat &depth, cv::Mat &color, cv::Mat &out)
{
  if (depth.rows != color.rows || depth.cols != color.cols)
    return false;
  if (depth_intrinsic_.rows != 3 || depth_intrinsic_.cols != 3 || color_intrinsic_.rows != 3 || color_intrinsic_.cols != 3 ||
    color2depth_.rows != 4 || color2depth_.cols != 4)
    return false;
  cv::Mat depthtemp(depth.size(), depth.type());
  for (int i = 0; i<depth.rows; i++)
  {
    unsigned short* temp = depth.ptr<unsigned short>(i);
    for (int j = 0; j<depth.cols; j++)
    {
      float z = temp[j];
      float x = (j - depth_intrinsic_.data[2])*z / depth_intrinsic_.data[0];
      float y = (i - depth_intrinsic_.data[5])*z / depth_intrinsic_.data[4];
 
      float px = color2depth_.data[0] * x + color2depth_.data[1] * y + color2depth_.data[2] * z + color2depth_.data[3];
      float py = color2depth_.data[4] * x + color2depth_.data[5] * y + color2depth_.data[6] * z + color2depth_.data[7];
      float pz = color2depth_.data[8] * x + color2depth_.data[9] * y + color2depth_.data[10] * z + color2depth_.data[11];
      int u = int(px*color_intrinsic_.data[0] / pz + color_intrinsic_.data[2]);
      int v = int(py*color_intrinsic_.data[4] / pz + color_intrinsic_.data[5]);
 
      depthtemp.data[v*depth.cols + u] = (unsigned int)pz;
    }
  }
  for (int i = 0; i<depth.rows; i++)
  {
    unsigned short* temp = depthtemp.ptr<unsigned short>(i);
    unsigned short pre = temp[0];
    int preindex = 0;
    for (int j = 1; j<depth.cols; j++)
    {
      if (temp[j] != 0)
      {
        pre = temp[j];
        preindex = j;
        continue;
      }
      int after = j + 1;
      while (after<depth.cols&&temp[after] == 0)
        after++;
      if (after >= depth.cols)
        continue;
      temp[j] = (unsigned short)(pre + (temp[after] - pre) / (after - preindex)*(j - preindex));
    }
  }
  out = depthtemp.clone();
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calib_rgbd_with_laser");
//   ros::NodeHandle n;
  DepthColorRegistration t;
  ros::spin();
  return 1;
}