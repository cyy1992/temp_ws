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

#include "line_matcher.h"
#include <glog/logging.h>
using namespace std;
using namespace cv;
LineMatcher::LineMatcher()
{
  init();
}

LineMatcher::~LineMatcher()
{

}

void LineMatcher::init()
{
  line_img_ = cv::imread("/home/cyy/map/test01/1234.png",CV_LOAD_IMAGE_UNCHANGED);
  imshow("origin",line_img_);
  Mat dstImage;
  Mat dist;
//   threshold(line_img_, d1, 80, 255, CV_THRESH_BINARY);
//   imshow("origin2",d1);
  cv::distanceTransform(line_img_, dist, CV_DIST_L1, CV_DIST_MASK_PRECISE);
  normalize(dist, dstImage,0,1);

//   imshow("dst",dstImage);
//   waitKey(0);
  
  dist *= 100;
  pow(dist, 0.5, dist);
  
  imshow("t1",dist);
  waitKey(0);
  
  imshow("t1",dist);
  Mat dist32s, dist8u1, dist8u2;
  
  dist.convertTo(dist32s, CV_32S);
  dist32s &= Scalar::all(255);
  int k =0 ;
  float kk=0;
  for(int i=0;i<dist32s.rows;i++)       
  {
    uchar* data=dist32s.ptr<uchar>(i);
    for(int j=0;j<dist32s.cols;j++)
    {
      if(data[j] > kk)
        kk = data[j];
    }
  }
  LOG(WARNING) << kk;
  
//   imshow("t1",dist32s);
//   waitKey(0);
  dist32s.convertTo(dist8u1, CV_8U, 1, 0);
//   dist32s *= 1;
  imshow("t1",dist8u1);
  std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
  imwrite("/home/cyy/distance_transform.png",dist8u1,compression_params);
  waitKey(0);
//   dist32s += Scalar::all(255);
//   dist32s.convertTo(dist8u2, CV_8U);
// 
// //   Mat planes[] = {dist8u1, dist8u2, dist8u2};
// //   Mat dist8u;
// //   merge(planes, 3, dist8u);
//   imshow("t1",dist8u2);
//   waitKey(0);

}

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::GLOG_INFO);
  ros::init(argc,argv, "line_matcher");
  
  LineMatcher line;
  google::ShutdownGoogleLogging();
//   ros::spin();
  return 1;
}