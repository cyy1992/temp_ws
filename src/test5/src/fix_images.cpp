#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/opencv.hpp>
#include <stdio.h>  
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <iostream>
#include <string>

#include "dbg.h"
using namespace cv;  
using namespace std;
Mat image16ToImage8(const Mat& image16)
{
  Mat image8(image16.rows, image16.cols, CV_8UC1);
  for (int i = 0; i < image16.rows; i++)
    for (int j = 0; j < image16.cols; j++)
    {
      int value = image16.at<uint16_t>(i, j);
      if (value == 0)
        image8.at<uchar>(i, j) = 128;
      else
        image8.at<uchar>(i, j) = value / 128;
    } 
    
    return image8;
}
int main(int argc, char** argv)
{
  Eigen::Matrix3d a,b;
  a << 1,2,3,4,5,6,7,8,9;
  b << 1,2,3,4,5,6,7,9,9;
  dbg(b);
  dbg(a);
  return 1;
  Mat img1,img2;
  img1 = imread("/home/cyy/map/000111231205/frames/0/probability_grid.png",IMREAD_UNCHANGED);
  img2 = imread("/home/cyy/map/000111231205/mapGrow.png",IMREAD_UNCHANGED);
  imshow("origin_map",image16ToImage8(img1));
  imshow("mapGrow",img2);
  cout << img1.type() <<endl;
  cout << img2.type() <<endl;
  for (int i = 0; i < img1.rows; i++)
  {
    for (int j = 0; j < img1.cols; j++)
    {
      int img_delta = img2.at<Vec4b>(i, j)[0];
      int img_alpha = img2.at<Vec4b>(i, j)[3];
      uchar img_value;
      if(img_alpha > 0){
//         grey = img_alpha * 32766. / 255;

          //brush add
//           if(img2.at<cv::Vec4b>(i, j)[0] < 128)
//           {
//             
//           }
//           float temp = img_alpha * 1.0/255.0;
          float out_alpha, ori_alpha, dst_alpha;
          ori_alpha = 0;
          dst_alpha = 1 - img_alpha * 1.0/255.0;
          cout << img_alpha <<endl;
          out_alpha = ori_alpha + dst_alpha* (1 - ori_alpha);
//           img1.at<Vec4b>(i, j)[0] = (img1.at<Vec4b>(i, j)[0] * ori_alpha + dst_alpha * img_delta *(1-ori_alpha)) / out_alpha;
//           img1.at<Vec4b>(i, j)[1] = (img1.at<Vec4b>(i, j)[1] * ori_alpha + dst_alpha * img_delta *(1-ori_alpha)) / out_alpha;
//           img1.at<Vec4b>(i, j)[2] = (img1.at<Vec4b>(i, j)[2] * ori_alpha + dst_alpha * img_delta *(1-ori_alpha)) / out_alpha;
//           img1.at<Vec4b>(i, j)[0] = (img1.at<Vec4b>(i, j)[0] * (1-dst_alpha) + dst_alpha * img_delta);
//           img1.at<Vec4b>(i, j)[1] = (img1.at<Vec4b>(i, j)[1] * (1-dst_alpha) + dst_alpha * img_delta);
//           img1.at<Vec4b>(i, j)[2] = (img1.at<Vec4b>(i, j)[2] * (1-dst_alpha) + dst_alpha * img_delta);
          img1.at<uint16_t>(i,j) = img1.at<uint16_t>(i,j) * (1-dst_alpha) + dst_alpha * img_delta * 128;
        
      }

    }
  }
  
  imshow("temp",image16ToImage8(img1));
  waitKey(0);
  return 1;
}