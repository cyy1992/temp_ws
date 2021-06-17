#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/image_encodings.h"     
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <iostream>

using namespace cv;
using namespace std;


int rate  = 30;
//Mat globalImage(Size(640,480),CV_8UC3);
int chess_num =0;

void callback_raw_image(
  const sensor_msgs::ImageConstPtr &img_l_msg,
  const sensor_msgs::ImageConstPtr &img_r_msg)
{
  ROS_INFO("save images with header: %f", img_l_msg->header.stamp.toSec());
  
  
  try
  {
    cv_bridge::CvImagePtr img_l_ptr = cv_bridge::toCvCopy(img_l_msg, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImagePtr img_r_ptr = cv_bridge::toCvCopy(img_r_msg, sensor_msgs::image_encodings::BGR8);
    
    cv::Mat img_ll=img_l_ptr->image;
    cv::Mat img_rr=img_r_ptr->image;
    
    cv::Mat img_show(img_ll.rows,img_ll.cols+img_rr.cols,img_ll.type());
    img_ll.copyTo(img_show.colRange(0,img_ll.cols));
    img_rr.copyTo(img_show.colRange(img_ll.cols,img_ll.cols+img_rr.cols));
    cv::imshow("show",img_show);
    
    if(cv::waitKey(33)=='s')
    {
      char path[30];
      sprintf(path, "%s%d%s", "/home/cyy/left/", chess_num, ".png");
      imwrite(path,img_ll);
      sprintf(path, "%s%d%s", "/home/cyy/right/", chess_num, ".png");
      imwrite(path,img_rr);
      chess_num++;
    }
    
  }
  catch(cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge Exception %s",e.what());
  }
  
}

void imageCallback1(const sensor_msgs::ImageConstPtr& tem_msg)
{
  ROS_INFO("save images with header: %f", tem_msg->header.stamp.toSec());
  
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(tem_msg, "bgr8");
    
    cv::Mat chess = cv_ptr->image;
    cv::imshow("color", chess);
    
    if(waitKey(33) == 's')
    {
      char path[30];
      sprintf(path, "%s%d%s", "/home/oatrc/Desktop/pic/pic", chess_num++, ".jpg");
      imwrite(path,chess);
    }
    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", tem_msg->encoding.c_str());
    ROS_ERROR("cv_bridge Exception %s",e.what());
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "SavePicture");
  ros::NodeHandle nh;
  
  //image_transport::Subscriber sub1 = it.subscribe("/camera/color/image_raw",1,imageCallback1);
  
  message_filters::Subscriber<sensor_msgs::Image> sub_img_l(nh, "/mynteye/left/image_raw", 100);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_r(nh, "/mynteye/right/image_raw", 100);
  
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy_img;
  message_filters::Synchronizer<MySyncPolicy_img> sync_img(MySyncPolicy_img(1000), sub_img_l, sub_img_r);
  sync_img.registerCallback(boost::bind(&callback_raw_image, _1, _2));
  
  ros::Rate loop_rate(30);
  while (nh.ok()) {
    //pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  
}

