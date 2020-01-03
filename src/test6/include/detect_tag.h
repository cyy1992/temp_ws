#ifndef APRILTAG_FAST_H_
#define APRILTAG_FAST_H_

#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <ros/param.h>
#include <thread>
#include <eigen_conversions/eigen_msg.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <AprilTags/TagDetector.h>
using namespace std;
class detect_tag
{
public:
  detect_tag(ros::NodeHandle nh);
  ~detect_tag();
  bool tagToPose(const cv::Mat& srcImg,const string& file_name);

private:
  void imgCb(const cv::Mat& srcImg/*const sensor_msgs::ImageConstPtr& img_msg*/,const string& file_name);
	void getKey();

  ros::NodeHandle nh_;
  ros::Subscriber img_sub_;
  ros::Publisher landmarks_pub_;

  cv::Mat intrinsic_;
  cv::Mat distortion_,distortion_zero_;
  Eigen::Affine3d camera2base_;

  double dm_code_size_;
	double dm_grid_size_;
	
  bool connected_;
  ros::Time detection_time_start_;
  int detection_cnt_;
  bool isMapping_;
  bool cam_towards_down_;
  std::string camera_topic_; 
	
	int width_;
  int height_;
  int projWidth_;//376
  int projHeight_;//240
  double objWidth_;//135.36
  double objHeight_;//86.4
  int tag_cols_;
	string tf_name_world_,tf_name_camera_;
	int name_id_;
  
  double tags_size_[6];
  double grids_size_[6];
  AprilTags::TagDetector tag_detector_;
// 	std::vector<std::vector<cv::Point3f> > points3d_;
// 	std::vector<std::vector<cv::Point2d> > points2d_;
	
};

#endif // APRILTAG_FAST_H_
