#ifndef APRILTAG_FAST_H_
#define APRILTAG_FAST_H_

#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <thread>
#include <opencv2/opencv.hpp>
#include "ImageProjector.h"
#include "AprilTags/apriltag_family.h"
using namespace std;
class detect_tag
{
public:
  detect_tag();
  ~detect_tag();
  bool tagToPose(const cv::Mat& srcImg);

private:
  void imgCb(const cv::Mat& srcImg);
	void getKey();

  cv::Mat intrinsic_;
  cv::Mat distortion_,distortion_zero_;
  Eigen::Affine3d camera2base_;

  double dm_code_size_;
	double dm_grid_size_;
	
  bool connected_;
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
//   AprilTags::TagDetector tag_detector_;
	ImageProjector* p_ip_ ;
  apriltag_family_t *tf_;
  apriltag_detector_t *td_;
};

#endif // APRILTAG_FAST_H_
