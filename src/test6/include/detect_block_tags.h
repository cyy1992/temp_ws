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

#ifndef DETECTTAGS_H
#define DETECTTAGS_H
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

#include <ros/ros.h>
#include <iostream>
#include <string>

#include "AprilTagsOld/ImageProjector.h"
extern "C" {
#include "AprilTagsOld/apriltag.h"
#include "AprilTagsOld/tag36h10.h"
#include "AprilTagsOld/tag36h11.h"
}
class detectBlockTags
{
public:
  detectBlockTags(const ros::NodeHandle& nh,const std::string& cam_type);
  ~detectBlockTags();
private:
  void PerceptionCB(const boost::shared_ptr<sensor_msgs::Image const>& sensor_msg);
  bool tagToPose2(const cv::Mat& srcImg, int& tag_id, int& detect_tags);
  
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_;
  ros::Publisher perception_pub_;
  std::shared_ptr<ImageProjector> p_ip_;
  std::string camera_type_;
  int tag_type_;
  cv::Mat intrinsic_;
  cv::Mat distortion_;
  apriltag_family_t* tf_;
  apriltag_detector_t* td_;
  
  int tags_per_line_;
  float grid_size_, tag_size_;
  
  bool init_param_;
  int tag_id_;
  cv::Mat mask_img_;
  cv::Mat undist_map1_;
  cv::Mat undist_map2_;
  
  bool show_color_image_;
};

#endif // DETECTTAGS_H
