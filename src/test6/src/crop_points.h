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

#ifndef CROPPOINTS_H
#define CROPPOINTS_H
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "test6/SetPose.h"
#include <std_srvs/Empty.h>
#include <pointmatcher/Functions.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/Timer.h>
#include <pointmatcher_ros/point_cloud.h>
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Matches Matches;
class CropPoints
{
public:
  CropPoints(const ros::NodeHandle& n);
  ~CropPoints();
  void handlePointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void pubCropPoints(const ::ros::WallTimerEvent& unused_timer_event);
  bool setTfPose(test6::SetPose::Request& request, test6::SetPose::Response& response);
  bool savePoints(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
private:
  
  ros::NodeHandle nh_;
  ros::WallTimer wall_timer_;
  ros::Subscriber pointcloud_sub_;
  ros::ServiceServer set_tf_srv_;
  ros::ServiceServer save_points_srv_;
  ros::Publisher cloud_pub_;
  Eigen::Vector3d tf_translation_;
  Eigen::Quaterniond tf_quaternion_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<Eigen::Vector3d> origin_points_;
  std::vector<Eigen::Vector3d> vertex_points_;
  geometry_msgs::TransformStamped stamped_transform_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  float max_x_,max_y_,max_z_;
  sensor_msgs::PointCloud2 cloud_msg;
};

#endif // CROPPOINTS_H
