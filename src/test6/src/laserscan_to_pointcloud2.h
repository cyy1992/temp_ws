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

#ifndef LASERSCANTOPOINTCLOUD2_H
#define LASERSCANTOPOINTCLOUD2_H
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class LaserscanToPointcloud2
{
  struct PointCloudWithIntensity{
    Eigen::Vector3f position;

    float intensity;
  };

public:
  LaserscanToPointcloud2(const ros::NodeHandle& n);
  ~LaserscanToPointcloud2();
  void handleLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg);
  sensor_msgs::PointCloud2 PreparePointCloud2Message(const ros::Time timestamp,
                                                   const std::string& frame_id,
                                                   const int num_points);
  sensor_msgs::PointCloud2 ToPointCloud2Message(
    const ros::Time timestamp, const std::string& frame_id,
    const std::vector<PointCloudWithIntensity>& point_cloud);
private:
  ros::NodeHandle nh_;
  ros::Subscriber laser_sub_;
  ros::Publisher pointcloud2_pub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
};

#endif // LASERSCANTOPOINTCLOUD2_H
