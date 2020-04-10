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

#include "laserscan_to_pointcloud2.h"
#include <tf2/convert.h>

LaserscanToPointcloud2::LaserscanToPointcloud2(const ros::NodeHandle& n):nh_(n), tfBuffer_(ros::Duration(10.)),
      tfListener_(tfBuffer_)
{
  laser_sub_ = nh_.subscribe("/scan_emma_nav_front",5,&LaserscanToPointcloud2::handleLaserScan,this);
  pointcloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/scan_matched_points2",10);
}

LaserscanToPointcloud2::~LaserscanToPointcloud2()
{

}
float GetFirstEcho(const float& echo) {
  return echo;
}
sensor_msgs::PointCloud2 LaserscanToPointcloud2::PreparePointCloud2Message(const ros::Time timestamp,
                                                   const std::string& frame_id,
                                                   const int num_points) {
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = timestamp;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = num_points;
  msg.fields.resize(4);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "i";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.is_bigendian = false;
  msg.point_step = 20;
  msg.row_step = 20 * msg.width;
  msg.is_dense = true;
  msg.data.resize(20 * num_points);
  return msg;
}
sensor_msgs::PointCloud2 LaserscanToPointcloud2::ToPointCloud2Message(
    const ros::Time timestamp, const std::string& frame_id,
    const std::vector<PointCloudWithIntensity>& point_cloud) {
  auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());
  ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
  for (const PointCloudWithIntensity& point : point_cloud) {
    stream.next(point.position.x());
    stream.next(point.position.y());
    stream.next(point.position.z());
    stream.next(point.intensity);
    stream.next(1);
  }
  // LOG(INFO) <<"intensity:" <<  point_cloud[0].intensity <<std::endl;
  return msg;
}
void LaserscanToPointcloud2::handleLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
  sensor_msgs::LaserScan msg = *laser_msg;
  std::vector<PointCloudWithIntensity> point_cloud;
  sensor_msgs::PointCloud2 point_cloud_msg;
  
  ros::Time t = msg.header.stamp;
  float angle = msg.angle_min;
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const auto& echoes = msg.ranges[i];
    const float first_echo = msg.ranges[i];
    if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
      const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
      if (msg.intensities.size() > 0) {
        const auto& echo_intensities = msg.intensities[i];
        double intensity = GetFirstEcho(echo_intensities);
        const float intensity_tmp = GetFirstEcho(msg.intensities[i]);
        Eigen::Vector3f position = rotation * (first_echo * Eigen::Vector3f::UnitX());
        PointCloudWithIntensity point;
        point.position = position;
        point.intensity = intensity_tmp;
        point_cloud.push_back(point);
      
      } 
      
    }
    angle += msg.angle_increment;
  }
  point_cloud_msg = ToPointCloud2Message(t, "laser_link", point_cloud);
  geometry_msgs::TransformStamped laser2odom_tf;
  ros::Rate rate(100);
  while(1){
    try
    {
      
      laser2odom_tf = tfBuffer_.lookupTransform("odom", "laser_link", t);
      break;
    }
    catch(tf::TransformException ex)
    {
  //     std::cout << "can not get tf"<<std::endl;
      rate.sleep();
    }
  }
  sensor_msgs::PointCloud2 pointcloud_in_odom;
  tf2::doTransform(point_cloud_msg,pointcloud_in_odom,laser2odom_tf);
  pointcloud_in_odom.header.frame_id = "odom";
  pointcloud2_pub_.publish(pointcloud_in_odom);
}

int main(int argc, char** argv)
{
  ::ros::init(argc, argv, "LaserscanToPointcloud2");
  ros::NodeHandle n;
  LaserscanToPointcloud2 temp(n);
  ros::spin();
  return 1;
}