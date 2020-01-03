/*
 * Copyright 2019 <copyright holder> <email>
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

#include "pub_submap_points.h"

PubSubmapPoints::PubSubmapPoints(const ros::NodeHandle& n):nh_(n)
{
  sub_ = nh_.subscribe("/submap_lasermark",10,&PubSubmapPoints::handleSubmapLasermark,this);
  points_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/temp_pointcloud",3);
}
void PubSubmapPoints::handleSubmapLasermark(const cartographer_ros_msgs::SubmapLasermark::ConstPtr& msg)
{
  sensor_msgs::PointCloud points = msg->submaps.point_cloud;
  points.header.frame_id = "base_footprint";
  points_pub_.publish(points);
}


int main(int argc, char** argv)
{
  ros::init(argc,argv,"aa");
  ros::NodeHandle n;
  PubSubmapPoints a(n);
  ros::spin();
  return 1;
}