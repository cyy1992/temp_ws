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

#include "point_cloud_trans.h"

PointcloudTrans::PointcloudTrans(const ros::NodeHandle& n):nh_(n)
{
  cloud1_sub_ = nh_.subscribe("/pointcloud_front",10, &PointcloudTrans::HandleCloudFront,this);
  cloud2_sub_ = nh_.subscribe("/pointcloud_back",10, &PointcloudTrans::HandleCloudBack,this);
  cloud1_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_front1",10);
  cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_back1",10);
}

PointcloudTrans::~PointcloudTrans()
{

}


  
void PointcloudTrans::HandleCloudFront(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  sensor_msgs::PointCloud2 point_cloud;
  point_cloud = (*msg);
  int range_size = point_cloud.width * point_cloud.height;
  if(range_size > 0)
  {
    sensor_msgs::PointCloud2Iterator<float> iter_x0(point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y0(point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z0(point_cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity0(point_cloud, "intensity");
//       sensor_msgs::PointCloud2Iterator<int> iter_intensity(point_cloud, "intensity");
//       sensor_msgs::PointCloud2Iterator<double> iter_intensity(point_cloud, "timestamp");
//      sensor_msgs::PointCloud points;
//      points.header = msg->header;
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header = msg->header;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.height = 1;
    cloud_msg.width = range_size;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32, 
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(range_size);
    sensor_msgs::PointCloud2Iterator<float> iter_x1(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y1(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z1(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity1(cloud_msg, "intensity");
    for(int i = 0; i < range_size; i++)
    {
      *iter_x1 = *iter_x0;
      *iter_y1 = *iter_y0;
      *iter_z1 = *iter_z0;
      *iter_intensity1 = *iter_intensity0;
      ++iter_x0;
      ++iter_y0;
      ++iter_z0;
      ++iter_intensity0;
      
      ++iter_x1;
      ++iter_y1;
      ++iter_z1;
      ++iter_intensity1;
    }
    cloud1_pub_.publish(cloud_msg);
  }
}


void PointcloudTrans::HandleCloudBack(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 point_cloud;
  point_cloud = (*msg);
  int range_size = point_cloud.width * point_cloud.height;
  if(range_size > 0)
  {
    sensor_msgs::PointCloud2Iterator<float> iter_x0(point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y0(point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z0(point_cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity0(point_cloud, "intensity");
//       sensor_msgs::PointCloud2Iterator<int> iter_intensity(point_cloud, "intensity");
//       sensor_msgs::PointCloud2Iterator<double> iter_intensity(point_cloud, "timestamp");
//      sensor_msgs::PointCloud points;
//      points.header = msg->header;
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header = msg->header;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.height = 1;
    cloud_msg.width = range_size;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32, 
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(range_size);
    sensor_msgs::PointCloud2Iterator<float> iter_x1(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y1(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z1(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity1(cloud_msg, "intensity");
    for(int i = 0; i < range_size; i++)
    {
      *iter_x1 = *iter_x0;
      *iter_y1 = *iter_y0;
      *iter_z1 = *iter_z0;
      *iter_intensity1 = *iter_intensity0;
      ++iter_x0;
      ++iter_y0;
      ++iter_z0;
      ++iter_intensity0;
      
      ++iter_x1;
      ++iter_y1;
      ++iter_z1;
      ++iter_intensity1;
    }
    cloud2_pub_.publish(cloud_msg);
  }

}
int main(int argc, char **argv)
{
//   google::InitGoogleLogging(argv[0]);
//   google::InstallFailureSignalHandler();
//   FLAGS_alsologtostderr = true;
//   FLAGS_colorlogtostderr = true;
  
  ros::init(argc, argv, "point_cloud_trans");

  ros::NodeHandle n;
  PointcloudTrans a(n);
  ros::spin();
  return 1;
}