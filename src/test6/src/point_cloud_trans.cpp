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
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
using namespace std;
PointcloudTrans::PointcloudTrans(const ros::NodeHandle& n):nh_(n),tfBuffer_(ros::Duration(20.)),tfListener_(tfBuffer_)
{
//   cloud1_sub_ = nh_.subscribe("/pointcloud_front",10, &PointcloudTrans::HandleCloudFront,this);
  cloud2_sub_ = nh_.subscribe("/pointcloud_back",10, &PointcloudTrans::HandleCloudBack,this);
  cloud1_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_front1",10);
  cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_back1",10);
  cloud_all_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/merge_back1",10);
  ros::Rate rate(10);
//   while(1){
//     
// //     try {
// //         lidar2base_tf_ = tfBuffer_.lookupTransform("base_footprint", "lidar_link2",
// //                 ros::Time(0));
// // //         tf2::doTransform(point_cloud_origin, point_cloud, transformStamped);
// // 
// //   //          pcl_ros::transformPointCloud("robotarm", cloud_in, cloud_out, tfListener);
// //         break;
// // 
// //     } catch (tf2::TransformException &ex) {
// //       rate.sleep();
// //     }
//   }
  down_sample_.setLeafSize(0.02, 0.02, 0.02);
  points_.reset(new pcl::PointCloud<PointType>());
}

PointcloudTrans::~PointcloudTrans()
{

}


  
void PointcloudTrans::HandleCloudFront(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  sensor_msgs::PointCloud2 point_cloud_origin;
  sensor_msgs::PointCloud2 point_cloud;
  point_cloud_origin = (*msg);
  int range_size = point_cloud_origin.width * point_cloud_origin.height;
  geometry_msgs::TransformStamped transformStamped;
  try {
      transformStamped = tfBuffer_.lookupTransform("odom1", "lidar_link2",
              ros::Time(0));
      tf2::doTransform(point_cloud_origin, point_cloud, transformStamped);

//          pcl_ros::transformPointCloud("robotarm", cloud_in, cloud_out, tfListener);

  } catch (tf2::TransformException &ex) {
    return;
  }

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
//     cloud_msg.header = msg->header;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "base_footprint";
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
      if(*iter_z0 > -0.02 && *iter_z0 < 0.02 )
        *iter_intensity1 = 500;
      else 
        *iter_intensity1 = 0.;
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
  sensor_msgs::PointCloud2 point_cloud_origin;
  sensor_msgs::PointCloud2 point_cloud,point_cloud_in_base;
  point_cloud_origin = (*msg);
  ros::Time cur_time = msg->header.stamp + ros::Duration(1610179077.576312);
  geometry_msgs::TransformStamped lidar2odom, lidar2base;
  ros::Rate rate(10);
  cout << cur_time  <<"," << ros::Time::now()<<endl;
  while(1){
    try {
        lidar2odom = tfBuffer_.lookupTransform("odom1", msg->header.frame_id,
                cur_time);
        lidar2base = tfBuffer_.lookupTransform("base_footprint", msg->header.frame_id,
                cur_time);
        tf2::doTransform(point_cloud_origin, point_cloud, lidar2odom);
        tf2::doTransform(point_cloud_origin, point_cloud_in_base, lidar2base);
        break;
  //          pcl_ros::transformPointCloud("robotarm", cloud_in, cloud_out, tfListener);

    } catch (tf2::TransformException &ex) {
      rate.sleep();
    }
  }
  int range_size = point_cloud.width * point_cloud.height;
  if(range_size > 0)
  {
    sensor_msgs::PointCloud2Iterator<float> iter_x0(point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y0(point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z0(point_cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity0(point_cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float> iter_x_base(point_cloud_in_base, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y_base(point_cloud_in_base, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z_base(point_cloud_in_base, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i_base(point_cloud_in_base, "intensity");
//       sensor_msgs::PointCloud2Iterator<int> iter_intensity(point_cloud, "intensity");
//       sensor_msgs::PointCloud2Iterator<double> iter_intensity(point_cloud, "timestamp");
//      sensor_msgs::PointCloud points;
//      points.header = msg->header;
    sensor_msgs::PointCloud2 cloud_msg;
//     cloud_msg.header = msg->header;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "base_footprint2";
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
    vector<Eigen::Vector3d> temp_points;
    temp_points.reserve(range_size);
    float ref_x,ref_y,ref_z;
    ref_x = -11.0;ref_y = 0; ref_z = 1.5;
    float size_x,size_y,size_z;
    size_x = 4.;
    size_y = 3.5;
    size_z = 1.5;
    for(int i = 0; i < range_size; i++)
    {
      float temp_x = *iter_x_base;
//       if(i < 10 )
//         cout << temp_x <<endl;
      if(temp_x > -10){
        ++iter_x0;
        ++iter_y0;
        ++iter_z0;
        ++iter_intensity0;
        
        ++iter_x1;
        ++iter_y1;
        ++iter_z1;
        ++iter_intensity1;
        
        ++iter_x_base;
        ++iter_y_base;
        ++iter_z_base;
        ++iter_i_base;
//         cout << temp_x <<endl;
        continue;
      }
      *iter_x1 = *iter_x_base;
      *iter_y1 = *iter_y_base;
      *iter_z1 = *iter_z_base;
      if(*iter_z0 > -0.02 && *iter_z0 < 0.02 )
        *iter_intensity1 = 500;
      else 
        *iter_intensity1 = 0.;
//       *iter_intensity1 = *iter_intensity0;
//       temp_points.push_back(Eigen::Vector3d(*iter_x0, *iter_y0, *iter_z0));
      
      if((*iter_x0) > (ref_x - size_x )&& (*iter_x0) <  (ref_x + size_x)&&
          (*iter_y0) > (ref_y - size_y)&& (*iter_y0) < (ref_y + size_y) &&
          (*iter_z0) > (ref_z - size_z )&& (*iter_z0) < (ref_z + size_z))
      {
        PointType p;
        p.x = *iter_x0;
        p.y = *iter_y0;
        p.z = *iter_z0;
        points_->points.push_back(p);
      }
      ++iter_x0;
      ++iter_y0;
      ++iter_z0;
      ++iter_intensity0;
      
      ++iter_x1;
      ++iter_y1;
      ++iter_z1;
      ++iter_intensity1;
      
       ++iter_x_base;
        ++iter_y_base;
         ++iter_z_base;
        ++iter_i_base;
    }
//     cloud_msg.header.frame_id = "odom1";
    cloud_msg.header.stamp = cur_time;
    cloud2_pub_.publish(cloud_msg);
    
    static int kk = 0;
    
//     cloud_points_.insert(cloud_points_.end(), temp_points.begin(), temp_points.end());
    if(kk%2 == 0)
    {
      sensor_msgs::PointCloud2 all_cloud_msg;
      
//       all_cloud_msg.height = 1;
//       all_cloud_msg.width = cloud_points_.size();
// 
//       sensor_msgs::PointCloud2Modifier modifier2(all_cloud_msg);
//       modifier2.setPointCloud2Fields(3,
//                                     "x", 1, sensor_msgs::PointField::FLOAT32,
//                                     "y", 1, sensor_msgs::PointField::FLOAT32, 
//                                     "z", 1, sensor_msgs::PointField::FLOAT32);
//       modifier2.resize(cloud_points_.size());
//       sensor_msgs::PointCloud2Iterator<float> iter_x1(all_cloud_msg, "x");
//       sensor_msgs::PointCloud2Iterator<float> iter_y1(all_cloud_msg, "y");
//       sensor_msgs::PointCloud2Iterator<float> iter_z1(all_cloud_msg, "z");
//       
//       for(int j = 0; j < cloud_points_.size(); j++)
//       {
//         if(cloud_points_[j].x() > ref_x - size_x && cloud_points_[j].x() < ref_x + size_x&&
//           cloud_points_[j].y() > ref_y - size_y && cloud_points_[j].y() < ref_y + size_y &&
//           cloud_points_[j].z() > ref_z - size_z && cloud_points_[j].z() < ref_z + size_z){
//         *iter_x1 = cloud_points_[j].x();
//         *iter_y1 = cloud_points_[j].y();
//         *iter_z1 = cloud_points_[j].z();
//         
//         ++iter_x1;
//         ++iter_y1;
//         ++iter_z1;
//             
//           }
//       }
//       
// //       cloud_points_.clear();
      pcl::PointCloud<PointType> down_sample_points;
      
      down_sample_.setInputCloud(points_);
      down_sample_.filter(down_sample_points);
      pcl::toROSMsg(down_sample_points, all_cloud_msg);
      all_cloud_msg.header.frame_id = "odom1";
      all_cloud_msg.header.stamp = cur_time;
      cloud_all_pub_.publish(all_cloud_msg);
    }
    kk++;
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