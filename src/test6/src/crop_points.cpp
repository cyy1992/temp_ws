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

#include "crop_points.h"
#include <pcl/visualization/vtk.h>
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/common/io.h>
#include <pcl/impl/point_types.hpp>
using namespace std;
CropPoints::CropPoints(const ros::NodeHandle& n):nh_(n),tfBuffer_(ros::Duration(20.)),tfListener_(tfBuffer_)
{
  pointcloud_sub_ = nh_.subscribe("/merge_back1", 5, &CropPoints::handlePointcloud,this);
  wall_timer_ = nh_.createWallTimer(::ros::WallDuration(0.2), &CropPoints::pubCropPoints, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/crop_cloud2",10);
  polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/crop_polygon",10);
  set_tf_srv_ = nh_.advertiseService("set_pose",&CropPoints::setTfPose,this);
  save_points_srv_ = nh_.advertiseService("save_points",&CropPoints::savePoints,this);
  vertex_points_.push_back(Eigen::Vector3d(-3.0895,-1.8597,-0.79852));
  vertex_points_.push_back(Eigen::Vector3d(-3.5712,-1.2166,-0.75));
  
  vertex_points_.push_back(Eigen::Vector3d(-3.5712,-1.2166,-0.75));
  vertex_points_.push_back(Eigen::Vector3d(-4.9779,-2.1997,-0.76));
  
  vertex_points_.push_back(Eigen::Vector3d(-4.9779,-2.1997,-0.76));
  vertex_points_.push_back(Eigen::Vector3d(-4.5411,-1.9095, -0.04));
  
  stamped_transform_.header.frame_id = "odom";
  stamped_transform_.child_frame_id = "crop_points_link";
  stamped_transform_.transform.translation.x = -3.3;
  stamped_transform_.transform.translation.y = -1.5;
  stamped_transform_.transform.translation.z = -0.4;
  stamped_transform_.transform.rotation.x = 0;
  stamped_transform_.transform.rotation.y = 0;
  stamped_transform_.transform.rotation.z = 0;
  stamped_transform_.transform.rotation.w = 1;
  max_x_ = 0.5;max_y_ = 0.5; max_z_ = 0.5;
}

CropPoints::~CropPoints()
{

}

void CropPoints::handlePointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  sensor_msgs::PointCloud2 point_cloud;
  sensor_msgs::PointCloud2 origin_point_cloud;
  point_cloud = (*msg);
  ros::Rate rate(10);
  int k = 0; 
//   while(1){
//     geometry_msgs::TransformStamped transformStamped;
//     try {
//         transformStamped = tfBuffer_.lookupTransform("base_footprint", msg->header.frame_id,
//                 ros::Time(0));
//         tf2::doTransform(origin_point_cloud, point_cloud, transformStamped);
//         break;
//   //          pcl_ros::transformPointCloud("robotarm", cloud_in, cloud_out, tfListener);
// 
//     } catch (tf2::TransformException &ex) {
//       rate.sleep();
//       if(k++ > 10){
//         cout << "err: can not get tf" <<endl;
//         return;
//       }
//     }
//   }
  int range_size = point_cloud.width * point_cloud.height;
  if(!origin_points_.empty())
    return;
  if(range_size > 0)
  {
    sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud, "z");
//     vector<Eigen::Vector3d> vec;
//     for(int i =0; i < 3; i++)
//     {
//       vec.push_back(vertex_points_[2*i+1] - vertex_points_[2*i]);
//       cout << vec[i] <<endl;
//     }
    
    for(int i = 0; i < range_size; i++)
    {
      Eigen::Vector3d point;
      point(0) = *iter_x;
      point(1) = *iter_y;
      point(2) = *iter_z;
            ++iter_x;
      ++iter_y;
      ++iter_z;
      if(isnan(point(0)) || isnan(point(1)) || isnan(point(2)))
        continue;
//       bool flag = true;
//       for(int j =0; j < 3; j++)
//       {
//         Eigen::Vector3d vec1 = point - vertex_points_[2*j];
//         Eigen::Vector3d vec2 = point - vertex_points_[2*j+1];
//         
//         double temp1 = vec1.dot(vec[j]);
//         double temp2= vec2.dot(vec[j]);
//         if(temp1 * temp2 > 0)
//           flag = false;
//       }
      origin_points_.push_back(point);
//       if(!flag)
//         continue;
// //       cout << point.transpose() <<endl;
//       points_.push_back(point);
    }
  }
  cout << "done" <<endl;
}

void CropPoints::pubCropPoints(const ::ros::WallTimerEvent& unused_timer_event)
{
  if(points_.empty())
  {
    Eigen::Quaterniond inv_quaternion = tf_quaternion_.conjugate();
    Eigen::Vector3d inv_translation = -(inv_quaternion * tf_translation_);
    for(auto point:origin_points_)
    {
      Eigen::Vector3d new_point = inv_quaternion * point + inv_translation;
      if(new_point(0) > -max_x_ && new_point(0) < max_x_ &&
        new_point(1) > -max_y_ && new_point(1) < max_y_ &&
        new_point(2) > -max_z_ && new_point(2) < max_z_)
      {
        points_.push_back(new_point);
      }
    }
    cloud_msg.header.frame_id = "crop_points_link";
    cloud_msg.height = 1;
    cloud_msg.width = points_.size();
  //   cout << points_.size() <<endl;
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32, "z",
                                  1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(points_.size());
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    
    for(const Eigen::Vector3d& point : points_)
    {
      *iter_x = point.x();
      *iter_y = point.y();
      *iter_z = point.z();
      ++iter_x;
      ++iter_y;
      ++iter_z;
      pcl::PointXYZ p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      save_cloud_.points.push_back(p);
    }
    
  }
  stamped_transform_.header.stamp = ros::Time::now();
  obj_polygon_.header.frame_id = "crop_points_link";
  obj_polygon_.header.stamp = ros::Time::now();
  polygon_pub_.publish(obj_polygon_);
  tf_broadcaster_.sendTransform(stamped_transform_);
  cloud_pub_.publish(cloud_msg);
}

bool CropPoints::setTfPose(test6::SetPose::Request& request, test6::SetPose::Response& response)
{
  tf_translation_(0) = request.pose.position.x;
  tf_translation_(1) = request.pose.position.y;
  tf_translation_(2) = request.pose.position.z;
  Eigen::Matrix3d matrix_tmp;
  matrix_tmp = (Eigen::AngleAxisd(request.pose.orientation.w,Eigen::Vector3d::UnitZ()) 
        * Eigen::AngleAxisd(0,Eigen::Vector3d::UnitY()) 
        * Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())).toRotationMatrix();
  Eigen::Quaterniond q(matrix_tmp);
  max_x_ = request.pose.orientation.x;
  max_y_ = request.pose.orientation.y;
  max_z_ = request.pose.orientation.z;
//   Eigen::Quaterniond q(request.pose.orientation.w, 
//                        request.pose.orientation.x,
//                        request.pose.orientation.y,
//                        request.pose.orientation.z);
  tf_quaternion_ = q;
  
  stamped_transform_.transform.translation.x = tf_translation_(0);
  stamped_transform_.transform.translation.y = tf_translation_(1);
  stamped_transform_.transform.translation.z = tf_translation_(2);
  stamped_transform_.transform.rotation.x = tf_quaternion_.x();
  stamped_transform_.transform.rotation.y = tf_quaternion_.y();
  stamped_transform_.transform.rotation.z = tf_quaternion_.z();
  stamped_transform_.transform.rotation.w = tf_quaternion_.w();
  points_.clear();
  obj_polygon_.polygon.points.clear();
  geometry_msgs::Point32 point;
  point.x = max_x_;
  point.y = max_y_;
  point.z = max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = max_x_;
  point.y = -max_y_;
  point.z = max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = -max_x_;
  point.y = -max_y_;
  point.z = max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = -max_x_;
  point.y = max_y_;
  point.z = max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = max_x_;
  point.y = max_y_;
  point.z = max_z_;
  obj_polygon_.polygon.points.push_back(point);
  
  point.x = max_x_;
  point.y = max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);

  point.x = max_x_;
  point.y = -max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = max_x_;
  point.y = -max_y_;
  point.z = max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = max_x_;
  point.y = -max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);
  
  point.x = -max_x_;
  point.y = -max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = -max_x_;
  point.y = -max_y_;
  point.z = max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = -max_x_;
  point.y = -max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);
  
  point.x = -max_x_;
  point.y = max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = -max_x_;
  point.y = max_y_;
  point.z = max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = -max_x_;
  point.y = max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);
  
  point.x = max_x_;
  point.y = max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);
  
  
  return true;
}

bool CropPoints::savePoints(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  std::unique_ptr<DP> cloud(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud_msg)));
  cloud->save("/home/cyy/finalMap.vtk");
  
  pcl::io::savePLYFile("/home/cyy/finalMap.ply", save_cloud_); 
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "temp_test");
  ros::NodeHandle n;
  CropPoints crop(n);
  ros::spin();
  return 1;
}