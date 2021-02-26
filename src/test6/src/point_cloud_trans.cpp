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
  Eigen::Quaterniond q(0.689, 0,0,-0.725);
  cout << q.toRotationMatrix() <<endl;
  cloud1_sub_ = nh_.subscribe("/velodyne_points",10, &PointcloudTrans::HandleCloudFront,this);
  cloud2_sub_ = nh_.subscribe("/pointcloud_back",10, &PointcloudTrans::HandleCloudBack,this);
  cloud1_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_front1",10);
  cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_back1",10);
}

PointcloudTrans::~PointcloudTrans()
{

}

void PointcloudTrans::calibRollPitchZ(const geometry_msgs::TransformStamped& prior_transform, 
                                      const sensor_msgs::PointCloud2& cloud, 
                                      geometry_msgs::TransformStamped& result_transform)
{
  Eigen::Vector3d translation(prior_transform.transform.translation.x, 
                              prior_transform.transform.translation.y,
                              prior_transform.transform.translation.z);
  float init_z = prior_transform.transform.translation.z;
  double init_roll = 0, init_pitch =0,init_yaw = 0;
  tf::Matrix3x3(tf::Quaternion(prior_transform.transform.rotation.x,
                               prior_transform.transform.rotation.y, 
                               prior_transform.transform.rotation.z,
                               prior_transform.transform.rotation.w)).getRPY(init_roll, init_pitch, init_yaw);
  geometry_msgs::TransformStamped transform = prior_transform;
  int range_size = cloud.width * cloud.height;
  int max_points_sum = 0;
  for(int i = -10; i <10; i++)
  {
    float z = i * 0.01 + init_z;
    for(int j = -30; j < 30; j++)
    {
      float pitch = init_pitch + j * 0.001;
      for(int k = -30; k < 30; k++)
      {
        float roll = init_roll + k * 0.001;
        Eigen::Matrix3d matrix_tmp;
        matrix_tmp = (Eigen::AngleAxisd(init_yaw,Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
        Eigen::Quaterniond q_temp(matrix_tmp);
        transform.transform.translation.z = z;
        transform.transform.rotation.x = q_temp.x();
        transform.transform.rotation.y = q_temp.y();
        transform.transform.rotation.z = q_temp.z();
        transform.transform.rotation.w = q_temp.w();
        
        sensor_msgs::PointCloud2 temp_cloud;
        tf2::doTransform(cloud, temp_cloud, transform);
        
        sensor_msgs::PointCloud2Iterator<float> iter_z0(temp_cloud, "z");
        int points_sum = 0;
        for(int ii = 0; ii < range_size; ii++)
        {
          if(*iter_z0 > -0.015 && *iter_z0 < 0.015 )
            points_sum++;
          ++iter_z0;
        }
        if(points_sum > max_points_sum){
          max_points_sum = points_sum;
          result_transform = transform;
          cout << "max points sum update: " << max_points_sum <<", " << i <<" " << j <<" " << k<<endl;
          cout << z << ", " << roll << ", " << pitch <<endl;
        }
      }
    }
  }
}

  
void PointcloudTrans::HandleCloudFront(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  sensor_msgs::PointCloud2 point_cloud_origin;
  sensor_msgs::PointCloud2 point_cloud;
  point_cloud_origin = (*msg);
  int range_size = point_cloud_origin.width * point_cloud_origin.height;
  geometry_msgs::TransformStamped transformStamped;
  try {
      transformStamped = tfBuffer_.lookupTransform("base_footprint", msg->header.frame_id,
              ros::Time(0));
//       tf2::doTransform(point_cloud_origin, point_cloud, transformStamped);

//          pcl_ros::transformPointCloud("robotarm", cloud_in, cloud_out, tfListener);

  } catch (tf2::TransformException &ex) {
    return;
  }
  static bool flag = false;
  geometry_msgs::TransformStamped result_transform;
  if(!flag){
    cout << "start calib roll pitch z!" <<endl;
    calibRollPitchZ(transformStamped, point_cloud_origin, result_transform);
    flag = true;
    cout <<"done!" <<endl;
  }
  tf2::doTransform(point_cloud_origin, point_cloud, transformStamped);
  point_cloud.header.frame_id = "base_footprint";
  point_cloud.header.stamp = ros::Time::now();
//   cloud1_pub_.publish(point_cloud);
  if(range_size > 0 && 1)
  {
    sensor_msgs::PointCloud2Iterator<float> iter_x0(point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y0(point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z0(point_cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity0(point_cloud, "intensity");
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
      if(*iter_z0 > -0.02 && *iter_z0 < 0.02)
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
  sensor_msgs::PointCloud2 point_cloud;
  point_cloud_origin = (*msg);
  
  geometry_msgs::TransformStamped transformStamped;
  try {
      transformStamped = tfBuffer_.lookupTransform("base_footprint", msg->header.frame_id,
              ros::Time(0));
      tf2::doTransform(point_cloud_origin, point_cloud, transformStamped);

//          pcl_ros::transformPointCloud("robotarm", cloud_in, cloud_out, tfListener);

  } catch (tf2::TransformException &ex) {
    return;
  }
  int range_size = point_cloud.width * point_cloud.height;
  if(range_size > 0)
  {
    sensor_msgs::PointCloud2Iterator<float> iter_x0(point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y0(point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z0(point_cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity0(point_cloud, "reflectivity");
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
      if(*iter_intensity0 < 300)
      {
        ++iter_x0;
        ++iter_y0;
        ++iter_z0;
        ++iter_intensity0;
        continue;
      }
      *iter_x1 = *iter_x0;
      *iter_y1 = *iter_y0;
      *iter_z1 = *iter_z0;
      if(*iter_z0 > -0.02 && *iter_z0 < 0.02 )
        *iter_intensity1 = 500;
      else 
        *iter_intensity1 = 0.;
//       *iter_intensity1 = *iter_intensity0;
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