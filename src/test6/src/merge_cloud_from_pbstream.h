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

#ifndef MERGECLOUDFROMPBSTREAM_H
#define MERGECLOUDFROMPBSTREAM_H

#include <cartographer/io/submap_painter.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
// #include <vtkPolyData.h>
// #include <vtkSmartPointer.h>
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/conversions.h>
#include <opencv2/opencv.hpp>
#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer_ros_msgs/ReferenceLocationFromMap.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/PointCloud.h"
#include "cartographer_ros_msgs/SubmapImagesServer.h" 
#include "cartographer_ros_msgs/LandmarkListServer.h"
#include "visualization_msgs/MarkerArray.h"
#include "cartographer_ros_msgs/UpdateLandmarkServer.h"
#include <Eigen/Eigen>
#include <cartographer/transform/rigid_transform.h>
#include "cartographer/sensor/range_data.h"
#include <cartographer/io/submap_painter.h>
#include <cartographer/io/image.h>

#include <eigen_conversions/eigen_msg.h>
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "vtr_msgs/GlobalLocalizationPose.h"
#include "std_msgs/Empty.h"
#include "cartographer_ros/node_options.h"
#include <ros/package.h>
#include <cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/filters/voxel_grid.h>
#include <cartographer_ros_msgs/CloudWithPose.h>
class MergeCloudFromPbstream
{
public:
  MergeCloudFromPbstream(const ros::NodeHandle& n);
  ~MergeCloudFromPbstream();
  bool readFromPbstream(std::string filename);
private:
  void pubCloud(const ros::WallTimerEvent& unused_timer_event);
  void handleTopics(const geometry_msgs::PoseStampedConstPtr& pose_msg,const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void handleCloudWithPoseMsg(const cartographer_ros_msgs::CloudWithPoseConstPtr& cloud_msg);
  ros::NodeHandle nh_;
  ros::Publisher cloud_pub_;
  ros::WallTimer timer_pub_;
  ros::Subscriber cloud_with_pose_sub_;
  cartographer::sensor::PointCloud high_resolution_cloud_;
  std::queue<cartographer::mapping::NodeId> frame_ids_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
  bool initialised_;
  
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> poseCloudSync;
  message_filters::Synchronizer<poseCloudSync> *topicsSync_;
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter_;
  
  cartographer::mapping::MapById<cartographer::mapping::NodeId, cartographer::transform::Rigid3d> node_poses_;
  cartographer::mapping::MapById<cartographer::mapping::NodeId, cartographer::sensor::PointCloud> points_with_id_;
};

#endif // MERGECLOUDFROMPBSTREAM_H
