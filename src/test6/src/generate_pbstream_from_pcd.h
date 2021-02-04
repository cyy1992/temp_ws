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

#ifndef GENERATEPBSTREAMFROMPCD_H
#define GENERATEPBSTREAMFROMPCD_H
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
class GeneratePbstreamFromPcd
{
  struct NodeData
  {
    cartographer::sensor::RangeData range_data;
    Eigen::VectorXf rotational_scan_matcher_histogram_in_gravity;
    Eigen::Quaterniond local_from_gravity_aligned;
    cartographer::transform::Rigid3d global_pose;
  };
  enum MAP_FORMART{SHOW_MAP,SAVE_MAP,PUB_MAP};
public:
  GeneratePbstreamFromPcd();
  ~GeneratePbstreamFromPcd();
  void getPbstream(const std::vector<Eigen::Vector3f>& points);
  std::shared_ptr<cartographer::mapping::Submap3D> PointsToSubmap(const std::vector<Eigen::Vector3f>& points);
  void WriteToPbstream(const std::vector<std::shared_ptr<cartographer::mapping::Submap3D>>& submaps);
  void writeSubmap3DToPbstream(const std::shared_ptr<cartographer::mapping::Submap3D> & submap, 
                               const std::string& save_path, const int& index);
  void saveShowImage(const std::string& path);
  
  
private:
  cartographer::io::SubmapSlice submapToSlice(const std::shared_ptr<cartographer::mapping::Submap3D> submap,
                     const cartographer::transform::Rigid3d& global_pose,
                     const bool& is_high_resolution);
  const cv::Mat getMatMap(const cartographer::io::PaintSubmapSlicesResult& painted_slices, MAP_FORMART format = SHOW_MAP);
};

#endif // GENERATEPBSTREAMFROMPCD_H
