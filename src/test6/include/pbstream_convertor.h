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

#ifndef PBSTREAMCONVERTOR_H
#define PBSTREAMCONVERTOR_H
#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "cartographer/mapping/proto/map_builder_options.pb.h"
// #include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/transform/transform.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_2d.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/io/submap_painter.h"
namespace pbstream_convertor{
class PbstreamConvertor
{

public:
  PbstreamConvertor(const std::string& pbstream_path,const cartographer::mapping::proto::MapBuilderOptions& options);
  void mergeSubmaps(const std::vector<cartographer::mapping::SubmapId>& ids,const cartographer::mapping::SubmapId& new_submap_id);
private:
  std::map<int, int> LoadState(cartographer::io::ProtoStreamReaderInterface* const reader,
                           bool load_frozen_state);
  
  void preHandle();
  void clusteringSubmaps(); 
  int AddTrajectoryForDeserialization(
    const cartographer::mapping::proto::TrajectoryBuilderOptionsWithSensorIds&
    options_with_sensor_ids_proto);
  void getImage(std::unique_ptr<cartographer::mapping::Submap2D> merge_submap,const cartographer::transform::Rigid3f& global_pose);
  
  
  std::unique_ptr<cartographer::mapping::PoseGraph> origin_pose_graph_, new_pose_graph_;
  const cartographer::mapping::proto::MapBuilderOptions options_;
  cartographer::common::ThreadPool thread_pool_;
  std::vector<std::unique_ptr<cartographer::mapping::TrajectoryBuilderInterface>>
  trajectory_builders_;
  std::vector<cartographer::mapping::proto::TrajectoryBuilderOptionsWithSensorIds>
  all_trajectory_builder_options_;
  
  std::vector<int> num_range_data_;

  cartographer::mapping::MapById<cartographer::mapping::NodeId, cartographer::transform::Rigid3d> node_poses_;
  std::map<int,std::vector<cartographer::mapping::NodeId>> nodes_sort_by_trajectory_;
  std::map<int,std::vector<cartographer::mapping::SubmapId>> clustering_submaps_;
  std::map<cartographer::mapping::SubmapId,std::vector<cartographer::mapping::TrajectoryNode>> submap_with_nodes_; 
  std::map<cartographer::mapping::SubmapId,std::vector<cartographer::mapping::NodeId>> new_constraints_; 
  
  int interval_of_submap_;
  bool debug_flag_;
}; 
}
#endif // PBSTREAMCONVERTOR_H
