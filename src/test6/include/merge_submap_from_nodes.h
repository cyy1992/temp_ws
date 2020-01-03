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

#ifndef MERGESUBMAPFROMNODES_H
#define MERGESUBMAPFROMNODES_H
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
class MergeSubmapFromNodes
{
public:
  MergeSubmapFromNodes();
  ~MergeSubmapFromNodes();
  
  
  bool readFromPbstream(std::string filename);
};

#endif // MERGESUBMAPFROMNODES_H
