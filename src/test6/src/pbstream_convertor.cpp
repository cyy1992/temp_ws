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

#include "pbstream_convertor.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "math.h"
#include <ros/package.h>
namespace pbstream_convertor{
using namespace std;
using namespace cv;
using namespace cartographer;
// using namespace cartographer::common;
using namespace cartographer::mapping;
using namespace cartographer::io;

using mapping::proto::SerializedData;

using mapping::MapById;
using mapping::NodeId;
using mapping::PoseGraphInterface;
using mapping::SubmapId;
using mapping::TrajectoryNode;
using mapping::proto::SerializedData;
static constexpr int kMappingStateSerializationFormatVersion = 2;
static constexpr int kFormatVersionWithoutSubmapHistograms = 1;

const int kPaddingPixel = 0;
using UniqueCairoSurfacePtr =
    std::unique_ptr<cairo_surface_t, void (*)(cairo_surface_t*)>;
using UniqueCairoPtr = std::unique_ptr<cairo_t, void (*)(cairo_t*)>;
constexpr cairo_format_t kCairoFormat = CAIRO_FORMAT_ARGB32;

inline UniqueCairoSurfacePtr MakeUniqueCairoSurfacePtr(cairo_surface_t* surface) {
  return UniqueCairoSurfacePtr(surface, cairo_surface_destroy);
}
inline UniqueCairoPtr MakeUniqueCairoPtr(cairo_t* surface) {
  return UniqueCairoPtr(surface, cairo_destroy);
}
inline Eigen::Affine3d ToEigen_mapping(const ::cartographer::transform::Rigid3d& rigid3) {
  return Eigen::Translation3d(rigid3.translation()) * rigid3.rotation();
}
void CairoPaintSubmapSlice(
    const double scale,
    const SubmapSlice& submap,
    cairo_t* cr, std::function<void(const SubmapSlice&)> draw_callback) {
  cairo_scale(cr, scale, scale);
  const auto& submap_slice = submap;
  if (submap_slice.surface == nullptr) {
    return;
  }
  const Eigen::Matrix4d homo =
      ToEigen_mapping(submap_slice.slice_pose).matrix();

  cairo_save(cr);
  cairo_matrix_t matrix;
  cairo_matrix_init(&matrix, homo(1, 0), homo(0, 0), -homo(1, 1), -homo(0, 1),
                    homo(0, 3), -homo(1, 3));
  cairo_transform(cr, &matrix);

  const double submap_resolution = submap_slice.resolution;
  cairo_scale(cr, submap_resolution, submap_resolution);

  // Invokes caller's callback to utilize slice data in global cooridnate
  // frame. e.g. finds bounding box, paints slices.
  draw_callback(submap_slice);
  cairo_restore(cr);
}

PaintSubmapSlicesResult PaintSubmapSlice(
     SubmapSlice& submap,
    const double resolution) {
  Eigen::AlignedBox2f bounding_box;
  {
    auto surface = MakeUniqueCairoSurfacePtr(
        cairo_image_surface_create(kCairoFormat, 1, 1));
    auto cr = MakeUniqueCairoPtr(cairo_create(surface.get()));
    const auto update_bounding_box = [&bounding_box, &cr](double x, double y) {
      cairo_user_to_device(cr.get(), &x, &y);
      bounding_box.extend(Eigen::Vector2f(x, y));
    };

    CairoPaintSubmapSlice(
        1. / resolution, submap, cr.get(),
        [&update_bounding_box](const SubmapSlice& submap_slice) {
          update_bounding_box(0, 0);
          update_bounding_box(submap_slice.width, 0);
          update_bounding_box(0, submap_slice.height);
          update_bounding_box(submap_slice.width, submap_slice.height);
        });
  }

  const Eigen::Array2i size(
      std::ceil(bounding_box.sizes().x()) + 2 * kPaddingPixel,
      std::ceil(bounding_box.sizes().y()) + 2 * kPaddingPixel);
  const Eigen::Array2f origin(-bounding_box.min().x() + kPaddingPixel,
                              -bounding_box.min().y() + kPaddingPixel);

  auto surface = MakeUniqueCairoSurfacePtr(
      cairo_image_surface_create(kCairoFormat, size.x(), size.y()));
  {
    auto cr = MakeUniqueCairoPtr(cairo_create(surface.get()));
    cairo_set_source_rgba(cr.get(), 0.5, 0.0, 0.0, 1.);
    cairo_paint(cr.get());
    cairo_translate(cr.get(), origin.x(), origin.y());
    CairoPaintSubmapSlice(1. / resolution, submap, cr.get(),
                           [&cr](const SubmapSlice& submap_slice) {
                             cairo_set_source_surface(
                                 cr.get(), submap_slice.surface.get(), 0., 0.);
//                              cairo_set_antialias (cr.get(), CAIRO_ANTIALIAS_NONE);
//                              cairo_pattern_set_filter(cairo_get_source(cr.get()),CAIRO_FILTER_NEAREST);
                             cairo_paint(cr.get());
                           });
    cairo_surface_flush(surface.get());
  }
  return PaintSubmapSlicesResult(std::move(surface), origin);
}

std::unique_ptr<mapping::GridInterface> CreateGrid(
  const Eigen::Vector2f& origin,mapping::ValueConversionTables* conversion_tables) {
constexpr int kInitialSubmapSize = 100;
float resolution = 0.05;
return absl::make_unique<mapping::ProbabilityGrid>(
    mapping::MapLimits(resolution,
              origin.cast<double>() + 0.5 * kInitialSubmapSize *
              resolution * Eigen::Vector2d::Ones(),
              mapping::CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
    conversion_tables);
}

PbstreamConvertor::PbstreamConvertor(const std::string& pbstream_path,
                                     const cartographer::mapping::proto::MapBuilderOptions& options):
  options_(options), thread_pool_(options.num_background_threads())
{
//   cartographer::mapping::proto::MapBuilderOptions options;
  interval_of_submap_ = 10;
  debug_flag_ = false;
  origin_pose_graph_ = absl::make_unique<cartographer::mapping::PoseGraph2D>(
        options_.pose_graph_options(),
          absl::make_unique<optimization::OptimizationProblem2D>(
            options_.pose_graph_options().optimization_problem_options()),
          &thread_pool_);
  
  new_pose_graph_ = absl::make_unique<cartographer::mapping::PoseGraph2D>(
        options_.pose_graph_options(),
          absl::make_unique<optimization::OptimizationProblem2D>(
            options_.pose_graph_options().optimization_problem_options()),
          &thread_pool_);

  cartographer::io::ProtoStreamReader stream(pbstream_path + "/origin_map.pbstream");
  LoadState(&stream, false);
  preHandle();
  clusteringSubmaps();
  
  LOG(INFO) << "start merging submaps ...";
  for(auto submap_ids:clustering_submaps_)
  {
    mergeSubmaps(submap_ids.second,SubmapId(0,submap_ids.first));
  }
  LOG(INFO) << "merge submaps done !";
  io::ProtoStreamWriter writer(pbstream_path + "/map.pbstream");
  while(all_trajectory_builder_options_.size() >1 )
    all_trajectory_builder_options_.pop_back();
  
  LOG(INFO) << "start writing pbstream ... ";
  io::WritePbStream(*new_pose_graph_, all_trajectory_builder_options_, &writer,
                    true);
  cout << "done!" <<endl;
  exit(1);
}
int PbstreamConvertor::AddTrajectoryForDeserialization(
    const proto::TrajectoryBuilderOptionsWithSensorIds&
    options_with_sensor_ids_proto)
{
  const int trajectory_id = trajectory_builders_.size();
  trajectory_builders_.emplace_back();
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
//   CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}

std::map<int, int> PbstreamConvertor::LoadState(io::ProtoStreamReaderInterface* const reader,
                           bool load_frozen_state)
{
  LOG(INFO) << "start loading ... " <<endl;

  io::ProtoStreamDeserializer deserializer(reader);

  // Create a copy of the pose_graph_proto, such that we can re-write the
  // trajectory ids.
  proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
  const auto& all_builder_options_proto =
      deserializer.all_trajectory_builder_options();
      
  std::map<int, int> trajectory_remapping;
//   cout << "trajectory size: " <<pose_graph_proto.trajectory_size() <<endl;
  for (int i = 0; i < pose_graph_proto.trajectory_size(); ++i) {
    auto& trajectory_proto = *pose_graph_proto.mutable_trajectory(i);
    const auto& options_with_sensor_ids_proto =
        all_builder_options_proto.options_with_sensor_ids(i);
    const int new_trajectory_id = AddTrajectoryForDeserialization(options_with_sensor_ids_proto);
//     cout << "new_trajectory_id:" <<new_trajectory_id <<endl;
    CHECK(trajectory_remapping.emplace(trajectory_proto.trajectory_id(),
                                       new_trajectory_id).second)
        << "Duplicate trajectory ID: " << trajectory_proto.trajectory_id();
    trajectory_proto.set_trajectory_id(new_trajectory_id);
    if (load_frozen_state) {
      origin_pose_graph_->FreezeTrajectory(new_trajectory_id);
    }
  }
  new_pose_graph_->FreezeTrajectory(0);
//   cout << "pose_graph_proto.mutable_constraint size:" <<pose_graph_proto.mutable_constraint()->size()<<endl;
  // Apply the calculated remapping to constraints in the pose graph proto.
  for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) {
//     cout << "constraint_proto.submap_id().trajectory_id(): "<<constraint_proto.submap_id().trajectory_id() <<endl;
    constraint_proto.mutable_submap_id()->set_trajectory_id(
          trajectory_remapping.at(constraint_proto.submap_id().trajectory_id()));
    
//     cout << "constraint_proto.node_id().trajectory_id(): "<<constraint_proto.node_id().trajectory_id() <<endl;
    constraint_proto.mutable_node_id()->set_trajectory_id(
          trajectory_remapping.at(constraint_proto.node_id().trajectory_id()));
  }
  
  cartographer::mapping::MapById<SubmapId, transform::Rigid3d> submap_poses;

  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory())
  {
    for (const proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap())
    {
      submap_poses.Insert(SubmapId{trajectory_proto.trajectory_id(),
                                   submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }

  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory())
  {
    for (const proto::Trajectory::Node& node_proto : trajectory_proto.node())
    {
//       cout << "trajectory_proto.trajectory_id():" <<trajectory_proto.trajectory_id()  <<"," <<node_proto.node_index()<<endl;
      node_poses_.Insert(
          NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()},
          transform::ToRigid3(node_proto.pose()));
    }
  }
  
  // Set global poses of landmarks.
  for (const auto& landmark : pose_graph_proto.landmark_poses()) {
    origin_pose_graph_->SetLandmarkPose(landmark.landmark_id(),
                                 transform::ToRigid3(landmark.global_pose()),
                                 true);
    new_pose_graph_->SetLandmarkPose(landmark.landmark_id(),
                                 transform::ToRigid3(landmark.global_pose()),
                                 true);
  }
  
  MapById<SubmapId, mapping::proto::Submap> submap_id_to_submap;
  MapById<NodeId, mapping::proto::Node> node_id_to_node; 
  SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case SerializedData::kPoseGraph:
        LOG(ERROR) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
        break;
      case SerializedData::kAllTrajectoryBuilderOptions:
        LOG(ERROR) << "Found multiple serialized "
                      "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                      "corrupt!.";
        break;
      case SerializedData::kSubmap: {
        proto.mutable_submap()->mutable_submap_id()->set_trajectory_id(
            trajectory_remapping.at(
                proto.submap().submap_id().trajectory_id()));
        submap_id_to_submap.Insert(
            SubmapId{proto.submap().submap_id().trajectory_id(),
                     proto.submap().submap_id().submap_index()},
            proto.submap());
        break;
      }
      case SerializedData::kNode: {
        proto.mutable_node()->mutable_node_id()->set_trajectory_id(
            trajectory_remapping.at(proto.node().node_id().trajectory_id()));
        const NodeId node_id(proto.node().node_id().trajectory_id(),
                             proto.node().node_id().node_index());
        const transform::Rigid3d& node_pose = node_poses_.at(node_id);
        origin_pose_graph_->AddNodeFromProto(node_pose, proto.node());
        node_id_to_node.Insert(node_id, proto.node());
        break;
      }
      case SerializedData::kTrajectoryData: {
        proto.mutable_trajectory_data()->set_trajectory_id(
            trajectory_remapping.at(proto.trajectory_data().trajectory_id()));
        origin_pose_graph_->SetTrajectoryDataFromProto(proto.trajectory_data());
        // no use
        break;
      }
      case SerializedData::kImuData: {
        if (load_frozen_state) break; 
        origin_pose_graph_->AddImuData(
              trajectory_remapping.at(proto.imu_data().trajectory_id()),
              sensor::FromProto(proto.imu_data().imu_data()));
        new_pose_graph_->AddImuData(
              trajectory_remapping.at(0),
              sensor::FromProto(proto.imu_data().imu_data()));
        break;
      }
      case SerializedData::kOdometryData: {
        if (load_frozen_state) break;
        origin_pose_graph_->AddOdometryData(
              trajectory_remapping.at(proto.odometry_data().trajectory_id()),
              sensor::FromProto(proto.odometry_data().odometry_data()));
        new_pose_graph_->AddOdometryData(
              trajectory_remapping.at(0),
              sensor::FromProto(proto.odometry_data().odometry_data()));
        break;
      }
      case SerializedData::kFixedFramePoseData: {
        if (load_frozen_state) break;
        origin_pose_graph_->AddFixedFramePoseData(
              trajectory_remapping.at(
                proto.fixed_frame_pose_data().trajectory_id()),
              sensor::FromProto(
                proto.fixed_frame_pose_data().fixed_frame_pose_data()));
        new_pose_graph_->AddFixedFramePoseData(
              trajectory_remapping.at(
                0),
              sensor::FromProto(
                proto.fixed_frame_pose_data().fixed_frame_pose_data()));
        break;
      }
      case SerializedData::kLandmarkData: {
        if (load_frozen_state) break;
        origin_pose_graph_->AddLandmarkData(
              trajectory_remapping.at(proto.landmark_data().trajectory_id()),
              sensor::FromProto(proto.landmark_data().landmark_data()));
        new_pose_graph_->AddLandmarkData(
              trajectory_remapping.at(0),
              sensor::FromProto(proto.landmark_data().landmark_data()));
        break;
      }
      default:
        LOG(WARNING) << "Skipping unknown message type in stream: "
                     << proto.GetTypeName();       
    }
  }
  
  // TODO(schwoere): Remove backwards compatibility once the pbstream format
  // version 2 is established.
//   if (deserializer.header().format_version() ==
//       io::kFormatVersionWithoutSubmapHistograms) {
//     submap_id_to_submap =
//         cartographer::io::MigrateSubmapFormatVersion1ToVersion2(
//             submap_id_to_submap, node_id_to_node, pose_graph_proto);
//   }
  for (const auto& submap_id_submap : submap_id_to_submap) {
    origin_pose_graph_->AddSubmapFromProto(submap_poses.at(submap_id_submap.id),
                                    submap_id_submap.data);
    cout << "submap id: " << submap_id_submap.id << "\t submap_pose:" << submap_poses.at(submap_id_submap.id).translation().transpose() <<endl;
  }

  if (load_frozen_state) {
    // Add information about which nodes belong to which submap.
    // Required for 3D pure localization.
    for (const proto::PoseGraph::Constraint& constraint_proto :
         pose_graph_proto.constraint()) {
      if (constraint_proto.tag() !=
          proto::PoseGraph::Constraint::INTRA_SUBMAP) {
        continue;
      }
      origin_pose_graph_->AddNodeToSubmap(
            NodeId{constraint_proto.node_id().trajectory_id(),
                   constraint_proto.node_id().node_index()},
            SubmapId{constraint_proto.submap_id().trajectory_id(),
                     constraint_proto.submap_id().submap_index()});
    }
  } else {
    // When loading unfrozen trajectories, 'AddSerializedConstraints' will
    // take care of adding information about which nodes belong to which
    // submap.
    origin_pose_graph_->AddSerializedConstraints(
          FromProto(pose_graph_proto.constraint()));
  }

  CHECK(reader->eof());
  LOG(INFO) << "Load done!" <<endl;
  return trajectory_remapping;
}

void PbstreamConvertor::clusteringSubmaps()
{
  LOG(INFO) << "start clustering submaps ... " <<endl;

  map<SubmapId, Point2f> submap_poses;
  for(const auto& submap_nodes : submap_with_nodes_)
  {
    Mat points_temp(submap_nodes.second.size(), 1, CV_32FC2),labels_temp;
    int clusterCount = 1;
    Mat centers(clusterCount, 1, points_temp.type());
    int i=0;
    for(const auto& node: submap_nodes.second)
    {
      points_temp.at<Vec2f>(i)[0] = node.global_pose.translation()(0);
      points_temp.at<Vec2f>(i)[1] = node.global_pose.translation()(1);
      i++;
    }
    kmeans(points_temp, clusterCount, labels_temp, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);

    submap_poses[submap_nodes.first] = Point2f(centers.at<Vec2f>(0)[0],centers.at<Vec2f>(0)[1]);
  }
  
  
  
  Mat points(submap_poses.size(), 1, CV_32FC2),labels;
//   MapById<SubmapId,transform::Rigid3d>::ConstIterator it = submap_poses_.begin();
  int i = 0;
  float min_x=1e9,min_y=1e9,max_x=-1e9,max_y=-1e9;
  vector<SubmapId> submap_ids;
//   submap_ids.resize(submap_poses_.size());
  for(const auto& it : submap_poses)
  {
    float x = it.second.x;
    float y = it.second.y;
    points.at<Vec2f>(i)[0] = x;
    points.at<Vec2f>(i)[1] = y;
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
    submap_ids.push_back(it.first); 
    i++;
  }
  float det_temp = std::max(max_x - min_x,max_y - min_y);
  cout << "det_temp:" << det_temp <<endl;
  int clusterCount = det_temp / 30 + 1;
  
  Mat centers(clusterCount, 1, points.type());
  kmeans(points, clusterCount, labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);
  Mat img(1000, 1000, CV_8UC3,Scalar(255,255,255));
  Scalar colorTab[clusterCount];
  RNG rng(12435);
  for(int n =0; n < clusterCount; n++)
  {
    colorTab[n] = Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
  };
  int last_id =0;
  vector<int> temp_num;

  for(int j = 0; j < submap_poses.size(); j++)
  {
    int clusterIdx = labels.at<int>(j);
    int k =0;
    for(auto it : temp_num)
    {
      if(it == clusterIdx)
        break;
      k++;
    }
    int sort_id;
    
    if(k >= temp_num.size())
    {
      sort_id = temp_num.size();
      temp_num.push_back(clusterIdx);
    }
    else
      sort_id = k;
    cout <<clusterIdx <<" " << k << " " << sort_id <<endl;
    clustering_submaps_[sort_id].push_back(submap_ids[j]);
    if(debug_flag_)
    {
      Point ipt;
      ipt.x = 500 + 5* points.at<Point2f>(j).x;
      ipt.y = 500 + 5 * points.at<Point2f>(j).y;
      cv::circle( img, ipt, 2, colorTab[sort_id], CV_FILLED, CV_AA );
    }
  }
  for(auto it1: clustering_submaps_)
  {
    cout << "sort_id: " <<it1.first << endl;
    for(auto it2: it1.second)
      cout << it2 << ", ";
    cout <<endl;
  }
  
  if(debug_flag_)
  {
    imshow("temp1", img);
    waitKey(0);
  }
  LOG(INFO) << "clustering submaps done! " <<endl;
}


void PbstreamConvertor::preHandle()
{
  LOG(INFO) << "start prehandling nodes and submaps ... " ;
  // if there has merge before, read information from xml
  {
    
  }
  // if there has no merge before, deal it with below method
  num_range_data_.resize(trajectory_builders_.size());
  const MapById<SubmapId,PoseGraphInterface::SubmapData>& submaps_data =
    origin_pose_graph_->GetAllSubmapData();
  const MapById<NodeId,TrajectoryNode> nodes =
    origin_pose_graph_->GetTrajectoryNodes();
  for(const auto& it: nodes)
  {
    nodes_sort_by_trajectory_[it.id.trajectory_id].push_back(it.id);
  }
  
  
  for(int i =0; i< num_range_data_.size(); i++)
  {
    proto::TrajectoryBuilderOptionsWithSensorIds options =
      all_trajectory_builder_options_[i];
    num_range_data_[i] = 
      options.trajectory_builder_options().trajectory_builder_2d_options().submaps_options().num_range_data();
    int j =0, node_cnt = 0;
    for(const auto& node_id:nodes_sort_by_trajectory_[i] )
    {
      if(node_cnt < num_range_data_[i] / 2)
        submap_with_nodes_[SubmapId(i,j)].push_back(nodes.at(node_id));
      else
      {
        submap_with_nodes_[SubmapId(i,j)].push_back(nodes.at(node_id));
        submap_with_nodes_[SubmapId(i,j+1)].push_back(nodes.at(node_id));
      }
      if(node_cnt%num_range_data_[i] == 0)
        ++j;
      ++node_cnt;
    }
  }
  const vector<PoseGraphInterface::Constraint>& constraints =
  origin_pose_graph_->constraints();
  
  int trajectory_nodes_size[nodes_sort_by_trajectory_.size()];
  {
    
    for(int i=0; i< nodes_sort_by_trajectory_.size(); i++)
    {
      trajectory_nodes_size[i] = nodes_sort_by_trajectory_[i].size();
    }
    
    for (const auto& node_id_data : origin_pose_graph_->GetTrajectoryNodes()) {
      SerializedData proto;
      auto* const node_proto = proto.mutable_node();
      const NodeId node_id(node_id_data.id.trajectory_id,node_id_data.id.node_index);
      const transform::Rigid3d& node_pose = node_poses_.at(node_id);
      int origin_node_index = node_id_data.id.node_index;
      int update_node_index = origin_node_index;
      for(int i =0; i < node_id_data.id.trajectory_id; i++)
        update_node_index += trajectory_nodes_size[i];
      
      node_proto->mutable_node_id()->set_trajectory_id(0);
      node_proto->mutable_node_id()->set_node_index(update_node_index);
      *node_proto->mutable_node_data() = ToProto(*node_id_data.data.constant_data);
      new_pose_graph_->AddNodeFromProto(node_pose, proto.node());
    }
    
/*    
    for(auto node: nodes_sort_by_trajectory_){
      proto.mutable_node()->mutable_node_id()->set_trajectory_id(0);
      const NodeId node_id(0,proto.node().node_id().node_index());
      
      new_pose_graph_->AddNodeFromProto(node_pose, proto.node());
    }*/
  }
  for(const auto& constrait :constraints)
  {
    int origin_node_index = constrait.node_id.node_index;
    int update_node_index = origin_node_index;
    for(int i =0; i < constrait.node_id.trajectory_id; i++)
      update_node_index += trajectory_nodes_size[i];
    new_constraints_[constrait.submap_id].push_back(NodeId(0,update_node_index));
  }
  
  LOG(INFO) << "prehandle done !" ;
}

void PbstreamConvertor::mergeSubmaps(const vector< SubmapId >& ids,const SubmapId& new_submap_id)
{
  mapping::ValueConversionTables conversion_tables;
  mapping::proto::ProbabilityGridRangeDataInserterOptions2D options;
  options.set_hit_probability(0.55);
  options.set_miss_probability(0.49);
  options.set_insert_free_space(true);
  mapping::RangeDataInserterInterface* range_data_inserter = 
    new mapping::ProbabilityGridRangeDataInserter2D(options);
  
  std::unique_ptr<mapping::Submap2D> merge_submap;
  bool initialised = false;
  //  avoid repetition
  set<common::Time> avoid_repetition_nodes;
  transform::Rigid3f global_pose_first;
  set<NodeId> constraint_nodes;
  for (const auto& submap_id : ids)
  {
    for(const auto& node:submap_with_nodes_[submap_id])
    {
//       if(avoid_repetition_nodes.emplace(node.time()).second)
//         continue;
      sensor::PointCloud points = node.constant_data->filtered_gravity_aligned_point_cloud;
      const transform::Rigid3d gravity_alignment =
        transform::Rigid3d::Rotation(node.constant_data->gravity_alignment);
      transform::Rigid3f global_pose =
        node.global_pose.cast<float>() * gravity_alignment.inverse().cast<float>();
//       transform::Rigid3f global_pose = node.global_pose.cast<float>();
      transform::Rigid2f global_pose_2d = transform::Project2D(global_pose);
      sensor::PointCloud points_in_global = sensor::TransformPointCloud(points,global_pose);
      Eigen::Vector2f origin;
      origin = global_pose_2d.translation();
      sensor::RangeData rangedata{{origin(0),origin(1),0},points_in_global,{}};
      if(!initialised){
        cout << "PointCloud size:" << points.size() <<endl;
        global_pose_first = global_pose;
        merge_submap.reset(new mapping::Submap2D(
          node.time(), Eigen::Vector2f(0,0),//origin
          std::unique_ptr<mapping::Grid2D>(
            static_cast<mapping::Grid2D*>(CreateGrid(Eigen::Vector2f(0,0), //origin
                                          &conversion_tables).release())),
          &conversion_tables));  
        initialised = true;
      }
      merge_submap->InsertRangeData(rangedata,range_data_inserter);
    }
    for(auto node_id : new_constraints_[submap_id])
      constraint_nodes.insert(node_id);
  }
  proto::Submap submap_proto =  merge_submap->ToProto(true);
  proto::SubmapId* proto_submap_id = submap_proto.mutable_submap_id();
  proto_submap_id->set_trajectory_id(new_submap_id.trajectory_id);
  proto_submap_id->set_submap_index(new_submap_id.submap_index);
  new_pose_graph_->AddSubmapFromProto(transform::Rigid3d::Identity(),submap_proto);
  //transform::Rigid3d(global_pose_first.cast<double>().translation(), Eigen::Quaterniond::Identity())
  for(auto& node_id:constraint_nodes)
  {
    new_pose_graph_->AddNodeToSubmap(
            NodeId{node_id.trajectory_id,
                   node_id.node_index},
            SubmapId{new_submap_id.trajectory_id,
                     new_submap_id.submap_index});
  }
  cout << "num_x_cells: " <<merge_submap->grid()->limits().cell_limits().num_x_cells <<endl;
  cout << "num_y_cells: " <<merge_submap->grid()->limits().cell_limits().num_y_cells <<endl;
  if(debug_flag_)
    getImage(std::move(merge_submap),transform::Rigid3f::Identity());
    //transform::Rigid3f(global_pose_first.translation(), Eigen::Quaternionf::Identity())
}

void PbstreamConvertor::getImage(std::unique_ptr<mapping::Submap2D> merge_submap,const transform::Rigid3f& global_pose)
{ 
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  SubmapSlice& submap_slice = submap_slices[SubmapId(0,0)];
//    string str = submap_query_client_.getService();
  cartographer_ros_msgs::SubmapQuery::Response response_query;
  proto::SubmapQuery::Response response_proto;
  merge_submap->ToResponseProto(global_pose.cast<double>(), &response_proto);
  for (const auto& texture_proto : response_proto.textures())
  {
    response_query.textures.emplace_back();
    auto& texture = response_query.textures.back();
    texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                         texture_proto.cells().end());
    texture.width = texture_proto.width();
    texture.height = texture_proto.height();
    texture.resolution = texture_proto.resolution();
    cout << "resolution: "<<texture_proto.resolution()<<endl;
  }
  auto response = absl::make_unique<::cartographer::io::SubmapTextures>();
  for (const auto& texture : response_query.textures) {
    const std::string compressed_cells(texture.cells.begin(),
                                       texture.cells.end());
    response->textures.emplace_back(::cartographer::io::SubmapTexture{
        ::cartographer::io::UnpackTextureData(compressed_cells, texture.width,
                                              texture.height),
        texture.width, texture.height, texture.resolution,
        global_pose.cast<double>()});
  }
  submap_slice.version = response->version;

  // We use the first texture only. By convention this is the highest
  // resolution texture and that is the one we want to use to construct the
  // map for ROS.
  const auto fetched_texture = response->textures.begin();
  submap_slice.width = fetched_texture->width;
  submap_slice.height = fetched_texture->height;
  submap_slice.slice_pose = fetched_texture->slice_pose;
  submap_slice.resolution = fetched_texture->resolution;
  submap_slice.cairo_data.clear();
  
  submap_slice.surface = ::cartographer::io::DrawTexture(
      fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
      fetched_texture->width, fetched_texture->height,
      &submap_slice.cairo_data);
  
  PaintSubmapSlicesResult result = PaintSubmapSlice(submap_slice,0.05);
  const int width = cairo_image_surface_get_width(result.surface.get());
  const int height = cairo_image_surface_get_height(result.surface.get());
  const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
      cairo_image_surface_get_data(result.surface.get()));
  cv::Mat pub_img(height,width,CV_8UC4);
  cv::Mat show_img(height,width,CV_8UC4);
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      const uint32_t packed = pixel_data[y * width + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int delta =
        128 - int(color);
      const unsigned char alpha = delta > 0 ? delta : -delta;
      const int value =
          observed == 0
              ? -1
              : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
      CHECK_LE(-1, value);
      CHECK_GE(100, value);
      show_img.at<Vec4b>(y, x)[0] = color;
      show_img.at<Vec4b>(y, x)[1] = color;
      show_img.at<Vec4b>(y, x)[2] = color;
      show_img.at<Vec4b>(y, x)[3] = 255;
//       if (color == 128)
//       {
//         pub_img.at<Vec4b>(y, x)[0] = 128;
//         pub_img.at<Vec4b>(y, x)[1] = 128;
//         pub_img.at<Vec4b>(y, x)[2] = 128;
//         pub_img.at<Vec4b>(y, x)[3] = 0;
//       }
//       
//       if(color == 128)
//           save_img.at<uint16_t>(y, x) = 0;
//         else
//           save_img.at<uint16_t>(y, x) = color * 128;
    }
  }
  
  imshow("show_img", show_img);
  waitKey(0);
}


}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"aa");
  ros::NodeHandle n;
  cartographer_ros::NodeOptions node_options;
  cartographer_ros::TrajectoryOptions trajectory_options;
  std::string cartographer_path = ros::package::getPath("cartographer_ros");
  LOG(INFO) << cartographer_path;
  const std::string path = "/home/cyy/map/1010_g2";
  std::size_t t_position = path.rfind('/');
  std::string id_string = path.substr(t_position + 1);
  LOG(INFO) << id_string;
  return 0;
  std::tie(node_options, trajectory_options) =
     cartographer_ros::LoadOptions("/home/cyy/jz_project/cartographer_ws/install_isolated/share/cartographer_ros/configuration_files", "backpack_2d_jz_mapping.lua");
  cartographer::mapping::proto::MapBuilderOptions options;
  pbstream_convertor::PbstreamConvertor pbstream("/home/cyy/map/1010_g2",node_options.map_builder_options);
  return 1;
}