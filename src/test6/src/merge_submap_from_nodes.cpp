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

#include "merge_submap_from_nodes.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include <fstream>
using namespace std;
using namespace cv;
using namespace cartographer;
using namespace cartographer::mapping;
std::unique_ptr<mapping::GridInterface> CreateGrid(
  const Eigen::Vector2f& origin,mapping::ValueConversionTables* conversion_tables) 
{
  constexpr int kInitialSubmapSize = 20;
  float resolution = 0.05;
  return absl::make_unique<mapping::ProbabilityGrid>(
    mapping::MapLimits(resolution,
                        origin.cast<double>() + 0.5 * kInitialSubmapSize *
                        resolution * Eigen::Vector2d::Ones(),
                        mapping::CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
                                                      conversion_tables);
}
cv::Mat image16ToImage8(const cv::Mat& image16)
{
  cv::Mat image8(image16.rows, image16.cols, CV_8UC1);
  for (int i = 0; i < image16.rows; i++)
    for (int j = 0; j < image16.cols; j++)
    {
      int value = image16.at<uint16_t>(i, j);
      if (value == 0)
        image8.at<uchar>(i, j) = 128;
      else
        image8.at<uchar>(i, j) = value / 128;
    } 
  return image8;
}
MergeSubmapFromNodes::MergeSubmapFromNodes()
{
  sub_ = nh_.subscribe("/front/color/image_raw",1000, &MergeSubmapFromNodes::HandleImage,this);
  outFile_.open("/home/cyy/poses.txt", std::ios::out | std::ios::app);
}

MergeSubmapFromNodes::~MergeSubmapFromNodes()
{
  
}

void MergeSubmapFromNodes::HandleImage(const sensor_msgs::Image::ConstPtr& msg)
{
//   double cur_t = msg->header.stamp.toSec();
  common::Time cur_t = cartographer_ros::FromRos(msg->header.stamp);
  map<double, transform::Rigid3d>::iterator it1 = poses_with_times_.begin();
  map<double, transform::Rigid3d>::iterator it2 = poses_with_times_.begin();
  ++it2;
  transform::Rigid3d transform;
  for(; it2 != poses_with_times_.end(); ++it1,++it2)
  {
    double t = it1->first;
    ros::Time t1 = ros::Time().fromSec(t);
    t = it2->first;
    ros::Time t2 = ros::Time().fromSec(t);
    common::Time t11 = cartographer_ros::FromRos(t1);
    common::Time t22 = cartographer_ros::FromRos(t2);
    if(cur_t > t11 && cur_t < t22){
      transform = transform::Interpolate(transform::TimestampedTransform{t11,
                                             it1->second},
             transform::TimestampedTransform{t22,
                                             it2->second},
             cur_t).transform;
      
      
      outFile_ << msg->header.stamp << "\t" << transform.translation().x() <<" " << transform.translation().y() <<" "
      << transform.translation().z() <<" "<< transform.rotation().x() <<" "<< transform.rotation().y() <<" "<< transform.rotation().z()
      <<" "<< transform.rotation().w() <<endl;
      break;
    }
  }
  cout << transform <<endl;
  
}

bool MergeSubmapFromNodes::readFromPbstream(std::string filename)
{
  vector<cartographer::mapping::TrajectoryNode::Data> node_datas;
  std::unique_ptr<cartographer::mapping::Submap2D> merge_submap;
  mapping::ValueConversionTables conversion_tables;
  mapping::proto::ProbabilityGridRangeDataInserterOptions2D options;
  options.set_hit_probability(0.55);
  options.set_miss_probability(0.49);
  options.set_insert_free_space(true);
  mapping::RangeDataInserterInterface* range_data_inserter = 
    new mapping::ProbabilityGridRangeDataInserter2D(options);
  io::ProtoStreamReader stream(filename);
  io::ProtoStreamDeserializer deserializer(&stream);
  cartographer::mapping::proto::PoseGraph pose_graph_proto =
  deserializer.pose_graph();
  MapById<NodeId, transform::Rigid3d> node_poses;
  for (const cartographer::mapping::proto::Trajectory& trajectory_proto :
    pose_graph_proto.trajectory())
  {
    node_datas.resize(trajectory_proto.node().size());
    for (const cartographer::mapping::proto::Trajectory::Node& node_proto :
      trajectory_proto.node())
    {
      transform::Rigid3d global_pose = transform::ToRigid3(node_proto.pose());
      node_poses.Insert(
        NodeId{ trajectory_proto.trajectory_id(), node_proto.node_index() },
                        global_pose);
      common::Time time= common::FromUniversal(node_proto.timestamp());
      ros::Time ros_time = cartographer_ros::ToRos(time);
      double t = ros_time.toSec();
      poses_with_times_[t] = global_pose;
    }
  }
  return true;
//   cartographer::mapping::proto::SerializedData proto_tmp;
//   bool initialised = false;
//   cout << "node_poses size:" << node_poses.size() <<endl;
//   while (deserializer.ReadNextSerializedData(&proto_tmp))
//   {
//     switch (proto_tmp.data_case())
//     {
//       case cartographer::mapping::proto::SerializedData::kNode:
//       {
//         cartographer::mapping::proto::Node* proto_node = proto_tmp.mutable_node();
//         const cartographer::mapping::proto::TrajectoryNodeData proto_node_data =
//         *proto_node->mutable_node_data();
//         const cartographer::mapping::TrajectoryNode::Data node_data =
//         cartographer::mapping::FromProto(proto_node_data);
//         
//         const sensor::PointCloud points = node_data.filtered_gravity_aligned_point_cloud;
//         
//         NodeId id(proto_node->node_id().trajectory_id(),
//                   proto_node->node_id().node_index());
//         const transform::Rigid3d gravity_alignment =
//         transform::Rigid3d::Rotation(node_data.gravity_alignment);
//         transform::Rigid3f node_global_pose =
//         node_poses.at(id).cast<float>()
//         * gravity_alignment.inverse().cast<float>();
//         sensor::PointCloud points_in_global =
//         sensor::TransformPointCloud(points, node_global_pose);
// //         all_point_cloud.insert(all_point_cloud.end(), point_cloud_in_map.begin(),
// //                                point_cloud_in_map.end());
//         transform::Rigid2f global_pose_2d = transform::Project2D(node_global_pose);
//         Eigen::Vector2f origin;
//         origin = global_pose_2d.translation();
//         sensor::RangeData rangedata{{origin(0),origin(1),0},points_in_global,{}};
//         if(!initialised){
// //           global_pose_first = global_pose;
//           merge_submap.reset(new mapping::Submap2D(
//             node_data.time, Eigen::Vector2f(0,0),//origin
//             std::unique_ptr<mapping::Grid2D>(
//               static_cast<mapping::Grid2D*>(CreateGrid(Eigen::Vector2f(0,0), //origin
//                                             &conversion_tables).release())),
//             &conversion_tables));  
//           initialised = true;
//         }
//         merge_submap->InsertRangeData(rangedata,range_data_inserter);
//       }
//     }
//   }
//   merge_submap->Finish();
//   const Grid2D* pg = merge_submap->grid();
//   double resolution = pg->limits().resolution();
//   int max[2];
//   max[0] = pg->limits().max()[0];
//   max[1] = pg->limits().max()[1];
//   int num_x_cells = pg->limits().cell_limits().num_x_cells;
//   int num_y_cells = pg->limits().cell_limits().num_y_cells;
//   const Eigen::AlignedBox2i& known_cells_box = pg->known_cells_box();
//   
//   const std::vector<uint16_t>&  cells = pg->cells();
//   cout << "max: " << max[0] <<"," << max[1] <<endl;
//   cout << "cells: " << num_x_cells <<"," << num_y_cells <<endl;
//   cout << "known_cells_box: " << known_cells_box.min().x() <<"," << known_cells_box.min().y() <<"," <<known_cells_box.max().x() <<"," << known_cells_box.max().y()<<endl;
//   cv::Mat cells_mat = cv::Mat::zeros(num_y_cells, num_x_cells, CV_16UC1);
//   for (uint i = 0; i < num_y_cells; i++)
//     for (uint j = 0; j < num_x_cells; j++)
//     {
//       uint16_t value = cells[i * num_x_cells + j];
//       cells_mat.at<uint16_t>(i, j) = value;
//     }
//     
// //   cv::imshow("16", cells_mat);
//   cv::Mat img = image16ToImage8(cells_mat);
//   imwrite("/home/cyy/temp.png",img);
//   cv::Mat show_img;
//   cv::resize(img, show_img, Size(img.cols*0.1, img.rows*0.1));
//   cv::imshow("8", show_img);
//   {
//     boost::property_tree::ptree p_map,p_top;
//     std::string data;
//     const int width = show_img.cols;
//     const int height = show_img.rows;
//     data = to_string(height);
//     p_map.put("width", data);
//     data.clear();
//     data = to_string(width);
//     p_map.put("height", data);
//     data.clear();
//     data = to_string(resolution);
//     p_map.put("resolution", data);data.clear();
//     double x = max[0] / resolution;
//     double y = max[1] / resolution;
//     cartographer::transform::Rigid3d::Vector translation(y,x,0);
//     cartographer::transform::Rigid3d::Quaternion quaternion(0, -sqrt(2)/2,sqrt(2)/2,0);
//     for (int i = 0; i < 3; i++)
//       data = data + std::to_string(translation[i]) + " ";
//     for (int i = 0; i < 3; i++)
//       data = data + std::to_string(quaternion.coeffs()[i]) + " ";
//     data = data + std::to_string(quaternion.coeffs()[3]);
//     p_map.put("pose", data);data.clear();
//     p_top.add_child("mapPng", p_map);
//     
//     boost::property_tree::xml_writer_settings<std::string> setting(' ', 2);
//     boost::property_tree::write_xml(full_map_path + "map_data.xml", p_top,
//                                     std::locale(), setting);
//   }
//   
//   cv::waitKey(0);
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"read_nodes");
  
  MergeSubmapFromNodes merge;
  merge.readFromPbstream("/home/cyy/map/.line2/map.pbstream");
  
  ros::spin();
  return 1;
}
  