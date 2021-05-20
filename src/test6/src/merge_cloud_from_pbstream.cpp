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

#include "merge_cloud_from_pbstream.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include<pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>  
#include "pcl_conversions/pcl_conversions.h"
using namespace std;
using namespace cv;
using namespace cartographer;
using namespace cartographer::mapping;

MergeCloudFromPbstream::MergeCloudFromPbstream(const ros::NodeHandle& n):nh_(n),initialised_(false)
{
  cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/merge_point_cloud", 1);
  timer_pub_ = nh_.createWallTimer(ros::WallDuration(0.2),
                                            &MergeCloudFromPbstream::pubCloud,this);
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  
  pose_sub_.subscribe(nh_, "/cartographer_ros/scan_pose", 3);
  cloud_sub_.subscribe(nh_, "/cartographer_ros/merge_point_cloud", 3);
  cloud_with_pose_sub_ = nh_.subscribe("/cartographer_node/cloud_with_pose", 3, &MergeCloudFromPbstream::handleCloudWithPoseMsg,this);
  poseCloudSync policy_lidars(15);
  topicsSync_ = new message_filters::Synchronizer<poseCloudSync>(poseCloudSync(policy_lidars), pose_sub_, cloud_sub_);
  topicsSync_->registerCallback(boost::bind(&MergeCloudFromPbstream::handleTopics, this, _1, _2));
  downSizeFilter_.setLeafSize(0.1, 0.1, 0.1);
}

MergeCloudFromPbstream::~MergeCloudFromPbstream()
{

}

void MergeCloudFromPbstream::pubCloud(const ros::WallTimerEvent& unused_timer_event)
{
  if(!initialised_)
    return;
  mapping::NodeId id = frame_ids_.front();
  auto point_cloud_temp = points_with_id_.at(id);
  auto base2map = node_poses_.at(id);
  
  for(vector<sensor::RangefinderPoint>::iterator it = point_cloud_temp.begin(); it!=point_cloud_temp.end();  )
  {
    if(it->position.norm() > 15 /*|| it->position.z() > 0.3*/)
      it = point_cloud_temp.erase(it);
    else
      it++;
  }
  cartographer::sensor::PointCloud temp
          = cartographer::sensor::TransformPointCloud(point_cloud_temp, base2map.cast<float>());
          
  for(int i = 0; i < point_cloud_temp.size(); i++)
  {
    pcl::PointXYZI p;
    p.x = point_cloud_temp[i].position.x();
    p.y = point_cloud_temp[i].position.y();
    p.z = point_cloud_temp[i].position.z();
    p.intensity = point_cloud_temp[i].intensity;
    if(point_cloud_temp[i].intensity != 0)
      cout << p.intensity <<endl;
    cloud_->points.push_back(p);
    if(i > 500) 
      break;
  }
  sensor_msgs::PointCloud2 laserCloudTemp;
  if(cloud_pub_.getNumSubscribers())
  {
    pcl::toROSMsg(*cloud_, laserCloudTemp);
    laserCloudTemp.header.stamp = ros::Time::now();
    laserCloudTemp.header.frame_id = "map";
    cloud_pub_.publish(laserCloudTemp);
  }
  frame_ids_.pop();
}

bool MergeCloudFromPbstream::readFromPbstream(std::string filename)
{
//   vector<cartographer::mapping::TrajectoryNode::Data> node_datas;
//   std::unique_ptr<cartographer::mapping::Submap3D> merge_submap;
//   mapping::ValueConversionTables conversion_tables;
//   mapping::proto::ProbabilityGridRangeDataInserterOptions3D options;
//   options.set_hit_probability(0.55);
//   options.set_miss_probability(0.49);
//   options.set_insert_free_space(true);
//   mapping::RangeDataInserterInterface* range_data_inserter = 
//     new mapping::ProbabilityGridRangeDataInserter2D(options);
  io::ProtoStreamReader stream(filename);
  io::ProtoStreamDeserializer deserializer(&stream);
  cartographer::mapping::proto::PoseGraph pose_graph_proto =
  deserializer.pose_graph();
  
  for (const cartographer::mapping::proto::Trajectory& trajectory_proto :
    pose_graph_proto.trajectory())
  {
//     node_datas.resize(trajectory_proto.node().size());
    for (const cartographer::mapping::proto::Trajectory::Node& node_proto :
      trajectory_proto.node())
    {
      
      transform::Rigid3d global_pose = transform::ToRigid3(node_proto.pose());
      node_poses_.Insert(
        NodeId{ trajectory_proto.trajectory_id(), node_proto.node_index() },
                        global_pose);
    }
  }
  
  
  cartographer::mapping::proto::SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case cartographer::mapping::proto::SerializedData::kNode:
      {
        cartographer::mapping::proto::Node* proto_node = proto.mutable_node();
        cartographer::mapping::proto::TrajectoryNodeData proto_node_data = *proto_node->mutable_node_data();
        
        cartographer::mapping::TrajectoryNode::Data node_data = cartographer::mapping::FromProto(proto_node_data);
        NodeId id(proto_node->node_id().trajectory_id(),proto_node->node_id().node_index());
        transform::Rigid3d node_pose = node_poses_.at(id);
        points_with_id_.Insert(id,node_data.low_resolution_point_cloud);
        for(int i =0; i < node_data.low_resolution_point_cloud.size();i++)
        {
          if(node_data.low_resolution_point_cloud[i].intensity > 0)
          {
            cout << node_data.low_resolution_point_cloud[i].intensity << " " <<endl;
            break;
          }
        }
        cartographer::sensor::PointCloud temp
          = cartographer::sensor::TransformPointCloud(node_data.high_resolution_point_cloud, node_pose.cast<float>());
        high_resolution_cloud_.insert(high_resolution_cloud_.end(),temp.begin(),temp.end());
        frame_ids_.push(id);
//         sensor::RangeData range_data_origin_high{{0.,0.,0.},
//                                      node_data.high_resolution_point_cloud,
//                                      {}};
//         
//         sensor::RangeData range_data_high = 
//         sensor::TransformRangeData(range_data_origin_high, node_pose.cast<float>());        
//         nodes_data.emplace_back(NodeData{range_data_high,node_data.rotational_scan_matcher_histogram,
//                             node_data.gravity_alignment/*Eigen::Quaterniond::Identity()*/,node_pose});
      }
      default: break;
    }
  }
  initialised_ = true;
  cout << frame_ids_.size() <<endl;
  return true;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  ros::Rate rate(100);
  for(sensor::RangefinderPoint point: high_resolution_cloud_)
  {
    pcl::PointXYZI p;
    p.x = point.position.x();
    p.y = point.position.y();
    p.z = point.position.z();
    p.intensity = point.intensity;
//     if(p.x * p.x + p.y * p.y + p.z * p.z > 20)
//       continue;
    cloud->points.push_back(p);
    sensor_msgs::PointCloud2 laserCloudTemp;
    if(cloud_pub_.getNumSubscribers())
    {
      pcl::toROSMsg(*cloud, laserCloudTemp);
      laserCloudTemp.header.stamp = ros::Time::now();
      laserCloudTemp.header.frame_id = "base_footprint";
      cloud_pub_.publish(laserCloudTemp);
    }
    rate.sleep(); 
  }
  return true;
//   pcl::visualization::CloudViewer viewer("cloud viewer");
//   viewer.showCloud(cloud);
  if(0){
    pcl::visualization::PCLVisualizer p;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> tgt_h (cloud, abs(rand() % 255), abs(rand() % 255), abs(rand() % 255));
    p.addPointCloud(cloud, tgt_h, "target" + to_string(abs(rand())));
    p.spin();
  }
  cloud->width = 1;
  cloud->height = cloud->points.size();
  pcl::io::savePCDFileASCII("/home/cyy/temp1.pcd", *cloud);
  pcl::PCLPointCloud2 cloud1;
  if(pcl::io::loadPCDFile("/home/cyy/temp1.pcd", cloud1) < 0)
  {
    cout << "Error: cannot load the PCD file!!!" << endl;
    return -1;
  }
  pcl::PLYWriter writer; 
  writer.write("/home/cyy/temp2.ply", cloud1,Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false, true);
  std::cout << "11done!" <<std::endl;
  return true;
}

void MergeCloudFromPbstream::handleTopics(const geometry_msgs::PoseStampedConstPtr& pose_msg, 
                                          const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_msg, *cur_cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  Eigen::Affine3d transform_2;
  tf::poseMsgToEigen(pose_msg->pose,transform_2);
  pcl::transformPointCloud (*cur_cloud, *transformed_cloud, transform_2);
  *cloud_ += *transformed_cloud;
  sensor_msgs::PointCloud2 laserCloudTemp;
  downSizeFilter_.setInputCloud(cloud_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  downSizeFilter_.filter(*filtered_cloud);
  if(cloud_pub_.getNumSubscribers())
  {
    pcl::toROSMsg(*filtered_cloud, laserCloudTemp);
    laserCloudTemp.header.stamp = ros::Time::now();
    laserCloudTemp.header.frame_id = "map";
    cloud_pub_.publish(laserCloudTemp);
  }
}

void MergeCloudFromPbstream::handleCloudWithPoseMsg(const cartographer_ros_msgs::CloudWithPoseConstPtr& cloud_msg)
{
  for(int i = 0; i < cloud_msg->clouds.size(); i++){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(cloud_msg->clouds[i], *cur_cloud);
    for(int j = 0; j < cur_cloud->points.size(); j++  )
    {
      pcl::PointXYZI point = cur_cloud->points[j];
      
      if( (point.x * point.x + point.y * point.y +  point.z * point.z) > 20 || point.z > -0.8)
        continue;
      Eigen::Affine3d lidar2base;
      Eigen::Affine3d base2odom;
      tf::poseMsgToEigen(cloud_msg->lidar2base_poses[i], lidar2base);
      tf::poseMsgToEigen(cloud_msg->base2map_pose, base2odom);
      Eigen::Vector3d origin_point(point.x, point.y,point.z);
      auto new_point = base2odom * lidar2base * origin_point;
      point.x = new_point[0];
      point.y = new_point[1];
      point.z = new_point[2];
      cloud_->points.push_back(point);
    }
  }
  sensor_msgs::PointCloud2 laserCloudTemp;
  if(cloud_pub_.getNumSubscribers())
  {
    pcl::toROSMsg(*cloud_, laserCloudTemp);
    laserCloudTemp.header.stamp = cloud_msg->header.stamp;
    laserCloudTemp.header.frame_id = "odom";
    cloud_pub_.publish(laserCloudTemp);
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "merge_cloud_from_pbstream");
  ros::NodeHandle n;
  MergeCloudFromPbstream temp(n);
  temp.readFromPbstream("/home/cyy/map/sanhuan06/map.pbstream");
  
  ros::spin();
  return 1;
}



