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

#include "../include/pub_gps_path.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <cartographer/io/proto_stream.h>
#include "cartographer/mapping/proto/serialization.pb.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cartographer_ros/time_conversion.h>
using namespace std;
using namespace std::chrono;
using namespace cartographer;
using mapping::proto::SerializedData;

PubGpsPath::PubGpsPath(const ros::NodeHandle& n, const std::string& map_path):nh_(n),
  tf_buffer_{::ros::Duration(10.)}, tfListener_(tf_buffer_), map_path_(map_path)
{
  path_pub_ = nh_.advertise<nav_msgs::Path>("/pub_gps_path/gps_path",1);
  setParam();
  service_servers_.push_back(nh_.advertiseService(
      "save_poses_with_time", &PubGpsPath::SaveDataSrv, this));
  gps_sub_ = nh_.subscribe<gps_common::GPSFix>("/jzhw/gps/fix", 3,
                                          &PubGpsPath::handleGps, this);
  
  odom_frame_ = "odom";
  base_frame_ = "base_footprint";
}

PubGpsPath::~PubGpsPath()
{

}
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }
void PubGpsPath::setParam()
{
  //get gps2base
  double gps2base_x = 0, gps2base_y = 0, gps2base_yaw = 0;
  ::ros::param::get("/jzhw/calib/gps/default/px",gps2base_x);
  ::ros::param::get("/jzhw/calib/gps/default/py",gps2base_y);
  ::ros::param::get("/jzhw/calib/gps/default/yaw",gps2base_yaw);
  
  gps2base_.t = Eigen::Vector3d(gps2base_x,gps2base_y,0);
  gps2base_.q = Eigen::Quaterniond(cos(gps2base_yaw/2),0,0,sin(gps2base_yaw/2));
  std::cout <<"\033[36m gps2base_:" << gps2base_ <<"\033[0m" <<std::endl;
  
  //get fix_in_map_, ecef_in_map_
  boost::property_tree::ptree pt;
  cout << map_path_ << "/gps_data.xml! ";
  boost::property_tree::xml_parser::read_xml(map_path_+"/gps_data.xml", pt);
  if(pt.empty())
  {
    cout << "Can not get gps_data.xml! ";
    return;
  }
  double pose[7];
  string data = pt.get<std::string>("ecef_to_local_frame_pose");
  
  sscanf(data.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &pose[0], &pose[1], &pose[2],
      &pose[3], &pose[4], &pose[5], &pose[6]);
  cout << "ecef_to_local_frame_pose: " <<endl;
  for(int i =0; i < 7 ;i ++)
    cout <<std::setprecision(15)<< pose[i] <<" " ;
  cout <<endl;
  ecef_to_local_frame_.q = Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]);
  ecef_to_local_frame_.t = Eigen::Vector3d(pose[0], pose[1], pose[2]);
  data.clear();
  data = pt.get<std::string>("fix_frame_origin_in_map_pose");
  cout << "fix_frame_origin_in_map_pose: " <<data;
  sscanf(data.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &pose[0], &pose[1], &pose[2],
      &pose[3], &pose[4], &pose[5], &pose[6]);
  fix_in_map_.q = Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]);
  fix_in_map_.t = Eigen::Vector3d(pose[0], pose[1], pose[2]);
  ecef_in_map_.q = fix_in_map_.q * ecef_to_local_frame_.q;
  ecef_in_map_.t = fix_in_map_.q * ecef_to_local_frame_.t + fix_in_map_.t;
  
  auto item = pt.get_child_optional("gps_to_base_init_pose");
  if(!item)
  {
    gps_to_base_init_.t = Eigen::Vector3d(0,0,0);
    gps_to_base_init_.q = Eigen::Quaterniond::Identity();
  }
  else{
    data = pt.get<std::string>("gps_to_base_init_pose");
    sscanf(data.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &pose[0], &pose[1], &pose[2],
        &pose[3], &pose[4], &pose[5], &pose[6]);
    gps_to_base_init_.q = Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]);
    gps_to_base_init_.t = Eigen::Vector3d(pose[0], pose[1], pose[2]);
  }
  initialized_ = false;
  last_pub_time_ = steady_clock::now();
  cout << "fix_frame_origin_in_map: " <<fix_in_map_<<endl;
  cout << "ecef_to_local_frame: " <<ecef_to_local_frame_<<endl;
  cout << "ecef_in_map_: " <<ecef_in_map_<<endl;
  cout << "gps_to_base_init_: " <<gps_to_base_init_<<endl;
  fix_frame_in_map_yaw_ = (fix_in_map_ * gps_to_base_init_).getYaw();
//   dbg(fix_frame_in_map_yaw_);
  while(!gps_datas_.empty())
    gps_datas_.pop();
  if(pose[0] !=0 || pose[1] != 0 || pose[2] != 0)
    has_gps_data_info_ = true;

}

Eigen::Vector3d PubGpsPath::LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude) {
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(DegToRad(latitude));
  const double cos_phi = std::cos(DegToRad(latitude));
  const double sin_lambda = std::sin(DegToRad(longitude));
  const double cos_lambda = std::cos(DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + altitude) * cos_phi * cos_lambda;
  const double y = (N + altitude) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + altitude) * sin_phi;

  return Eigen::Vector3d(x, y, z);
}
Eigen::Quaterniond PubGpsPath::eul2quat(const Eigen::Vector3d& eul)
{
  Eigen::Matrix3d mat;
  mat= 
    Eigen::AngleAxisd(eul[0], ::Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(eul[1], ::Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(eul[2], ::Eigen::Vector3d::UnitX());
  return Eigen::Quaterniond(mat);
}
void PubGpsPath::handleGps(const gps_common::GPSFix::ConstPtr& msg)
{
  if(!has_gps_data_info_)
    return;
  if(msg->status.status == 2)
    valid_msg_ = *msg;
  else 
    return;
  geometry_msgs::TransformStamped base2map;
  Rigid3d northGps2northBase;
  double dip = valid_msg_.dip * M_PI / 180 ;
  Eigen::Quaterniond north2gps(cos(dip /2),0,0,sin(dip/2));
  northGps2northBase.q = gps2base_.q * north2gps; 
  northGps2northBase.t = gps2base_.t;
  
  Rigid3d fix_pose1;
  fix_pose1.t = ecef_to_local_frame_.q * LatLongAltToEcef(valid_msg_.latitude, valid_msg_.longitude,valid_msg_.altitude) + ecef_to_local_frame_.t;
  fix_pose1.q = Eigen::Quaterniond::Identity();
  geometry_msgs::PoseStamped temp_pose;

  Rigid3d base2fix =gps_to_base_init_* fix_pose1 * northGps2northBase.inverse();
//   cout << "base2map_gps:" << base2map_gps <<endl;
  Rigid3d base_to_map_gps = fix_in_map_ * base2fix;

  double yaw_corrected = fix_frame_in_map_yaw_ - dip;
//   
  Eigen::Quaterniond q_corrected = eul2quat(Eigen::Vector3d(yaw_corrected,0,0));
  
  Rigid3d base2map_gps;
  base2map_gps.t = base_to_map_gps.t;
  {
    geometry_msgs::PoseStamped temp_pose;
    temp_pose.header.stamp = valid_msg_.header.stamp;
    temp_pose.pose.position.x = base2map_gps.t(0);
    temp_pose.pose.position.y = base2map_gps.t(1);
    temp_pose.pose.position.z = base2map_gps.t(2);
    gps_path_.poses.push_back(temp_pose);
    gps_path_.header.frame_id = odom_frame_;
//     pathPub(valid_msg_);
    if(path_pub_.getNumSubscribers()){
      path_pub_.publish(gps_path_);
    }
  }
  times_.push(valid_msg_.header.stamp);
  poses_with_time_[times_.back()].push_back(base_to_map_gps);
  
  while(times_.size() >0 ){
    geometry_msgs::TransformStamped base2odom;
    try
    { 
      base2odom = tf_buffer_.lookupTransform(odom_frame_, base_frame_, times_.front());
      Eigen::Vector3d t(base2odom.transform.translation.x,
                        base2odom.transform.translation.y,
                        base2odom.transform.translation.z);
      Eigen::Quaterniond q(base2odom.transform.rotation.w, base2odom.transform.rotation.x,
                        base2odom.transform.rotation.y, base2odom.transform.rotation.z);
      poses_with_time_[times_.front()].push_back({q,t});
      cout << "time: " << times_.front() <<endl;
      times_.pop();
    }
    catch (tf2::TransformException& ex)
    {
      if((times_.back() - times_.front()).toSec() > 8.0)
      {
        
        poses_with_time_.erase(poses_with_time_.find(times_.front()));
        times_.pop();
      }
      break;
    }
  }
  
}
void PubGpsPath::pathPub(const gps_common::GPSFix& msg)
{
  Rigid3d northGps2northBase;
  double dip = msg.dip * M_PI / 180 ;
  Eigen::Quaterniond north2gps(cos(dip /2),0,0,sin(dip/2));
  northGps2northBase.q = gps2base_.q * north2gps; 
  northGps2northBase.t = gps2base_.t;
  Rigid3d fix_pose1;
  fix_pose1.t = ecef_to_local_frame_.q * LatLongAltToEcef(msg.latitude, msg.longitude,msg.altitude) + ecef_to_local_frame_.t;
  fix_pose1.q = Eigen::Quaterniond::Identity();
  geometry_msgs::PoseStamped temp_pose;

  Rigid3d base2fix =gps_to_base_init_* fix_pose1 *northGps2northBase.inverse();
//   cout << "base2map_gps:" << base2map_gps <<endl;
  Rigid3d base2map = fix_in_map_ * base2fix;
  temp_pose.header.stamp = msg.header.stamp;
  temp_pose.pose.position.x = base2map.t(0);
  temp_pose.pose.position.y = base2map.t(1);
  temp_pose.pose.position.z = base2map.t(2);
//   cout << base2map_gps <<endl;
  gps_mapping_path_.poses.push_back(temp_pose);
  gps_mapping_path_.header.frame_id = odom_frame_;
  if(path_pub_.getNumSubscribers()){
    path_mapping_pub_.publish(gps_mapping_path_);
  }
}

void PubGpsPath::handlePointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  mapping::TrajectoryNode::Data data;
  sensor::PointCloud cloud;
  sensor_msgs::PointCloud2 origin_cloud = *msg;
  sensor_msgs::PointCloud2Iterator<float> iter_x(origin_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(origin_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(origin_cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(origin_cloud, "intensity");
  int range_size = origin_cloud.width * origin_cloud.height;
  for(int i = 0; i < range_size; i++)
  {
    Eigen::Vector3f point;
    point(0) = *iter_x;
    point(1) = *iter_y;
    point(2) = *iter_z;
    float intensity = *iter_intensity;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
    cloud.push_back({point,intensity});
  }
  cloud_times_.push(msg->header.stamp);
  data.high_resolution_point_cloud = cloud;
  data.time = cartographer_ros::FromRos(cloud_times_.back());
  cloud_with_time_[cloud_times_.back()] = data;
  while(cloud_times_.size() > 0)
  {
    geometry_msgs::TransformStamped lidar2odom;
    try
    { 
      lidar2odom = tf_buffer_.lookupTransform(odom_frame_, lidar_frame_, cloud_times_.front());
      Eigen::Vector3d t(lidar2odom.transform.translation.x,
                        lidar2odom.transform.translation.y,
                        lidar2odom.transform.translation.z);
      Eigen::Quaterniond q(lidar2odom.transform.rotation.w, lidar2odom.transform.rotation.x,
                        lidar2odom.transform.rotation.y, lidar2odom.transform.rotation.z);
      Rigid3d lidar2odom_pose(q,t);
      for(auto& point : cloud_with_time_[cloud_times_.back()].high_resolution_point_cloud)
      {
        point.position = lidar2odom_pose * point.position;
      }
      cout << "cloud_times: " << cloud_times_.front() <<endl;
      cloud_times_.pop();
    }
    catch (tf2::TransformException& ex)
    {
      if((cloud_times_.back() - cloud_times_.front()).toSec() > 8.0)
      {
        cloud_with_time_.erase(cloud_with_time_.find(cloud_times_.front()));
        cloud_times_.pop();
      }
      break;
    }

  }
}

bool PubGpsPath::SaveDataSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ofstream outFile;
//连续写入
  outFile.open("/home/cyy/poses_with_time.txt", std::ios::out);
  for(auto it:poses_with_time_)
  {
    ros::Time time = it.first;
    long time_int = std::round(time.toSec() * 1000);
    outFile << time_int <<" ";
    for(auto it2 : it.second)
      outFile << it2.t(0) <<" "<< it2.t(1) <<" "<< it2.t(2) <<" "<< it2.q.w() <<" "<< it2.q.x() <<" "<< it2.q.y() <<" "<< it2.q.z() <<" ";
    outFile <<endl;
  }
  outFile.close();
  
  io::ProtoStreamWriter writer("/home/cyy/1.pbstream");
  int k = 0;
  for (const auto& node : cloud_with_time_) {
    SerializedData proto;
    auto* const node_proto = proto.mutable_node();
    node_proto->mutable_node_id()->set_trajectory_id(0);
    node_proto->mutable_node_id()->set_node_index(k++);
    *node_proto->mutable_node_data() = mapping::ToProto(node.second);
    writer.WriteProto(proto);
  }
  writer.Close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_gps_path_node");
  ros::NodeHandle n;
  string map_name = "/home/cyy/map/puyan_around";
  PubGpsPath gps(n,map_name);
  ros::spin();
  return 1;
}

