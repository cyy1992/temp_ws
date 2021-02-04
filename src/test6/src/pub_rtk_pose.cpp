/*
 * Copyright 2021 <copyright holder> <email>
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

#include "pub_rtk_pose.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <visual_servo_msgs/Model3DPoseArray.h>

using namespace std;
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }
inline Eigen::Quaterniond eul2quat(const Eigen::Vector3d& eul)
{
  Eigen::Matrix3d mat;
  mat= 
    Eigen::AngleAxisd(eul[0], ::Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(eul[1], ::Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(eul[2], ::Eigen::Vector3d::UnitX());
  return Eigen::Quaterniond(mat);
}
PubRtkPose::PubRtkPose(const ros::NodeHandle& n):nh_(n),tfBuffer_{::ros::Duration(10.)}, tfListener_(tfBuffer_)
{
  model_pose_pub_ = nh_.advertise<visual_servo_msgs::Model3DPoseArray>("/servo_guard/model_3d_pose",3);
  polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/trailer_obj",3);
  zb_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/zb_polygon",3);
  car_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/car_polygon",3);
  setParam();
  rtk_sub_ = nh_.subscribe("/jzhw/gps/back/fix", 10, &PubRtkPose::HandleGps, this);
  zb_sub_ = nh_.subscribe("/zb_pose", 10, &PubRtkPose::HandleZbPose, this);
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/zb_markers",3);
  zb_markers_.markers.resize(6);
  for(int i = 0; i <6; i++)
  {
    zb_markers_.markers[i].header.frame_id = "map";
  }
}


PubRtkPose::~PubRtkPose()
{

}

void PubRtkPose::setParam()
{
  boost::property_tree::ptree pt;
  string map_path;
  ros::param::get("/vtr/path",map_path);
  cout  << map_path << "/gps_data.xml! " <<endl;
  boost::property_tree::xml_parser::read_xml(map_path+"/gps_data.xml", pt);
  if(pt.empty())
  {
    cout << "Can not get gps_data.xml! " <<endl;
    return;
  }
  double pose[7];
  string data = pt.get<std::string>("ecef_to_local_frame_pose");
  
  sscanf(data.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &pose[0], &pose[1], &pose[2],
      &pose[3], &pose[4], &pose[5], &pose[6]);
  cout <<  "ecef_to_local_frame_pose: " <<endl;
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
  cout << "fix_frame_origin_in_map: " <<fix_in_map_<<endl;
  cout << "ecef_to_local_frame: " <<ecef_to_local_frame_<<endl;
  cout << "ecef_in_map_: " <<ecef_in_map_<<endl;
  cout << "gps_to_base_init_: " <<gps_to_base_init_<<endl;
  
  
  if(pose[0] !=0 || pose[1] != 0 || pose[2] != 0 || pose[5]!=0)
    has_gps_data_info_ = true;
  else
    cout << "No valid gps data!" <<endl;
  
  ros::param::get("/jzhw/calib/trailer/x", gps2trailer_.t(0));
  ros::param::get("/jzhw/calib/trailer/y", gps2trailer_.t(1));
  ros::param::get("/jzhw/calib/trailer/z", gps2trailer_.t(2));
  
  Eigen::Vector3d eul_gps2trailer;
  ros::param::get("/jzhw/calib/trailer/yaw", eul_gps2trailer(0));
  ros::param::get("/jzhw/calib/trailer/pitch", eul_gps2trailer(1));
  ros::param::get("/jzhw/calib/trailer/roll", eul_gps2trailer(2));
  gps2trailer_.q = eul2quat(eul_gps2trailer);
  cout << "gps2trailer: " <<gps2trailer_ <<endl;
  float max_x_,max_y_,max_z_;
  max_x_ = 4.2;
  max_y_ = 2;
  max_z_ = 0;
  ros::param::get("/jzhw/calib/trailer/max_x", max_x_);
  ros::param::get("/jzhw/calib/trailer/max_y", max_y_);
  ros::param::get("/jzhw/calib/trailer/max_z", max_z_);
  cout << "max: " << max_x_ << ", " << max_y_<< ", " << max_y_<<endl;
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
  obj_polygon_.header.frame_id = "trailer_link";
  
  hook1_in_trailer_.t = Eigen::Vector3d(-max_x_, max_y_-0.13, 0);
  hook2_in_trailer_.t = Eigen::Vector3d(-max_x_, -max_y_+0.13, 0);
  
  point.x = 0;
  point.y = 2.25;
  point.z = 0;
  car_polygon_.polygon.points.push_back(point);
  point.x = 0;
  point.y = -1.05;
  car_polygon_.polygon.points.push_back(point);
  
  point.x = -3.4;
  car_polygon_.polygon.points.push_back(point);
  point.y = 2.25;
  car_polygon_.polygon.points.push_back(point);
  point.x = 0;
  car_polygon_.polygon.points.push_back(point);
  car_polygon_.header.frame_id = "base_footprint";
//   point.x = max_x_;
//   point.y = max_y_;
//   point.z = -max_z_;
//   obj_polygon_.polygon.points.push_back(point);
// 
//   point.x = max_x_;
//   point.y = -max_y_;
//   point.z = -max_z_;
//   obj_polygon_.polygon.points.push_back(point);
//   point.x = max_x_;
//   point.y = -max_y_;
//   point.z = max_z_;
//   obj_polygon_.polygon.points.push_back(point);
//   point.x = max_x_;
//   point.y = -max_y_;
//   point.z = -max_z_;
//   obj_polygon_.polygon.points.push_back(point);
//   
//   point.x = -max_x_;
//   point.y = -max_y_;
//   point.z = -max_z_;
//   obj_polygon_.polygon.points.push_back(point);
//   point.x = -max_x_;
//   point.y = -max_y_;
//   point.z = max_z_;
//   obj_polygon_.polygon.points.push_back(point);
//   point.x = -max_x_;
//   point.y = -max_y_;
//   point.z = -max_z_;
//   obj_polygon_.polygon.points.push_back(point);
//   
//   point.x = -max_x_;
//   point.y = max_y_;
//   point.z = -max_z_;
//   obj_polygon_.polygon.points.push_back(point);
//   point.x = -max_x_;
//   point.y = max_y_;
//   point.z = max_z_;
//   obj_polygon_.polygon.points.push_back(point);
//   point.x = -max_x_;
//   point.y = max_y_;
//   point.z = -max_z_;
//   obj_polygon_.polygon.points.push_back(point);
//   
//   point.x = max_x_;
//   point.y = max_y_;
//   point.z = -max_z_;
//   obj_polygon_.polygon.points.push_back(point);
  
  
  float max_x1_ = 0.0;
  float max_y1_ = 2;
  float max_z1_ = 0;
  point.x = max_x1_;
  point.y = max_y1_;
  point.z = max_z1_;
  zb_polygon_.polygon.points.push_back(point);
  point.x = max_x1_;
  point.y = -max_y1_;
  point.z = max_z1_;
  zb_polygon_.polygon.points.push_back(point);
  
  
  
//   point.x = -max_x1_;
//   point.y = -max_y1_;
//   point.z = max_z1_;
//   zb_polygon_.polygon.points.push_back(point);
//   point.x = -max_x1_;
//   point.y = max_y1_;
//   point.z = max_z1_;
//   zb_polygon_.polygon.points.push_back(point);
//   point.x = max_x1_;
//   point.y = max_y1_;
//   point.z = max_z1_;
//   zb_polygon_.polygon.points.push_back(point);
//   
//   point.x = max_x1_;
//   point.y = max_y1_;
//   point.z = -max_z1_;
//   zb_polygon_.polygon.points.push_back(point);
// 
//   point.x = max_x1_;
//   point.y = -max_y1_;
//   point.z = -max_z1_;
//   zb_polygon_.polygon.points.push_back(point);
//   point.x = max_x1_;
//   point.y = -max_y1_;
//   point.z = max_z1_;
//   zb_polygon_.polygon.points.push_back(point);
//   point.x = max_x1_;
//   point.y = -max_y1_;
//   point.z = -max_z1_;
//   zb_polygon_.polygon.points.push_back(point);
//   
//   point.x = -max_x1_;
//   point.y = -max_y1_;
//   point.z = -max_z1_;
//   zb_polygon_.polygon.points.push_back(point);
//   point.x = -max_x1_;
//   point.y = -max_y1_;
//   point.z = max_z1_;
//   zb_polygon_.polygon.points.push_back(point);
//   point.x = -max_x1_;
//   point.y = -max_y1_;
//   point.z = -max_z1_;
//   zb_polygon_.polygon.points.push_back(point);
//   
//   point.x = -max_x1_;
//   point.y = max_y1_;
//   point.z = -max_z1_;
//   zb_polygon_.polygon.points.push_back(point);
//   point.x = -max_x1_;
//   point.y = max_y1_;
//   point.z = max_z1_;
//   zb_polygon_.polygon.points.push_back(point);
//   point.x = -max_x1_;
//   point.y = max_y1_;
//   point.z = -max_z1_;
//   zb_polygon_.polygon.points.push_back(point);
//   
//   point.x = max_x1_;
//   point.y = max_y1_;
//   point.z = -max_z1_;
//   zb_polygon_.polygon.points.push_back(point);
  zb_polygon_.header.frame_id = "zb_link";
  
  zb_in_map_tf_.transform.translation.x = 0;
  zb_in_map_tf_.transform.translation.y = 0;
  zb_in_map_tf_.transform.translation.z = 0;
    
  zb_in_map_tf_.transform.rotation.x = 0;
  zb_in_map_tf_.transform.rotation.y = 0;
  zb_in_map_tf_.transform.rotation.z = 0;
  zb_in_map_tf_.transform.rotation.w = 1;
  zb_in_map_tf_.header.frame_id = "map";
  zb_in_map_tf_.child_frame_id = "zb_link";
}

void PubRtkPose::HandleGps(const gps_common::GPSFix::ConstPtr &gps_msg)
{
  if(!has_gps_data_info_)
    return;
  if(gps_msg->status.status != 2){
    cout << "err: rtk status is not 2!" <<endl;
    return;
  }
  Rigid3d northGps2northBase;
  double dip = gps_msg->dip * M_PI / 180 ;
  Eigen::Quaterniond north2gps(cos(dip /2),0,0,sin(dip/2));
  northGps2northBase.q =gps2trailer_.q * north2gps; 
  northGps2northBase.t = gps2trailer_.t;
  
  Rigid3d fix_pose1;
  fix_pose1.t = ecef_to_local_frame_.q * LatLongAltToEcef(gps_msg->latitude, gps_msg->longitude,gps_msg->altitude) + ecef_to_local_frame_.t;
  fix_pose1.q = Eigen::Quaterniond::Identity();
  geometry_msgs::PoseStamped temp_pose;

  Rigid3d base2fix =gps_to_base_init_* fix_pose1 *northGps2northBase.inverse();
//   cout << "base2map_gps:" << base2map_gps <<endl;
  Rigid3d trailer_to_map_gps = fix_in_map_ * base2fix;
//   geometry_msgs::TransformStamped base2map_tf;
//   Rigid3d base2map;
//   ros::Rate rate(10);
//   int k = 0;
//   while(1){
//     try {
//       base2map_tf = tfBuffer_.lookupTransform("map", "base_footprint",gps_msg->header.stamp);
//       base2map.t = Eigen::Vector3d(base2map_tf.transform.translation.x, 
//                                    base2map_tf.transform.translation.y,
//                                    base2map_tf.transform.translation.z);
//       base2map.q = Eigen::Quaterniond(base2map_tf.transform.rotation.w, 
//                                       base2map_tf.transform.rotation.x,
//                                       base2map_tf.transform.rotation.y,
//                                       base2map_tf.transform.rotation.z);
//       break;
// //          pcl_ros::transformPointCloud("robotarm", cloud_in, cloud_out, tfListener);
// 
//     } catch (tf2::TransformException &ex) {
//       rate.sleep();
//       if(k++ > 200)
//       {
//         cout << "can not get base2map tf!" << endl;
//         return;
//       }
//     }
//   }
//   Rigid3d trailer2base = base2map.inverse() * trailer_to_map_gps;
  vector<geometry_msgs::TransformStamped> tfs;
  geometry_msgs::TransformStamped map_pose_tf;
  
  map_pose_tf.header.frame_id = "map";
  map_pose_tf.header.stamp = gps_msg->header.stamp;
  map_pose_tf.child_frame_id = "trailer_link";
  map_pose_tf.transform.translation.x = trailer_to_map_gps.t.x();
  map_pose_tf.transform.translation.y = trailer_to_map_gps.t.y();
  map_pose_tf.transform.translation.z = trailer_to_map_gps.t.z();
  
  map_pose_tf.transform.rotation.w = trailer_to_map_gps.q.w();
  map_pose_tf.transform.rotation.x = trailer_to_map_gps.q.x();
  map_pose_tf.transform.rotation.y = trailer_to_map_gps.q.y();
  map_pose_tf.transform.rotation.z = trailer_to_map_gps.q.z();
  tfs.push_back(map_pose_tf);
  zb_in_map_tf_.header.stamp = ros::Time::now();
  tfs.push_back(zb_in_map_tf_);
  tf_broadcaster_.sendTransform(tfs);
  
  visual_servo_msgs::Model3DPoseArray pub_msg;
  visual_servo_msgs::Model3DPose model_pose_msg;
  model_pose_msg.model_name = "empty";
  model_pose_msg.valid = true;
  model_pose_msg.pose.position.x = trailer_to_map_gps.t.x();
  model_pose_msg.pose.position.y = trailer_to_map_gps.t.y();
  model_pose_msg.pose.position.z = trailer_to_map_gps.t.z();
  
  model_pose_msg.pose.orientation.w = trailer_to_map_gps.q.w();
  model_pose_msg.pose.orientation.x = trailer_to_map_gps.q.x();
  model_pose_msg.pose.orientation.y = trailer_to_map_gps.q.y();
  model_pose_msg.pose.orientation.z = trailer_to_map_gps.q.z();
  pub_msg.model_pose_array.push_back(model_pose_msg);
  model_pose_pub_.publish(pub_msg);
  
  obj_polygon_.header.stamp = gps_msg->header.stamp;
  polygon_pub_.publish(obj_polygon_);
  zb_polygon_.header.stamp = gps_msg->header.stamp;
  zb_polygon_pub_.publish(zb_polygon_);
  car_polygon_.header.stamp = gps_msg->header.stamp;
  car_polygon_pub_.publish(car_polygon_);
  
  Rigid3d hook1_in_map = trailer_to_map_gps * hook1_in_trailer_;
  Rigid3d hook2_in_map = trailer_to_map_gps * hook2_in_trailer_;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";  
//   marker.
  marker.pose.position.x = hook1_in_map.t.x();
  marker.pose.position.y = hook1_in_map.t.y();
  marker.pose.position.z = hook1_in_map.t.z();
  
  marker.pose.orientation.x = hook1_in_map.q.x();
  marker.pose.orientation.y = hook1_in_map.q.y();
  marker.pose.orientation.z = hook1_in_map.q.z();
  marker.pose.orientation.w = hook1_in_map.q.w();
  
  marker.color.r = 255;
  marker.color.g = 0;
  marker.color.b = 255;
  marker.color.a = 1;
  marker.scale.x = 0.5;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.ns = "hook";
  marker.type = 1;
  marker.id = 3;
  zb_markers_.markers[2] = marker;
  
  marker.pose.position.x = hook2_in_map.t.x();
  marker.pose.position.y = hook2_in_map.t.y();
  marker.pose.position.z = hook2_in_map.t.z(); 
  marker.pose.orientation.x = hook2_in_map.q.x();
  marker.pose.orientation.y = hook2_in_map.q.y();
  marker.pose.orientation.z = hook2_in_map.q.z();
  marker.pose.orientation.w = hook2_in_map.q.w();
  marker.id = 4;
  zb_markers_.markers[3] = marker;
  
  
  marker.header.frame_id="zb_link";
  marker.ns = "distance";
  marker.action = visualization_msgs::Marker::ADD;
  marker.id =5;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.scale.x = 2;
  marker.scale.y = 2;
  marker.scale.z = 2;
  marker.color.b = 255;
  marker.color.g = 0;
  marker.color.r = 255;
  marker.color.a = 1;

  geometry_msgs::Pose pose;
  pose.position.x =  -2;
  pose.position.y =  5;
  pose.position.z =0;
//   ostringstream str;
//   str<<k;
  double distance1 = (hook1_in_map.t.x()- ear1_in_map_.t.x()) *(hook1_in_map.t.x()- ear1_in_map_.t.x()) +
    (hook1_in_map.t.y()- ear1_in_map_.t.y()) * (hook1_in_map.t.y()- ear1_in_map_.t.y());
  marker.text= to_string(sqrt(distance1));
  marker.pose=pose;
  zb_markers_.markers[4] = marker;
  pose.position.y = -5;
  marker.pose=pose;
  
  double distance2 = (hook2_in_map.t.x()- ear2_in_map_.t.x()) *(hook2_in_map.t.x()- ear2_in_map_.t.x()) +
    (hook2_in_map.t.y()- ear2_in_map_.t.y()) * (hook2_in_map.t.y()- ear2_in_map_.t.y());
  marker.text= to_string(sqrt(distance2));
  marker.id =6;
  zb_markers_.markers[5] = marker;
  for(int i =0; i < 6; i++){
    zb_markers_.markers[i].header.stamp = ros::Time::now();
  }
  markers_pub_.publish(zb_markers_);
}


Eigen::Vector3d PubRtkPose::LatLongAltToEcef(const double latitude, const double longitude,
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

const PubRtkPose::Rigid3d PubRtkPose::ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude, const double altitude) {
  const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, altitude);
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(DegToRad(latitude - 90.),
                        Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(DegToRad(-longitude),
                        Eigen::Vector3d::UnitZ());
  return PubRtkPose::Rigid3d({rotation * -translation, rotation});
}

void PubRtkPose::HandleZbPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  
  zb_in_map_tf_.transform.translation.x = msg->position.x;
  zb_in_map_tf_.transform.translation.y = msg->position.y;
  zb_in_map_tf_.transform.translation.z = msg->position.z;
  
  Eigen::Quaterniond q(cos(msg->orientation.w /2),0,0,sin(msg->orientation.w/2));
  
  zb_in_map_tf_.transform.rotation.x = q.x();
  zb_in_map_tf_.transform.rotation.y = q.y();
  zb_in_map_tf_.transform.rotation.z = q.z();
  zb_in_map_tf_.transform.rotation.w = q.w();
  Rigid3d zb_in_map;
  zb_in_map.t = Eigen::Vector3d(msg->position.x, 
                                msg->position.y, 
                                msg->position.z);
  zb_in_map.q = q;
  Rigid3d ear1_in_zb, ear2_in_zb;
  ear1_in_zb.t = Eigen::Vector3d(0, 1.87, 0);
  ear2_in_zb.t = Eigen::Vector3d(0, -1.87, 0);
  ear1_in_map_ = zb_in_map * ear1_in_zb; 
  ear2_in_map_ = zb_in_map * ear2_in_zb;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  
//   marker.
  marker.pose.position.x = ear1_in_map_.t.x();
  marker.pose.position.y = ear1_in_map_.t.y();
  marker.pose.position.z = ear1_in_map_.t.z();
  
  marker.pose.orientation.x = ear1_in_map_.q.x();
  marker.pose.orientation.y = ear1_in_map_.q.y();
  marker.pose.orientation.z = ear1_in_map_.q.z();
  marker.pose.orientation.w = ear1_in_map_.q.w();
  
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1;
  marker.scale.x = 0.26;
  marker.scale.y = 0.36;
  marker.scale.z = 0.36;
  marker.ns = "zb";
  marker.type = 1;
  marker.id = 1;
  zb_markers_.markers[0] = marker;
  
  marker.pose.position.x = ear2_in_map_.t.x();
  marker.pose.position.y = ear2_in_map_.t.y();
  marker.pose.position.z = ear2_in_map_.t.z();
  
  marker.pose.orientation.x = ear2_in_map_.q.x();
  marker.pose.orientation.y = ear2_in_map_.q.y();
  marker.pose.orientation.z = ear2_in_map_.q.z();
  marker.pose.orientation.w = ear2_in_map_.q.w();
  
//   marker.color.b = 255;
//   marker.color.g = 255;
//   marker.color.r = 0;
  
  marker.scale.x = 0.26;
  marker.scale.y = 0.36;
  marker.scale.z = 0.36;
  
  marker.id = 2;
  zb_markers_.markers[1] = marker;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_rtk_pose_node");
  ros::NodeHandle n;
  PubRtkPose p(n);
  ros::spin();
  return 1;
}