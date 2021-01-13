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
  setParam();
  rtk_sub_ = nh_.subscribe("/jzhw/gps/back/fix", 10, &PubRtkPose::HandleGps, this);
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
  max_y_ = 2.5;
  max_z_ = 1.8;
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
  
  point.x = max_x_;
  point.y = max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);

  point.x = max_x_;
  point.y = -max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = max_x_;
  point.y = -max_y_;
  point.z = max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = max_x_;
  point.y = -max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);
  
  point.x = -max_x_;
  point.y = -max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = -max_x_;
  point.y = -max_y_;
  point.z = max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = -max_x_;
  point.y = -max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);
  
  point.x = -max_x_;
  point.y = max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = -max_x_;
  point.y = max_y_;
  point.z = max_z_;
  obj_polygon_.polygon.points.push_back(point);
  point.x = -max_x_;
  point.y = max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);
  
  point.x = max_x_;
  point.y = max_y_;
  point.z = -max_z_;
  obj_polygon_.polygon.points.push_back(point);
  obj_polygon_.header.frame_id = "trailer_link";
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
  geometry_msgs::TransformStamped base2map_tf;
  Rigid3d base2map;
  ros::Rate rate(10);
  int k = 0;
  while(1){
    try {
      base2map_tf = tfBuffer_.lookupTransform("map", "base_footprint",gps_msg->header.stamp);
      base2map.t = Eigen::Vector3d(base2map_tf.transform.translation.x, 
                                   base2map_tf.transform.translation.y,
                                   base2map_tf.transform.translation.z);
      base2map.q = Eigen::Quaterniond(base2map_tf.transform.rotation.w, 
                                      base2map_tf.transform.rotation.x,
                                      base2map_tf.transform.rotation.y,
                                      base2map_tf.transform.rotation.z);
      break;
//          pcl_ros::transformPointCloud("robotarm", cloud_in, cloud_out, tfListener);

    } catch (tf2::TransformException &ex) {
      rate.sleep();
      if(k++ > 200)
      {
        cout << "can not get base2map tf!" << endl;
        return;
      }
    }
  }
  Rigid3d trailer2base = base2map.inverse() * trailer_to_map_gps;
  geometry_msgs::TransformStamped map_pose_tf;
  
  map_pose_tf.header.frame_id = "base_footprint";
  map_pose_tf.header.stamp = gps_msg->header.stamp;
  map_pose_tf.child_frame_id = "trailer_link";
  map_pose_tf.transform.translation.x = trailer2base.t.x();
  map_pose_tf.transform.translation.y = trailer2base.t.y();
  map_pose_tf.transform.translation.z = trailer2base.t.z();
  
  map_pose_tf.transform.rotation.w = trailer2base.q.w();
  map_pose_tf.transform.rotation.x = trailer2base.q.x();
  map_pose_tf.transform.rotation.y = trailer2base.q.y();
  map_pose_tf.transform.rotation.z = trailer2base.q.z();
  tf_broadcaster_.sendTransform(map_pose_tf);
  
  visual_servo_msgs::Model3DPoseArray pub_msg;
  visual_servo_msgs::Model3DPose model_pose_msg;
  model_pose_msg.model_name = "empty";
  model_pose_msg.valid = true;
  model_pose_msg.pose.position.x = trailer2base.t.x();
  model_pose_msg.pose.position.y = trailer2base.t.y();
  model_pose_msg.pose.position.z = trailer2base.t.z();
  
  model_pose_msg.pose.orientation.w = trailer2base.q.w();
  model_pose_msg.pose.orientation.x = trailer2base.q.x();
  model_pose_msg.pose.orientation.y = trailer2base.q.y();
  model_pose_msg.pose.orientation.z = trailer2base.q.z();
  pub_msg.model_pose_array.push_back(model_pose_msg);
  model_pose_pub_.publish(pub_msg);
  
  obj_polygon_.header.stamp = gps_msg->header.stamp;
  polygon_pub_.publish(obj_polygon_);
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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_rtk_pose_node");
  ros::NodeHandle n;
  PubRtkPose p(n);
  ros::spin();
  return 1;
}