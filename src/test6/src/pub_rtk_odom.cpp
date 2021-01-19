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

#include "pub_rtk_odom.h"
// #include <boost/property_tree/ptree.hpp>
// #include <boost/property_tree/xml_parser.hpp>
// #include <boost/iostreams/device/back_inserter.hpp>
// #include <boost/iostreams/filter/gzip.hpp>
// #include <boost/iostreams/filtering_stream.hpp>
// #include <cartographer/io/proto_stream.h>
// #include "cartographer/mapping/proto/serialization.pb.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cartographer_ros/time_conversion.h>
#include <cartographer/sensor/internal/voxel_filter.h>
using namespace std;
using namespace cartographer;
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }
constexpr double kExtrapolationEstimationTimeSec = 0.001;
inline Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

inline Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

inline cartographer::transform::Rigid3d ToRigid3d(const geometry_msgs::TransformStamped& transform) {
  return cartographer::transform::Rigid3d(ToEigen(transform.transform.translation),
                 ToEigen(transform.transform.rotation));
}

inline cartographer::transform::Rigid3d ToRigid3d(const geometry_msgs::Pose& pose) {
  return cartographer::transform::Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                 ToEigen(pose.orientation));
}

inline Eigen::Quaterniond eul2quat(const Eigen::Vector3d& eul)
{
  Eigen::Matrix3d mat;
  mat= 
    Eigen::AngleAxisd(eul[0], ::Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(eul[1], ::Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(eul[2], ::Eigen::Vector3d::UnitX());
  return Eigen::Quaterniond(mat);
}
inline geometry_msgs::Transform ToGeometryMsgTransform(const cartographer::transform::Rigid3d& rigid3d) {
  geometry_msgs::Transform transform;
  transform.translation.x = rigid3d.translation().x();
  transform.translation.y = rigid3d.translation().y();
  transform.translation.z = 0;
  transform.rotation.w = rigid3d.rotation().w();
  transform.rotation.x = rigid3d.rotation().x();
  transform.rotation.y = rigid3d.rotation().y();
  transform.rotation.z = rigid3d.rotation().z();
  return transform;
}
PubRtkOdom::PubRtkOdom(const ros::NodeHandle& n):nh_(n),tfBuffer_{::ros::Duration(10.)},tfListener_(tfBuffer_)
{
  imu_sub_ = nh_.subscribe("/jzhw/imu", 5, &PubRtkOdom::HandleImu, this);
  rtk_sub_ = nh_.subscribe("/jzhw/gps/front/fix", 5, &PubRtkOdom::HandleGps, this);
  init_flag_ = false;
  imu2base_init_ = false;
  initialised_ = false;
  wall_timer_ = nh_.createWallTimer(::ros::WallDuration(0.02),
                          &PubRtkOdom::PubTf, this);
}

PubRtkOdom::~PubRtkOdom()
{
  
}
Eigen::Vector3d PubRtkOdom::LatLongAltToEcef(const double latitude, const double longitude,
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

const transform::Rigid3d PubRtkOdom::ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude, const double altitude) {
  const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, altitude);
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(DegToRad(latitude - 90.),
                        Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(DegToRad(-longitude),
                        Eigen::Vector3d::UnitZ());
  return transform::Rigid3d({rotation * -translation, rotation});
}

void PubRtkOdom::HandleGps(const gps_common::GPSFix::ConstPtr& msg)
{
  if(!imu2base_init_)
    return;
  transform::Rigid3d northGps2northBase;
  double dip = msg->dip * M_PI / 180 ;
  Eigen::Quaterniond north2gps(cos(dip /2),0,0,sin(dip/2));
//   northGps2northBase.q =gps2base_.q * north2gps; 
//   northGps2northBase.t = gps2base_.t;
  northGps2northBase = transform::Rigid3d(gps2base_.translation(), 
                                          gps2base_.rotation() * north2gps);
//   cout << "northBase2northGps:" << northBase2northGps <<endl;
  if(!init_flag_ )
  {
    init_flag_ = true;
    gps_to_base_init_ = northGps2northBase;
    ecef_to_local_frame_ = ComputeLocalFrameFromLatLong(msg->latitude, msg->longitude,0.0);
  }
  transform::Rigid3d fix_pose1;//= ComputeLocalFrameFromLatLong(msg->latitude, msg->longitude,msg->altitude);
  fix_pose1 = transform::Rigid3d(ecef_to_local_frame_.rotation() * LatLongAltToEcef(msg->latitude, msg->longitude,msg->altitude) 
      + ecef_to_local_frame_.translation(), Eigen::Quaterniond::Identity());
  geometry_msgs::PoseStamped temp_pose;
  transform::Rigid3d base2map_gps =gps_to_base_init_* fix_pose1 *northGps2northBase.inverse();
//   cout << "base2map_gps:" << base2map_gps <<endl;
  
  pose_extrapolator_->AddPose(cartographer_ros::FromRos(msg->header.stamp), base2map_gps);
  
}

std::unique_ptr<cartographer::sensor::ImuData> PubRtkOdom::ToImuData(
    const sensor_msgs::Imu::ConstPtr& msg) {
  const cartographer::common::Time time = cartographer_ros::FromRos(msg->header.stamp);
  Eigen::Vector3d linear_acceleration = imu2base_.rotation() * ToEigen(msg->linear_acceleration);
  Eigen::Vector3d angular_velocity = imu2base_.rotation() * (ToEigen(msg->angular_velocity));

  linear_acceleration[0] = 0;
  linear_acceleration[1] = 0;
  linear_acceleration[2] = 10;

  angular_velocity[0] = 0;
  angular_velocity[1] = 0;
  //angular_velocity[2] = 0;

  return absl::make_unique<cartographer::sensor::ImuData>(
      cartographer::sensor::ImuData{
          time,
          linear_acceleration,
          angular_velocity});
}

void PubRtkOdom::HandleImu(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  if(!imu2base_init_)
  {
    try
    {
      geometry_msgs::TransformStamped imu2base_stamp, gps2base_stamp;
      imu2base_stamp =
          tfBuffer_.lookupTransform("base_footprint", imu_msg->header.frame_id, ros::Time(0));
      gps2base_stamp =
          tfBuffer_.lookupTransform("base_footprint", "gnss_link", ros::Time(0));
      gps2base_stamp.transform.rotation.w = 0.997;
      gps2base_stamp.transform.rotation.z = -0.081;
      imu2base_ = ToRigid3d(imu2base_stamp);
      gps2base_ = ToRigid3d(gps2base_stamp);
      imu2base_init_ = true;
      
      LOG(INFO) <<"imu2base: " << imu2base_ ;
      LOG(INFO) <<"gps2base: " << gps2base_;
    }
    catch (tf2::TransformException& ex)
    {
      LOG(WARNING) << "can not get imu2base!";
      return;
    }
  }
  auto imu_data_ptr = ToImuData(imu_msg);
  if (imu_data_ptr != nullptr)
  {
    if(!initialised_){
      pose_extrapolator_ = mapping::PoseExtrapolator::InitializeWithImu(
        ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
        10, *imu_data_ptr);
      initialised_ = true;
    }
    else
      pose_extrapolator_->AddImuData(*imu_data_ptr);
  }
}

void PubRtkOdom::PubTf(const ros::WallTimerEvent& unused_timer_event)
{
  ros::Time time_now = ros::Time::now();
  if(!initialised_)
    return;
  geometry_msgs::TransformStamped stamped_transform;
  const ::cartographer::common::Time now = std::max(
            cartographer_ros::FromRos(ros::Time::now()), pose_extrapolator_->GetLastExtrapolatedTime());
  stamped_transform.header.stamp = cartographer_ros::ToRos(now);
  const cartographer::transform::Rigid3d tracking_to_local_3d = pose_extrapolator_->ExtrapolatePose(now);
  std::vector<geometry_msgs::TransformStamped> stamped_transforms;
  stamped_transform.header.frame_id =
      "odom";
  stamped_transform.child_frame_id =
      "base_footprint";
  stamped_transform.transform = ToGeometryMsgTransform(tracking_to_local_3d);
  stamped_transforms.push_back(stamped_transform);
  tf_broadcaster_.sendTransform(stamped_transforms);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_rtk_odom_node");
  ros::NodeHandle n;
  PubRtkOdom p(n);
  ros::spin();
  return 1;
}
