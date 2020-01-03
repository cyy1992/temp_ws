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

#include <time.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <vtr_msgs/SubCloud.h>
//#include <vtr_msgs/IcpInfo.h>

#include "absl/memory/memory.h"
#include "cartographer/common/time.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros/msg_conversion.h"
#include "pointMatcherMapping/point_matcher_mapping.h"
#include "pointMatcherMapping/sliding_mapper_utils.h"

using namespace std;
using namespace PointMatcherSupport;
using namespace cartographer;
using namespace cartographer_ros;

namespace point_matcher_mapping{
//#define TIME_TEST
pointMatcherMapping::pointMatcherMapping(ros::NodeHandle& nh)
    : nh_(nh), cloudInLocal_(nullptr),
      transformation_(
          PM::get().REG(Transformation).create("RigidTransformation")),
#if BOOST_VERSION >= 104100
      map_building_in_progress_(false),
#endif // BOOST_VERSION >= 104100
      min_reading_point_cnt_(getParam<int>("minReadingPointCount", 1000)),
      min_map_point_cnt_(getParam<int>("minMapPointCount", 5000000)),
      min_overlap_(getParam<double>("minOverlap", 0.5)),
      max_overlap_to_merge_(getParam<double>("maxOverlapToMerge", 0.9)),
      max_dist_new_point_(pow(getParam<double>("maxDistNewPoint", 0.5), 2)),
      tf_refresh_period_(getParam<double>("tfRefreshPeriod", 0.02)),
      base_frame_(getParam<string>("base_footprint_frame", "base_footprint")),
      odom_frame_(getParam<string>("odom_frame", "odom")),
      // map point update in consideration of origin
      map_pts_origin_interval_(getParam<int>("mapPtsOriginInterval", 1)),
      map_pts_origin_max_dist_(getParam<int>("mapPtsOriginMaxDist", 30)),
      // dynamic
      prior_static_(getParam<double>("priorStatic", 0.5)),
      prior_dyn_(getParam<double>("priorDyn", 0.5)),
      max_angle_(getParam<double>("maxAngle", 0.02)),
      eps_a_(getParam<double>("eps_a", 0.05)),
      eps_d_(getParam<double>("eps_d", 0.02)),
      alpha_(getParam<double>("alpha", 0.99)),
      beta_(getParam<double>("beta", 0.99)),
      max_dyn_(getParam<double>("maxDyn", 0.9)),
      // dynT(getParam<double>("dynThreshold", 0.95)),
      use_imu_data_(getParam<bool>("subscribe_imu", true)),
      tfBuffer_{::ros::Duration(30)}, tfListener_(tfBuffer_), eps_(0.0001),
      close_flag_(false)
{
  TLocalToLastPubBase_ = PM::TransformationParameters::Identity(4, 4);

  // set logger
  if (getParam<bool>("useROSLogger", false))
    PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);

  // load configs
  string configFileName;
  if (ros::param::get("~icpConfig", configFileName))
  {
    ifstream ifs(configFileName.c_str());
    if (ifs.good())
      icp_.loadFromYaml(ifs);
    else
    {
      ROS_ERROR_STREAM("sliding_mapper: cannot load ICP config from YAML file, "
                       "sliding_mapper: "
                       << configFileName);
      // icp_.setDefault();
      exit(0);
    }
  }
  else
  {
    ROS_ERROR_STREAM("sliding_mapper: no ICP config file given!");
    // icp_.setDefault();
    exit(0);
  }

  if (getParam<bool>("useROSLogger", false))
    PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);

  if (ros::param::get("~inputFiltersConfig", configFileName))
  {
    ifstream ifs(configFileName.c_str());
    if (ifs.good())
      inputFilters_ = PM::DataPointsFilters(ifs);
    else
    {
      ROS_ERROR_STREAM("sliding_mapper: cannot load input filters config from "
                       "YAML file, sliding_mapper: "
                       << configFileName);
      exit(0);
    }
  }
  else
  {
    ROS_ERROR_STREAM("sliding_mapper: no input filters config file given, not "
                     "using these filters");
    exit(0);
  }

  if (ros::param::get("~mapPostFiltersConfig", configFileName))
  {
    ifstream ifs(configFileName.c_str());
    if (ifs.good())
      mapPostFilters_ = PM::DataPointsFilters(ifs);
    else
    {
      ROS_ERROR_STREAM("sliding_mapper: cannot load map post-filters config "
                       "from YAML file, sliding_mapper: "
                       << configFileName);
      exit(0);
    }
  }
  else
  {
    ROS_INFO_STREAM("sliding_mapper: no map post-filters config file given, "
                    "not using these filters");
    exit(0);
  }

  max_valid_origin_ = ceil(map_pts_origin_max_dist_ / map_pts_origin_interval_);
  map_pts_origin_dists_.reserve(max_valid_origin_ + 1);
  map_pts_origin_dists_.clear();
  
  // topics and services initialization
//   if (getParam<bool>("subscribe_imu", true))
//     imu_sub_ = nh_.subscribe("imu_in", 10, &pointMatcherMapping::addImuMsg, this);
//   if (getParam<bool>("subscribe_odom", true))
//     odom_sub_ = nh_.subscribe("odom_in", 10, &pointMatcherMapping::addOdomMsg, this);
//   if (getParam<bool>("subscribe_cloud", true))
//     cloud_sub_ = nh_.subscribe("cloud_in", 3, &pointMatcherMapping::addCloudMsg, this);

  constexpr double kExtrapolationEstimationTimeSec = 0.001;
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.
  extrapolator_pub_ = absl::make_unique<mapping::PoseExtrapolator>(
      ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
      10.);
  newestPoseValid_ = false;

  submap_pub_ = nh_.advertise<vtr_msgs::SubCloud>("sliding_map", 3, true);
  submap_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("sliding_cloud", 3, true);
  new_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("new_cloud", 3, true);
  lidar_odometry_pub_ =
      nh_.advertise<nav_msgs::Odometry>("lidar_odometry", 3, true);
//  icp_info_pub_ = nh_.advertise<vtr_msgs::IcpInfo>("icp_info", 3, true);
  std::thread mthread(&pointMatcherMapping::computeLoop,this);
  mthread.detach();
}

pointMatcherMapping::~pointMatcherMapping()
{
#if BOOST_VERSION >= 104100
  // wait for map-building thread
  if (map_building_in_progress_)
  {
    map_building_future_.wait();
    if (map_building_future_.has_value())
      delete map_building_future_.get();
  }
#endif // BOOST_VERSION >= 104100
       // wait for publish thread
  // publishThread.join();
  // save point cloud
  if (cloudInLocal_)
  {
    cloudInLocal_->save("finalMap.vtk");
    delete cloudInLocal_;
  }
}

void pointMatcherMapping::InitializeExtrapolator(const common::Time time)
{
  if (extrapolator_ != nullptr)
  {
    return;
  }
  // We derive velocities from poses which are at least 1 ms apart for numerical
  // stability. Usually poses known to the extrapolator will be further apart
  // in time and thus the last two are used.
  constexpr double kExtrapolationEstimationTimeSec = 0.001;
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.
  extrapolator_ = absl::make_unique<mapping::PoseExtrapolator>(
      ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
      10.);
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());

  tfPub_timer_ = nh_.createWallTimer(::ros::WallDuration(tf_refresh_period_),
                                     &pointMatcherMapping::publishTf, this);
}

void pointMatcherMapping::addImuMsg(const sensor_msgs::ImuConstPtr& msg)
{
  CHECK_NE(msg->linear_acceleration_covariance[0], -1)
      << "Your IMU data claims to not contain linear acceleration measurements "
         "by setting linear_acceleration_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  CHECK_NE(msg->angular_velocity_covariance[0], -1)
      << "Your IMU data claims to not contain angular velocity measurements "
         "by setting angular_velocity_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

  const common::Time time = FromRos(msg->header.stamp);
  // LOG(WARNING) << "imuData:" << time;

  geometry_msgs::TransformStamped sensor2base_stamped;
  try
  {
    string imu_frame = msg->header.frame_id;
    if(imu_frame.at(0) == '/')
      imu_frame.erase(0, 1);

    sensor2base_stamped =
        tfBuffer_.lookupTransform(base_frame_, imu_frame, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1, "sensor2base_stamped: %s", ex.what());
    return;
  }

  const auto sensor_to_tracking =
      absl::make_unique<::cartographer::transform::Rigid3d>(
          ToRigid3d(sensor2base_stamped));

  CHECK(sensor_to_tracking->translation().norm() < 1e-5)
      << "The IMU frame must be colocated with the tracking frame. "
         "Transforming linear acceleration into the tracking frame will "
         "otherwise be imprecise.";

  Eigen::Vector3d linear_acceleration =
      sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration);
  Eigen::Vector3d angular_velocity =
      sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity);

  //  LOG(INFO) << linear_acceleration;
  //  LOG(WARNING) << angular_velocity;

  //  linear_acceleration[0] = 0;
  //  linear_acceleration[1] = 0;
  //  linear_acceleration[2] = -10;
  //  angular_velocity[0] = angular_velocity_msg[2];
  //  angular_velocity[1] = -angular_velocity_msg[0];
  //  angular_velocity[0] = 0;
  //  angular_velocity[1] = 0;
  // angular_velocity[2] = angular_velocity_msg[1];

  // angular_velocity = angular_velocity * 1.1;

  std::unique_ptr<sensor::ImuData> imu_data =
      absl::make_unique<sensor::ImuData>(
          sensor::ImuData{time, linear_acceleration, angular_velocity});

  //  Eigen::Vector3f omega, acc;
  //  omega[0] = imu_msg->angular_velocity.x;
  //  omega[1] = imu_msg->angular_velocity.z;
  //  omega[2] = -imu_msg->angular_velocity.y;
  //  acc[0] = imu_msg->linear_acceleration.x;
  //  acc[1] = -imu_msg->linear_acceleration.z;
  //  acc[2] = imu_msg->linear_acceleration.y;

  {
#ifdef SLIDING_MAPPER_MULTI_THREAD
    std::unique_lock<std::mutex> lck(extrapolator_pub_mtx_);
#endif

    if (imu_data->time < extrapolator_pub_->GetLastPoseTime())
    {
      LOG(WARNING) << imu_data->time << " < "
                   << extrapolator_pub_->GetLastPoseTime();
      return;
    }
    extrapolator_pub_->AddImuData(*imu_data);
  }

  addImuData(sensor::ImuData{imu_data->time, imu_data->linear_acceleration,
                             imu_data->angular_velocity});
}

//std::vector<sensor::ImuData> vecImu;

void pointMatcherMapping::addImuData(const sensor::ImuData& imu_data)
{
#ifdef SLIDING_MAPPER_MULTI_THREAD
  std::unique_lock<std::mutex> lck(extrapolator_mtx_);
#endif

//  vecImu.push_back(imu_data);
//  if(vecImu.size() > 200)
//  {
//    vecImu.erase(vecImu.begin());
//    common::Duration dTime = vecImu[200].time - vecImu[0].time;
//    double hz = 200./common::ToSeconds(dTime);
//    LOG_EVERY_N(WARNING, 50) << "Imu hz: " << hz;
//  }

  CHECK(use_imu_data_) << "An unexpected IMU packet was added.";
  // LOG(WARNING) << "Got IMU!";

  if (extrapolator_ != nullptr &&
      imu_data.time < extrapolator_->GetLastPoseTime())
  {
    LOG(WARNING) << imu_data.time << " < " << extrapolator_->GetLastPoseTime();
    return;
  }

  InitializeExtrapolator(imu_data.time);

  extrapolator_->AddImuData(imu_data);
}

void pointMatcherMapping::addOdomMsg(const nav_msgs::OdometryConstPtr& msg)
{
  const common::Time time = FromRos(msg->header.stamp);
  // LOG(WARNING) << "odomData:" << time;

  std::unique_ptr<sensor::OdometryData> odometry_data =
      absl::make_unique<sensor::OdometryData>(
          sensor::OdometryData{time, ToRigid3d(msg->pose.pose),
            Eigen::Vector3d(msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z),
            Eigen::Vector3d(msg->twist.twist.angular.x,msg->twist.twist.angular.y,msg->twist.twist.angular.z)});

  {
#ifdef SLIDING_MAPPER_MULTI_THREAD
    std::unique_lock<std::mutex> lck(extrapolator_pub_mtx_);
#endif

    if (odometry_data->time < extrapolator_pub_->GetLastPoseTime())
    {
      LOG(WARNING) << odometry_data->time << " < "
                   << extrapolator_pub_->GetLastPoseTime();
      return;
    }

    extrapolator_pub_->AddOdometryData(*odometry_data);
  }

  addOdom(sensor::OdometryData{odometry_data->time, odometry_data->pose});
}

void pointMatcherMapping::addOdom(const sensor::OdometryData& odometry_data)
{
#ifdef SLIDING_MAPPER_MULTI_THREAD
  std::unique_lock<std::mutex> lck(extrapolator_mtx_);
#endif

  if (extrapolator_ == nullptr)
  {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }

  if (odometry_data.time < extrapolator_->GetLastPoseTime())
  {
    LOG(WARNING) << odometry_data.time << " < "
                 << extrapolator_->GetLastPoseTime();
    return;
  }

  extrapolator_->AddOdometryData(odometry_data);
}

void pointMatcherMapping::addCloudMsg(const sensor_msgs::PointCloud2& cloudMsgIn)
{
  ros::Time start, end;
  start = ros::Time::now();

  unique_ptr<DP> cloud(
      new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));

  addCloudData(move(cloud), cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp,
               cloudMsgIn.header.seq);

  end = ros::Time::now();
  LOG_EVERY_N(INFO, 1) << "Process cloud used time: " << (end-start).toSec() << "s";
}

void pointMatcherMapping::addCloudData(unique_ptr<DP> newPointCloud,
                          const std::string& scannerFrame,
                          const ros::Time& stamp, uint32_t seq)
{
  common::Time time = FromRos(stamp);
  // LOG(WARNING) << "cloudData:" << stamp;

  // Initialize extrapolator now if we do not ever use an IMU.
  if (!use_imu_data_)
  {
#ifdef SLIDING_MAPPER_MULTI_THREAD
    std::unique_lock<std::mutex> lck(extrapolator_mtx_);
#endif
    InitializeExtrapolator(time);
  }

  if (extrapolator_ == nullptr)
  {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }

  common::Time time_first_point = time;
  if (time_first_point < extrapolator_->GetLastPoseTime())
  {
    LOG(INFO) << "Extrapolator is still initializing.";
    return;
  }

  int markerProcess = -1;

  // if the future has completed, use the new map
  processNewMapIfAvailable();
  // cerr << "received new map" << endl;

  // IMPORTANT:  We need to receive the point clouds in local coordinates
  // (scanner or robot)

  // Convert point cloud
  const size_t goodCount(newPointCloud->features.cols());
  if (goodCount == 0)
  {
    ROS_ERROR("I found no good points in the cloud");
    return;
  }
  // Dimension of the point cloud, important since we handle 2D and 3D
  const int dimp1 = 4; //(newPointCloud->features.rows());

  // Fetch transformation from scanner to odom
  // Note: we don't need to wait for transform. It is already called in
  // transformListenerToEigenMatrix()
  PM::TransformationParameters TBaseToSensor;
  try
  {
    geometry_msgs::TransformStamped base2sensor_stamped;
    base2sensor_stamped =
        tfBuffer_.lookupTransform(scannerFrame, base_frame_, ros::Time(0));

    transform::Rigid3d base2sensor_d = ToRigid3d(base2sensor_stamped);
    transform::Rigid3f base2sensor_f = base2sensor_d.cast<float>();
    TBaseToSensor = PM::TransformationParameters::Identity(dimp1, dimp1);
    TBaseToSensor.block(0, 0, 3, 3) =
        base2sensor_f.rotation().toRotationMatrix();
    TBaseToSensor(0, 3) = base2sensor_f.translation()[0];
    TBaseToSensor(1, 3) = base2sensor_f.translation()[1];
    TBaseToSensor(2, 3) = base2sensor_f.translation()[2];
  }
  catch (tf::ExtrapolationException e)
  {
    ROS_ERROR_STREAM("sliding_mapper: error in reading TF from base to sensor");
    ROS_ERROR_STREAM("sliding_mapper: Extrapolation Exception. stamp = "
                     << stamp << " now = " << ros::Time::now()
                     << " delta = " << ros::Time::now() - stamp << endl
                     << e.what());
    return;
  }
  PM::TransformationParameters TSensorToBase = TBaseToSensor.inverse();

  // Apply filters to incoming cloud, in scanner coordinates
  inputFilters_.apply(*newPointCloud);

  // Ensure a minimum amount of point after filtering
  const int ptsCount = newPointCloud->features.cols();
  if (ptsCount < min_reading_point_cnt_)
  {
    ROS_ERROR_STREAM("sliding_mapper: not enough points after input filters");
    ROS_ERROR_STREAM("sliding_mapper: only " << ptsCount << " pts");
    return;
  }

  // add descriptors
  if (newPointCloud->descriptorExists("probabilityStatic") == false)
    newPointCloud->addDescriptor(
        "probabilityStatic",
        PM::Matrix::Constant(1, newPointCloud->features.cols(), prior_static_));
  if (newPointCloud->descriptorExists("probabilityDynamic") == false)
    newPointCloud->addDescriptor(
        "probabilityDynamic",
        PM::Matrix::Constant(1, newPointCloud->features.cols(), prior_dyn_));
  if (newPointCloud->descriptorExists("dynamic_ratio") == false)
    newPointCloud->addDescriptor(
        "dynamic_ratio", PM::Matrix::Zero(1, newPointCloud->features.cols()));
  if (newPointCloud->descriptorExists("observedPoseMark") == false)
    newPointCloud->addDescriptor(
        "observedPoseMark",
        PM::Matrix::Constant(1, newPointCloud->features.cols(), 0));

  transform::Rigid3f base2odom;
  {
#ifdef SLIDING_MAPPER_MULTI_THREAD
    std::unique_lock<std::mutex> lck(extrapolator_mtx_);
#endif
    base2odom = extrapolator_->ExtrapolatePose(time).cast<float>();
  }

  //
  base2odom = newestPose_.cast<float>();
  //base2odom = transform::Rigid3f(newestPose_.cast<float>().translation(), base2odom.rotation());
//  LOG(INFO) << "base2odom:";
//  LOG(INFO) << base2odom;
  //

  PM::TransformationParameters TBaseToOdomReckon =
      PM::TransformationParameters::Identity(dimp1, dimp1);
  TBaseToOdomReckon.block(0, 0, 3, 3) = base2odom.rotation().toRotationMatrix();
  TBaseToOdomReckon(0, 3) = base2odom.translation()[0];
  TBaseToOdomReckon(1, 3) = base2odom.translation()[1];
  TBaseToOdomReckon(2, 3) = base2odom.translation()[2];

  PM::TransformationParameters TSensorToLocal, TBaseToLocal;

  // Initialize the transformation to identity if empty
  if (!icp_.hasMap())
  {
    // transformation initialization

    // odom transformation
    // initially map = base
    lastTBaseToLocal_ = TBaseToLocal = TBaseToOdomReckon;
    // sensor transformation
    TSensorToLocal = TSensorToBase * TBaseToLocal;

    // Initialize the map if empty
    setMap(updateMap(newPointCloud.release(), TSensorToLocal, false, false));
    // we must not delete newPointCloud because we just stored it in the
    // cloudInLocal_

    // point removal for experience based navigation
    TBaseToOrigin_ = PM::TransformationParameters::Identity(dimp1, dimp1);
    map_pts_origin_dists_.clear();

    ROS_INFO_STREAM("Initial map created!");
    return;
  }

  // Check dimension
  if (newPointCloud->features.rows() != icp_.getInternalMap().features.rows())
  {
    ROS_ERROR_STREAM("sliding_mapper: dimensionality missmatch");
    ROS_ERROR_STREAM("sliding_mapper: incoming cloud is"
                     << newPointCloud->features.rows() - 1 << " while map is "
                     << icp_.getInternalMap().features.rows() - 1);
    return;
  }

#ifdef TIME_TEST
  tEndTime = clock();
  fCostTime = (double)(tEndTime - tBeginTime) / CLOCKS_PER_SEC;
  ROS_INFO("Get tf, apply filter, add descriptor and transform:%lf ms\n",
           fCostTime * 1000);

  tBeginTime = clock();
#endif
  // clock_t tBeginTime = clock();
  try
  {
    // current sensor to map
    PM::TransformationParameters TSensorToMapReckon =
        TBaseToOdomReckon * TSensorToBase;

    Eigen::Matrix3f SensorToMapRotation = TSensorToMapReckon.block(0, 0, 3, 3);
    Eigen::Quaternionf SensorToMapQuat =
        Eigen::Quaternionf(SensorToMapRotation);
    SensorToMapQuat.normalize();
    TSensorToMapReckon.block(0, 0, 3, 3) = SensorToMapQuat.toRotationMatrix();

    // newPointCloud is initially in current sensor, then these cloud is
    // transformed
    // into map
    // got new sensor to map!
    TSensorToLocal = icp_(*newPointCloud, TSensorToMapReckon);
    // TSensorToMap = TSensorToMapReckon;

    //     PM::TransformationParameters icp_delta;
    //    icp_delta = TSensorToMap.inverse()*TSensorToMapReckon;

    //    if(icp_delta(0, 0) < 0.999 || icp_delta(1, 1) < 0.999 ||
    //       icp_delta(2, 2) < 0.999 || icp_delta(0, 3) > 0.1 ||
    //       icp_delta(1, 3) > 0.1 || icp_delta(2, 3) > 0.1)
    //     ROS_INFO_STREAM("icp_delta:\n" << icp_delta);

    ROS_DEBUG_STREAM("Ticp:\n" << TSensorToLocal);

    // Ensure minimum overlap between scans
    const double estimatedOverlap = icp_.errorMinimizer->getOverlap();
    // const double estimatedOverlap = 1.0;

    // ROS_INFO_STREAM("Overlap: " << estimatedOverlap);
    if (estimatedOverlap < min_overlap_)
    {
      ROS_ERROR_STREAM("sliding_mapper: estimated overlap too small, ignoring "
                       "ICP correction");
      return;
    }

//    if (icp_info_pub_.getNumSubscribers())
//    {
//      vtr_msgs::IcpInfo icpInfo;
//      icpInfo.header.frame_id = odom_frame_;
//      icpInfo.header.stamp = stamp;
//      icpInfo.referenceCloud = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
//          *cloudInLocal_, odom_frame_, stamp);
//      icpInfo.sensorCloud = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
//          *newPointCloud, scannerFrame, stamp);

//      Eigen::Affine3d sensor2mapReckon_eigen;
//      for (int i = 0; i < 4; i++)
//        for (int j = 0; j < 4; j++)
//          sensor2mapReckon_eigen(i, j) = TSensorToMapReckon(i, j);
//      tf::poseEigenToMsg(sensor2mapReckon_eigen, icpInfo.sensor2reference_init);

//      Eigen::Affine3d sensor2local_eigen;
//      for (int i = 0; i < 4; i++)
//        for (int j = 0; j < 4; j++)
//          sensor2local_eigen(i, j) = TSensorToLocal(i, j);
//      tf::poseEigenToMsg(sensor2local_eigen, icpInfo.sensor2reference_matched);

//      icp_info_pub_.publish(icpInfo);
//    }

    TBaseToLocal = TSensorToLocal * TBaseToSensor;

    Eigen::Matrix3f BaseToOdomRotation = TBaseToLocal.block(0, 0, 3, 3);
    Eigen::Vector3f BaseToOdomTranslation = TBaseToLocal.block(0, 3, 3, 1);
    Eigen::Quaterniond q =
        Eigen::Quaterniond(BaseToOdomRotation.cast<double>());
    const transform::Rigid3d pose_estimate =
        transform::Rigid3d(BaseToOdomTranslation.cast<double>(), q);
    {
#ifdef SLIDING_MAPPER_MULTI_THREAD
      std::unique_lock<std::mutex> lck(extrapolator_mtx_);
#endif
      extrapolator_->AddPose(time, pose_estimate);
    }

    {
#ifdef SLIDING_MAPPER_MULTI_THREAD
      std::unique_lock<std::mutex> lck(extrapolator_pub_mtx_);
#endif
      newestPoseTime_ = time;
      newestPose_ = pose_estimate;
      newestPoseValid_ = true;
    }

    // cout<<"start to estimate plane footprint"<<endl;
    // got current base to last base!
    PM::TransformationParameters TBaseRelative =
        lastTBaseToLocal_.inverse() * TBaseToLocal;

    // for pts removal
    TBaseToOrigin_ = TBaseToOrigin_ * TBaseRelative;
    Eigen::Vector3f translationToOrigin = TBaseToOrigin_.block(0, 3, 3, 1);
    double translation_distance = translationToOrigin.norm();
    // if base translation is far enough and not in map building progress, then
    // we calculate the accumulative translation and find markerProcess
    if (translation_distance > map_pts_origin_interval_ &&
        !map_building_in_progress_)
    {
      markerProcess =
          0; // markerProcess >=0 means there is new PtsOrigin generated

      map_pts_origin_dists_.insert(map_pts_origin_dists_.begin(),
                                   translation_distance);
      for (int i = 1; i < map_pts_origin_dists_.size(); i++)
        map_pts_origin_dists_[i] += translation_distance;

      // FIXME:seems only need to compare with the last one
      for (int i = 1; i < map_pts_origin_dists_.size(); i++)
      {
        if (map_pts_origin_dists_[i] > map_pts_origin_max_dist_)
          markerProcess = i;
      }

      while (map_pts_origin_dists_.size() > max_valid_origin_)
        map_pts_origin_dists_.pop_back();

      // start a new origin of point clouds
      TBaseToOrigin_ = PM::TransformationParameters::Identity(dimp1, dimp1);
    }
    else
      markerProcess =
          -1; // markerProcess < 0 means there is NO new PtsOrigin generated

    if (new_cloud_pub_.getNumSubscribers())
    {
      DP* cloudInLocal =
          new DP(newPointCloud->features, newPointCloud->featureLabels,
                 newPointCloud->descriptors, newPointCloud->descriptorLabels);

      *cloudInLocal = transformation_->compute(*newPointCloud, TSensorToLocal);

      sensor_msgs::PointCloud2 newCloud;
      newCloud = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
          *cloudInLocal, odom_frame_, stamp);
      newCloud.header.frame_id = odom_frame_;

      new_cloud_pub_.publish(newCloud);
    }

    // check if news points should be added to the map
    if (((estimatedOverlap < max_overlap_to_merge_) ||
         (icp_.getInternalMap().features.cols() < min_map_point_cnt_)) &&
#if BOOST_VERSION >= 104100
        (!map_building_in_progress_)
#else  // BOOST_VERSION >= 104100
        true
#endif // BOOST_VERSION >= 104100
            )
    {
// make sure we process the last available map
#if BOOST_VERSION >= 104100
      // ROS_INFO("Adding new points to the map in background");
      map_building_task_ = MapBuildingTask(
          boost::bind(&pointMatcherMapping::updateMap, this, newPointCloud.release(),
                      TSensorToLocal, markerProcess, true));
      map_building_future_ = map_building_task_.get_future();
      map_building_thread_ =
          boost::thread(boost::move(boost::ref(map_building_task_)));
      map_building_in_progress_ = true;
#else  // BOOST_VERSION >= 104100
      ROS_INFO("sliding_mapper: adding new points to the map");
      setMap(updateMap(newPointCloud.release(), TSensorToMap, markerProcess,
                       true));
#endif // BOOST_VERSION >=
    }
  }
  catch (PM::ConvergenceError error)
  {
    ROS_ERROR_STREAM("sliding_mapper: ICP failed to converge");
    ROS_ERROR_STREAM("sliding_mapper: " << error.what());
    return;
  }

  if (submap_cloud_pub_.getNumSubscribers())
  {
    sensor_msgs::PointCloud2 subCloud;
    subCloud = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
        *cloudInLocal_, odom_frame_, stamp);
    subCloud.header.frame_id = odom_frame_;

    submap_cloud_pub_.publish(subCloud);
  }

  if (submap_pub_.getNumSubscribers())
  {
    PM::TransformationParameters TBaseToLastPubBase =
        TLocalToLastPubBase_ * TBaseToLocal;
    Eigen::Vector3f t = TBaseToLastPubBase.block(0, 3, 3, 1);
    Eigen::Matrix3f rot = TBaseToLastPubBase.block(0, 0, 3, 3);
    Eigen::Quaternionf q =
        Eigen::Quaternionf(rot);

    if ((t.norm() + std::acos(q.w()) * 4) > 5.f)
    {
      DP* cloudInBase =
          new DP(cloudInLocal_->features, cloudInLocal_->featureLabels,
                 cloudInLocal_->descriptors, cloudInLocal_->descriptorLabels);

      *cloudInBase =
          transformation_->compute(*cloudInLocal_, TBaseToLocal.inverse());
      vtr_msgs::SubCloud submap;
      submap.header.frame_id = base_frame_;
      submap.header.stamp = stamp;
      submap.cloud = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
          *cloudInBase, scannerFrame, stamp);
      submap.cloud.header.frame_id = base_frame_;

      //    submap_cloud_pub_.publish(submap.cloud);

      Eigen::Affine3d base2odom_eigen;
      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
          base2odom_eigen(i, j) = TBaseToLocal(i, j);

      tf::poseEigenToMsg(base2odom_eigen, submap.pose);
      submap_pub_.publish(submap);
      delete cloudInBase;

      TLocalToLastPubBase_ = TBaseToLocal.inverse();
    }
  }

  lastTBaseToLocal_ = TBaseToLocal;
}

#include <tf_conversions/tf_eigen.h>

void pointMatcherMapping::processNewMapIfAvailable()
{
// if mapBuilding task finished,setMap
#if BOOST_VERSION >= 104100
  if (map_building_in_progress_ && map_building_future_.has_value())
  {
    //      ROS_INFO_STREAM("New map available");
    setMap(map_building_future_.get());
    map_building_in_progress_ = false;
  }
#endif // BOOST_VERSION >= 104100
}

void pointMatcherMapping::setMap(DP* newPointCloud)
{
  // delete old map
  if (cloudInLocal_)
    delete cloudInLocal_;

  // set new map
  cloudInLocal_ = newPointCloud;
  icp_.setMap(*cloudInLocal_);
}

pointMatcherMapping::DP* pointMatcherMapping::updateMap(DP* newPointCloud,
                              const PM::TransformationParameters TSensorToLocal,
                              int markerProcess, bool updateExisting)
{

  if (!updateExisting)
  {
    // FIXME: correct that, ugly
    cout << "Initial map creating..." << endl;
    *newPointCloud = transformation_->compute(*newPointCloud, TSensorToLocal);
    mapPostFilters_.apply(*newPointCloud);
    return newPointCloud;
  }

  DP* cloudInSensor =
      new DP(cloudInLocal_->features, cloudInLocal_->featureLabels,
             cloudInLocal_->descriptors, cloudInLocal_->descriptorLabels);
  *cloudInSensor =
      transformation_->compute(*cloudInLocal_, TSensorToLocal.inverse());

  dynamicProbabilityUpdate(newPointCloud, cloudInSensor);

  const int readPtsCount(newPointCloud->features.cols());
  const int mapPtsCount(cloudInLocal_->features.cols());

  int newMapCount = 0;
  // // update map points marker
  DP::View viewOnMapMark =
      cloudInSensor->getDescriptorViewByName("observedPoseMark");
  if (markerProcess >= 0)
  {
    for (int i = 0; i < mapPtsCount; i++)
      viewOnMapMark(0, i)++;
    cloudInSensor->addDescriptor("observedPoseMark", viewOnMapMark);
  }
  // if marker process >=0, which means new origin/pose generated, update
  // markers of the points in the map

  // delete map points which are too old
  if (markerProcess > 0)
  {
    DP* cloudInSensorCopy =
        new DP(cloudInSensor->features, cloudInSensor->featureLabels,
               cloudInSensor->descriptors, cloudInSensor->descriptorLabels);
    for (int i = 0; i < mapPtsCount; i++)
    {
      if (viewOnMapMark(0, i) < markerProcess) // delete the points that has
                                               // marker bigger than
                                               // markerProcess
      {
        cloudInSensor->setColFrom(newMapCount, *cloudInSensorCopy, i);
        newMapCount++;
      }
    }
    cloudInSensor->conservativeResize(newMapCount);
    delete cloudInSensorCopy;
  }

  const int trimmedMapPtsCount(cloudInSensor->features.cols());

  // Generate temporary map for density computation
  DP* tmp_map =
      new DP(cloudInSensor->features, cloudInSensor->featureLabels,
             cloudInSensor->descriptors, cloudInSensor->descriptorLabels);
  tmp_map->concatenate(*newPointCloud); // this method can decrease the density
                                        // of newPointCloud!!!!
  // build and populate NNS
  std::shared_ptr<NNS> featureNNS;
  featureNNS.reset(NNS::create(tmp_map->features, tmp_map->features.rows() - 1,
                               NNS::KDTREE_LINEAR_HEAP));
  PM::Matches matches_overlap(Matches::Dists(1, readPtsCount),
                              Matches::Ids(1, readPtsCount));
  featureNNS->knn(newPointCloud->features, matches_overlap.ids,
                  matches_overlap.dists, 1, 0);

  DP::View viewOnTrimmedMapMark =
      cloudInSensor->getDescriptorViewByName("observedPoseMark");
  DP no_overlap(newPointCloud->createSimilarEmpty());
  int ptsOut = 0;
  for (int i = 0; i < readPtsCount; ++i)
  {
    if (matches_overlap.dists(i) > max_dist_new_point_)
    {
      no_overlap.setColFrom(ptsOut, *newPointCloud, i);
      ptsOut++;
    }
    else
    {
      //std::cout << matches_overlap.dists(i) << " <= " << max_dist_new_point_ << std::endl;

      if (matches_overlap.ids(i) < trimmedMapPtsCount &&
             matches_overlap.dists(i) <
                 0.01) // map is constructed by mapPoints and newPoints
      viewOnTrimmedMapMark(0, matches_overlap.ids(i)) =
          0; // the associated points has been re-observed and mark as 0
    }
  }

  // update map points pose marker
  cloudInSensor->addDescriptor("observedPoseMark", viewOnTrimmedMapMark);

  // shrink the newPointCloud to the new information
  no_overlap.conservativeResize(ptsOut);
  *newPointCloud = no_overlap;

//  std::cout << "readPtsCount: " << readPtsCount << std::endl;
//  std::cout << "ptsOut: " << ptsOut << std::endl;
//  {
//    sensor_msgs::PointCloud2 subCloud;
//    subCloud = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
//        *newPointCloud, "velodyne", ros::Time::now());
//    subCloud.header.frame_id = "velodyne";

//    submap_cloud_pub_.publish(subCloud);
//  }

  // Merge point clouds to map
  newPointCloud->concatenate(*cloudInSensor);

  mapPostFilters_.apply(*newPointCloud);
  *newPointCloud = transformation_->compute(*newPointCloud, TSensorToLocal);

  delete tmp_map;
  delete cloudInSensor;
  // return mapPoint + newPont_noOverlap
  return newPointCloud;
}

void pointMatcherMapping::dynamicProbabilityUpdate(DP* newPointCloud, DP* cloudInSensor)
{
  const int newPointsCount(newPointCloud->features.cols());
  const int mapPointsCount(cloudInSensor->features.cols());

  // Build a range image of the reading point cloud (Senor Frame)
  PM::Matrix radius_newPts =
      newPointCloud->features.topRows(3).colwise().norm();
  PM::Matrix angles_newPts(2, newPointsCount); // 0=inclination, 1=azimuth
  // initializeing angular kd tree of new point cloud
  for (int i = 0; i < newPointsCount; i++)
  {
    const float ratio = newPointCloud->features(2, i) / radius_newPts(0, i);
    angles_newPts(0, i) = acos(ratio);
    angles_newPts(1, i) =
        atan2(newPointCloud->features(1, i), newPointCloud->features(0, i));
  }
  std::shared_ptr<NNS> featureNNS;
  featureNNS.reset(NNS::create(angles_newPts));

  PM::Matrix radius_mapPts =
      cloudInSensor->features.topRows(3).colwise().norm();
  PM::Matrix angles_mapPts(2, mapPointsCount); // 0=inclination, 1=azimuth
  // No atan in Eigen, so we are for to loop through it...
  for (int i = 0; i < mapPointsCount; i++)
  {
    const float ratio = cloudInSensor->features(2, i) / radius_mapPts(0, i);
    angles_mapPts(0, i) = acos(ratio);
    angles_mapPts(1, i) =
        atan2(cloudInSensor->features(1, i), cloudInSensor->features(0, i));
  }
  // Look for NN in spherical coordinates
  Matches::Dists dists(1, mapPointsCount);
  Matches::Ids ids(1, mapPointsCount);

  featureNNS->knn(angles_mapPts, ids, dists, 1, 0, NNS::ALLOW_SELF_MATCH,
                  max_angle_);

  DP::View viewOnnormals_Map =
      cloudInSensor->getDescriptorViewByName("normals");
  DP::View viewOnProbabilityStatic =
      cloudInSensor->getDescriptorViewByName("probabilityStatic");
  DP::View viewOnProbabilityDynamic =
      cloudInSensor->getDescriptorViewByName("probabilityDynamic");
  DP::View viewOnDynamicRatio =
      cloudInSensor->getDescriptorViewByName("dynamic_ratio");

  for (int i = 0; i < mapPointsCount; i++)
  {
    if (dists(i) != numeric_limits<float>::infinity())
    {
      // in local coordinates
      const Eigen::Vector3f newPt =
          newPointCloud->features.col(ids(0, i)).head(3);
      const Eigen::Vector3f mapPt = cloudInSensor->features.col(i).head(3);
      const Eigen::Vector3f mapPt_n = mapPt.normalized();
      const float delta = (newPt - mapPt).norm();
      const float d_max = eps_a_ * newPt.norm();

      const Eigen::Vector3f normal_map = viewOnnormals_Map.col(i);

      // Weight for dynamic elements
      const float w_v = eps_ + (1 - eps_) * fabs(normal_map.dot(mapPt_n));
      // const float w_d1 = 1 + eps -
      // acos(newPt.normalized().dot(mapPt_n))/max_angle_;
      const float w_d1 = eps_ + (1 - eps_) * (sqrt(dists(i)) / max_angle_);

      const float offset = delta - eps_d_;
      float w_d2 = 1;
      if (delta < eps_d_ || mapPt.norm() > newPt.norm())
      {
        w_d2 = eps_;
      }
      else
      {
        if (offset < d_max)
        {
          w_d2 = eps_ + (1 - eps_) * offset / d_max;
        }
      }

      float w_p2 = eps_;
      if (delta < eps_d_)
      {
        w_p2 = 1;
      }
      else
      {
        if (offset < d_max)
        {
          w_p2 = eps_ + (1 - eps_) * (1 - offset / d_max);
        }
      }

      // We don't update point behind the reading
      if ((newPt.norm() + eps_d_ + d_max) >= mapPt.norm())
      {

        const float lastDyn = viewOnProbabilityDynamic(0, i);
        const float lastStatic = viewOnProbabilityStatic(0, i);

        const float c1 = (1 - (w_v * (1 - w_d1)));
        const float c2 = w_v * (1 - w_d1);

        // FIXME: this is a parameter
        // const float maxDyn = 0.9; // ICRA 14
        //        const float maxDyn = 0.98; // ISER 14

        // Lock dynamic point to stay dynamic under a threshold
        if (lastDyn < max_dyn_)
        {
          viewOnProbabilityDynamic(0, i) =
              c1 * lastDyn +
              c2 * w_d2 * ((1 - alpha_) * lastStatic + beta_ * lastDyn);
          viewOnProbabilityStatic(0, i) =
              c1 * lastStatic +
              c2 * w_p2 * (alpha_ * lastStatic + (1 - beta_) * lastDyn);
        }
        else
        {
          viewOnProbabilityStatic(0, i) = eps_;
          viewOnProbabilityDynamic(0, i) = 1 - eps_;
        }

        // normalization
        const float sumZ =
            viewOnProbabilityDynamic(0, i) + viewOnProbabilityStatic(0, i);
        assert(sumZ >= eps_);

        viewOnProbabilityDynamic(0, i) /= sumZ;
        viewOnProbabilityStatic(0, i) /= sumZ;

        // viewOnDynamicRatio(0,mapId) =viewOnProbabilityDynamic(0, mapId);
        viewOnDynamicRatio(0, i) = w_d2;
      }
    }
  }

  cloudInLocal_->addDescriptor("probabilityDynamic", viewOnProbabilityDynamic);
  cloudInLocal_->addDescriptor("probabilityStatic", viewOnProbabilityStatic);
  cloudInLocal_->addDescriptor("dynamic_ratio", viewOnDynamicRatio);
}
// void pointMatcherMapping::waitForMapBuildingCompleted()
//{
//#if BOOST_VERSION >= 104100
//  if (map_building_in_progress_)
//  {
//    // we wait for now, in future we should kill it
//    map_building_future_.wait();
//    map_building_in_progress_ = false;
//  }
//#endif // BOOST_VERSION >= 104100
//}

// bool added = false;

void pointMatcherMapping::publishTf(const ::ros::WallTimerEvent& timer_event)
{
#ifdef SLIDING_MAPPER_MULTI_THREAD
  std::unique_lock<std::mutex> lck(extrapolator_pub_mtx_);
#endif

  if (!newestPoseValid_)
    return;

  if (newestPoseTime_ != extrapolator_pub_->GetLastPoseTime())
  {
    //    if(!added)
    //    {
    extrapolator_pub_->AddPose(newestPoseTime_, newestPose_);
    //    added = true;
    //    }
  }

  geometry_msgs::TransformStamped stamped_transform;
  // If we do not publish a new point cloud, we still allow time of the
  // published poses to advance. If we already know a newer pose, we use its
  // time instead. Since tf knows how to interpolate, providing newer
  // information is better.

  //  LOG(WARNING) << "now:" << ros::Time::now();
  //  LOG(WARNING) << "LastExtrapolatedTime:"<<
  //  extrapolator_pub_->GetLastExtrapolatedTime();

  const ::cartographer::common::Time now = std::max(
      FromRos(ros::Time::now()), extrapolator_pub_->GetLastExtrapolatedTime());
  stamped_transform.header.stamp = ToRos(now);
  const transform::Rigid3d tracking_to_local =
      extrapolator_pub_->ExtrapolatePose(now);

  std::vector<geometry_msgs::TransformStamped> stamped_transforms;
  stamped_transform.header.frame_id = odom_frame_;
  stamped_transform.child_frame_id = base_frame_;
  stamped_transform.transform = ToGeometryMsgTransform(tracking_to_local);
  stamped_transforms.push_back(stamped_transform);

  // 1. local_to_map
  // 2. baselink_to_local
  tf_broadcaster_.sendTransform(stamped_transforms);

  if (lidar_odometry_pub_.getNumSubscribers())
  {
    nav_msgs::Odometry odometry_msgs;

    odometry_msgs.pose.pose = ToGeometryMsgPose(tracking_to_local);
    odometry_msgs.header = stamped_transform.header;
    odometry_msgs.child_frame_id = "lidar_odometry";

    lidar_odometry_pub_.publish(odometry_msgs);
  }
}

void pointMatcherMapping::addCloudMsgToQueue(const sensor_msgs::PointCloud2ConstPtr& cloudMsgIn)
{
  cloud_lock_.lock();
  cloud_msgs_.push(cloudMsgIn);
  cloud_lock_.unlock();
}

void pointMatcherMapping::addImuMsgToQueue(const sensor_msgs::ImuConstPtr& imu_msg)
{
  imu_lock_.lock();
  imu_msgs_.push(imu_msg);
  imu_lock_.unlock();
}

void pointMatcherMapping::addOdomMsgToQueue(const nav_msgs::OdometryConstPtr& msg)
{
  odom_lock_.lock();
  odom_msgs_.push(msg);
  odom_lock_.unlock();
}

void pointMatcherMapping::computeLoop()
{
  ros::Rate rate(1000);
  ros::Time last_imu_time,last_odom_time,last_cloud_time;
  while(!close_flag_ || !cloud_msgs_.empty())
  {
//     if(imu_msgs_.empty() && odom_msgs_.empty() && cloud_msgs_.empty())
      
    while(!imu_msgs_.empty())
    {
      
      if(imu_msgs_.front()->header.stamp <= last_imu_time)
      {
        imu_lock_.lock();
        imu_msgs_.pop();
        imu_lock_.unlock();
        continue;
      }
      last_imu_time = imu_msgs_.front()->header.stamp;
//       if(!cloud_msgs_.empty() && !imu_msgs_.empty())
//       {
//         if(imu_msgs_.front()->header.stamp > cloud_msgs_.front()->header.stamp )
//         {
//           break;
//         }
//       }
      addImuMsg(imu_msgs_.front());
      imu_lock_.lock();
      imu_msgs_.pop();
      imu_lock_.unlock();
    }
    
    while(!odom_msgs_.empty())
    {
      
      if(odom_msgs_.front()->header.stamp <= last_odom_time)
      {
        odom_lock_.lock();
        odom_msgs_.pop();
        odom_lock_.unlock();
        continue;
      }
      last_odom_time = odom_msgs_.front()->header.stamp;
//       if(!cloud_msgs_.empty() && !odom_msgs_.empty())
//       {
//         if(odom_msgs_.front()->header.stamp > cloud_msgs_.front()->header.stamp )
//         {
//           break;
//         }
//       }
      
      addOdomMsg(odom_msgs_.front());
      odom_lock_.lock();
      odom_msgs_.pop();
      odom_lock_.unlock();
    }
    
    while(!cloud_msgs_.empty())
    {
      if(cloud_msgs_.front()->header.stamp <= last_cloud_time)
      {
        cloud_lock_.lock();
        cloud_msgs_.pop();
        cloud_lock_.unlock();
        continue;
      }
      last_cloud_time = cloud_msgs_.front()->header.stamp;
      if(!cloud_msgs_.empty())
      {
        if(!odom_msgs_.empty() && cloud_msgs_.front()->header.stamp > odom_msgs_.front()->header.stamp)
        {
          break;
        }
        if(!imu_msgs_.empty() && cloud_msgs_.front()->header.stamp > imu_msgs_.front()->header.stamp)
        {
          break;
        }

      }
      
      addCloudMsg(*(cloud_msgs_.front()));
      cloud_lock_.lock();
      cloud_msgs_.pop();
      cloud_lock_.unlock();
    }
    rate.sleep();
  }
}

const sensor_msgs::PointCloud2 pointMatcherMapping::getMap()
{
  ros::Rate rate(100);
  close_flag_ = true;
  while(!cloud_msgs_.empty())
  {
    rate.sleep();
  }
//   while(!close_flag_)
//   {
//     rate.sleep();
//   }
  sensor_msgs::PointCloud2 subCloud;
  subCloud = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
    *cloudInLocal_, odom_frame_, ros::Time::now());
  subCloud.header.frame_id = odom_frame_;

  return subCloud;
}

}
// Main function supporting the pointMatcherMapping class
unique_ptr<point_matcher_mapping::pointMatcherMapping> mapper_;
ros::Subscriber imu_sub_;
ros::Subscriber odom_sub_;
ros::Subscriber cloud_sub_;

void handleImu(const sensor_msgs::ImuConstPtr& msg)
{
  mapper_->addImuMsgToQueue(msg);
//   mapper_->addImuMsg(msg);
}

void handleOdom(const nav_msgs::OdometryConstPtr& msg)
{
  mapper_->addOdomMsgToQueue(msg);
//   mapper_->addOdomMsg(msg);
}

void handleCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  mapper_->addCloudMsgToQueue(msg);
//     mapper_->addCloudMsg(*msg);

}

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "mapper");
  ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  ros::NodeHandle nh;
  if (getParam<bool>("subscribe_imu", true))
    imu_sub_ = nh.subscribe("imu_in", 10, handleImu);
  if (getParam<bool>("subscribe_odom", true))
    odom_sub_ = nh.subscribe("odom_in", 10, handleOdom);
  if (getParam<bool>("subscribe_cloud", true))
    cloud_sub_ = nh.subscribe("cloud_in", 3, handleCloud);


#ifndef NDEBUG
  sleep(3);
#endif
  unique_ptr<point_matcher_mapping::pointMatcherMapping> temp(new point_matcher_mapping::pointMatcherMapping(nh));
  mapper_ = std::move(temp);

#ifdef SLIDING_MAPPER_MULTI_THREAD
  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin();
#else
  ros::spin();
#endif

  return 0;
}
