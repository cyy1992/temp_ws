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

#ifndef POINTMATCHERMAPPING_H
#define POINTMATCHERMAPPING_H
#include <fstream>
#include <mutex>

#include <boost/version.hpp>
#include <boost/thread.hpp>
#if BOOST_VERSION >= 104100
#include <boost/thread/future.hpp>
#endif // BOOST_VERSION >=  104100
#include <thread>
#include <queue>
#include <Eigen/Dense>

#include "ros/ros.h"
#include <ros/package.h>
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"

#include "pointmatcher/Functions.h"
#include "pointmatcher/Timer.h"

#include "nabo/nabo.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "eigen_conversions/eigen_msg.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

// Services
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "map_msgs/SaveMap.h"
#include "cartographer/mapping/pose_extrapolator.h"

#define SLIDING_MAPPER_MULTI_THREAD
namespace point_matcher_mapping{
class pointMatcherMapping
{
public:
  pointMatcherMapping(ros::NodeHandle& nh);
  ~pointMatcherMapping();
  void addImuMsgToQueue(const sensor_msgs::ImuConstPtr& imu_msg);
  void addOdomMsgToQueue(const nav_msgs::OdometryConstPtr& msg);
  void addCloudMsgToQueue(const sensor_msgs::PointCloud2ConstPtr& cloudMsgIn);
  
  void addImuMsg(const sensor_msgs::ImuConstPtr& imu_msg);
  void addOdomMsg(const nav_msgs::OdometryConstPtr& msg);
  void addCloudMsg(const sensor_msgs::PointCloud2& cloudMsgIn);

  const sensor_msgs::PointCloud2  getMap();
private:
  typedef PointMatcher<float> PM;
  typedef PM::DataPoints DP;
  typedef PM::Matches Matches;

  typedef typename Nabo::NearestNeighbourSearch<float> NNS;
  typedef typename NNS::SearchType NNSearchType;

  void InitializeExtrapolator(const cartographer::common::Time time);
  void publishTf(const ::ros::WallTimerEvent& timer_event);
  
  void addImuData(const cartographer::sensor::ImuData& imu_data);
  void addOdom(const cartographer::sensor::OdometryData& odometry_data);
  void addCloudData(std::unique_ptr<DP> cloud, const std::string& scannerFrame,
                    const ros::Time& stamp, uint32_t seq);
  void processNewMapIfAvailable();
  void setMap(DP* newPointCloud);
  DP* updateMap(DP* newPointCloud, const PM::TransformationParameters Ticp,
                int markerProcess, bool updateExisting);
  //void waitForMapBuildingCompleted();
  void dynamicProbabilityUpdate(DP* newPointCloud, DP* mapPointCloudInSensor);

  void computeLoop();
  ros::NodeHandle& nh_;
//   ros::Subscriber imu_sub_;
//   ros::Subscriber odom_sub_;
//   ros::Subscriber cloud_sub_;
  ros::Publisher submap_pub_;
  ros::Publisher submap_cloud_pub_;
  ros::Publisher new_cloud_pub_;
  ros::Publisher lidar_odometry_pub_;
  //ros::Publisher icp_info_pub_;
  ros::WallTimer tfPub_timer_;
  tf2_ros::Buffer tfBuffer_;                                                  \
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

#ifdef SLIDING_MAPPER_MULTI_THREAD
  std::mutex extrapolator_mtx_;
  std::mutex extrapolator_pub_mtx_;
#endif
  std::unique_ptr<cartographer::mapping::PoseExtrapolator> extrapolator_;
  std::unique_ptr<cartographer::mapping::PoseExtrapolator> extrapolator_pub_;
  bool use_imu_data_;

  // libpointmatcher for velodyne cloud
  PM::ICPSequence icp_;
  std::unique_ptr<PM::Transformation> transformation_;
  PM::DataPointsFilters inputFilters_;
  PM::DataPointsFilters mapPostFilters_;
  PM::DataPoints* cloudInLocal_;
  PM::TransformationParameters lastTBaseToLocal_;

  cartographer::common::Time newestPoseTime_;
  cartographer::transform::Rigid3d newestPose_;
  bool newestPoseValid_;

// multi-threading mapper
#if BOOST_VERSION >= 104100
  typedef boost::packaged_task<PM::DataPoints*> MapBuildingTask;
  typedef boost::unique_future<PM::DataPoints*> MapBuildingFuture;
  boost::thread map_building_thread_;
  MapBuildingTask map_building_task_;
  MapBuildingFuture map_building_future_;
  bool map_building_in_progress_;
#endif // BOOST_VERSION >= 104100

  const float max_dist_new_point_; //!< in meter. Distance at which a new point will
  // be added in the global map.
  const int min_reading_point_cnt_;
  const int min_map_point_cnt_;
  const double min_overlap_;
  const double max_overlap_to_merge_;
  const double tf_refresh_period_;
  const std::string base_frame_;
  const std::string odom_frame_;

  int max_valid_origin_;
  const double map_pts_origin_max_dist_;
  const double map_pts_origin_interval_;
  std::vector<double> map_pts_origin_dists_;
  PM::TransformationParameters TBaseToOrigin_;

  // Parameters for dynamic filtering
  const float
      prior_static_; //!< ratio. Prior to be static when a new point is added
  const float
      prior_dyn_; //!< ratio. Prior to be dynamic when a new point is added
  const float max_angle_; //!< in rad. Openning angle of a laser beam
  const float eps_a_;    //!< ratio. Error proportional to the laser distance
  const float eps_d_;    //!< in meter. Fix error on the laser distance
  const float alpha_;    //!< ratio. Propability of staying static given that the
  // point was dynamic
  const float beta_; //!< ratio. Propability of staying dynamic given that the
  // point was static
  const float max_dyn_; //!< ratio. Threshold for which a point will stay dynamic
  //const float dynT;   //!< in meter. Distance at which a new point will be added
  // in the global map.
  const float eps_;

  PM::TransformationParameters TLocalToLastPubBase_;
  
  std::queue<sensor_msgs::ImuConstPtr> imu_msgs_;
  std::queue<nav_msgs::OdometryConstPtr> odom_msgs_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_msgs_;
  bool close_flag_; 
  std::mutex imu_lock_,odom_lock_,cloud_lock_;
};
} //point_matcher_mapping



#endif // POINTMATCHERMAPPING_H
