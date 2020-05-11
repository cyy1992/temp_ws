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

#ifndef LASERMATCHER_H
#define LASERMATCHER_H
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


#include <tf/tf.h>
#include <math.h>
#include <iostream>
#include "Eigen/Eigen"
#include <tf/transform_broadcaster.h>
#include <csm/csm_all.h>
#undef min
#undef max

using  namespace std;
class LaserMatcher
{
public:
  LaserMatcher();
  ~LaserMatcher();
  void match(sensor_msgs::LaserScan scan1_msg, 
             sensor_msgs::LaserScan scan2_msg,
             tf::Transform &f2l);
  
private:
  void initParams();
  void laserScanToLDP(const sensor_msgs::LaserScan& scan_msg,
                            LDP& ldp);
  bool computePose(const LDP &ldp1,const LDP &ldp2,tf::Transform &f2l);
  void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t);
  
  sm_params input_;
  sm_result output_;
  double pr_ch_x_, pr_ch_y_, pr_ch_a_;
  
  ros::NodeHandle nh_;
  ros::Publisher scan1_pub_,scan2_pub_;
  tf::TransformBroadcaster broadcaster_;
};

#endif // LASERMATCHER_H
