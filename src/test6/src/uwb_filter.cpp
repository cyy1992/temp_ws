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

#include "uwb_filter.h"
#include "cartographer_ros_msgs/LinktrackNodeframe2.h"
using namespace cv;
using namespace std;

Kalman::Kalman()
{
  
  H << 1.0,0.0;
  A << 1.0,0.0,0.0,1.0;
  
  P_k << 1.0,0.0,0.0,1.0;
  Q_k << 1.0,0.0,0.0,1.0;
  Q_k = 1e-1 * Q_k;
  R_k = 10.0;
  x_k(0) = 0.0;
  x_k(1) = 0.0;

  initialised_ = false;
  unpub_cnt_ = 0;
  
}
double Kalman::Var()
{
  double sum = 0;
  for(auto it:distance_queue_)
  {
    sum += it;
  }
  double mean = sum/distance_queue_.size();
  double var = (distance_queue_.back() - mean) * (distance_queue_.back() - mean);
  return var;
}


double Kalman::Filter(const double& det_time, const double& distance)
{
  if(!initialised_)
  {
    
    if(!initial_distances_.empty() && initial_distances_.size() > 5)
    {
      cout << distance <<", " << *(initial_distances_.begin()) <<", "<< *(initial_distances_.rbegin()) <<endl;
      auto min_dis = std::min_element(initial_distances_.begin(), initial_distances_.end());
      auto max_dis = std::max_element(initial_distances_.begin(), initial_distances_.end());
      if(fabs(distance - *min_dis) < 1.0 && 
        fabs(distance - *max_dis) < 1.0)
      {
        initialised_ = true;
//         state_.at<float>(0) = distance;
//         state_.at<float>(1) = 0;
//         kf_.statePost = state_;
        last_distance_ = distance;
        x_k(0) = distance;
        unpub_cnt_ = 10;
        return distance;
      }
      else
      {
        initial_distances_.erase(initial_distances_.begin());
        initial_distances_.push_back(distance);
      }
    }
    else
      initial_distances_.push_back(distance);
    return -1.0;
  }
/*
  distance_queue_.push_back(distance);
  if(distance_queue_.size() > 200)
    distance_queue_.erase(distance_queue_.begin());
  double var = Var();

  if(var > 2.0){
    cout << var <<endl;
    return -2;
  }*/

  assert(det_time > 0);
  A(0,1) = det_time;
  Eigen::Vector2d x_ = A * x_k;
  Eigen::Matrix2d P_ = A * P_k * A.transpose() + Q_k;
  float tmp = fabs(x_(0) - distance);
  if(tmp > 0.1 * P_.trace())
  {
    x_k = x_;
    P_k = P_;
    unpub_cnt_ = 0;
    return -2;
  }
  else
  {
    K = P_*H * (1.0 / (P_(0,0) + R_k));
    x_k = x_ + K *(distance -x_(0) );
    Eigen::Matrix2d k_h ;
    k_h << K(0),0.,K(1),0.;
    P_k = (Eigen::Matrix2d::Identity() - k_h) * P_;
  }
  if(unpub_cnt_++ < 10)
    return -2;
  return x_k(0);
}

UwbFilter::UwbFilter(const ros::NodeHandle& n):nh_(n)
{
  uwb_sub_ = nh_.subscribe<nlink_parser::LinktrackNodeframe2>(
    "/nlink_linktrack_nodeframe2", 1, &UwbFilter::HandleUwbMsg,this);
  filter_uwb_pub_ = nh_.advertise<cartographer_ros_msgs::LinktrackNodeframe2>("/filter_uwb_datas",1);
  max_distance_ = 300;
}

UwbFilter::~UwbFilter()
{
  
}

void UwbFilter::HandleUwbMsg(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg)
{
  cartographer_ros_msgs::LinktrackNodeframe2 filtered_uwb_msg;
  ros::Time cur_time = msg->ros_time;
  filtered_uwb_msg.ros_time = cur_time;
//   cout << msg->nodes.size() <<endl;
  for(nlink_parser::LinktrackNode2 node: msg->nodes)
  {
    int id = node.id;
    float distance = node.dis;
    if(unuse_distance_count_.find(id) == unuse_distance_count_.end())
    {
      unuse_distance_count_[id] = 0;
      kalman_filters_[id].reset(new Kalman);
      origin_path_pub_[id] = nh_.advertise<geometry_msgs::Point>("origin_path_" + to_string(id), 1);
      filtered_path_pub_[id] = nh_.advertise<geometry_msgs::Point>("filtered_path_" + to_string(id), 1);
      last_times_[id] = cur_time;
      continue;
    }
    
    // 1. distance filter
//     if(distance > max_distance_)
//     {
//       unuse_distance_count_[id] +=1;
//       continue ;
//     }
//     else if(unuse_distance_count_[id] > 200)
//     {
//       kalman_filters_[id].reset(new Kalman);
//       unuse_distance_count_[id] = 0;
//     }
    
    
    // 2. kalman filter
    double det_time = (cur_time - last_times_[id]).toSec();
    double filtered_data = kalman_filters_[id]->Filter(det_time, distance);
    
    if(filtered_data >= 0.)
    {
      last_times_[id] = cur_time;
      
      cartographer_ros_msgs::LinktrackNode2 node2;
      node2.dis = static_cast<float>(filtered_data);
      node2.id = id;
      node2.fp_rssi = node.fp_rssi;
      node2.rx_rssi = node.rx_rssi;
      filtered_uwb_msg.nodes.push_back(node2);
      
      // visualization
      geometry_msgs::Point tmp_point;
      tmp_point.x = distance;
      origin_path_pub_[id].publish(tmp_point);
      
      tmp_point.x = filtered_data;
      filtered_path_pub_[id].publish(tmp_point); 
    }
    else if(filtered_data == -2)
    {
//       unuse_distance_count_[id] +=1;
    }
  }
  if(filtered_uwb_msg.nodes.size() > 0)
    filter_uwb_pub_.publish(filtered_uwb_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uwb_filter");
  ros::NodeHandle n;
  UwbFilter f(n);
  ros::spin();
  return 1;
}