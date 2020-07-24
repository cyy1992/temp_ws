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

#include "adjust_image_frequency.h"
using namespace std;
AdjustImageFrequency::AdjustImageFrequency(const ros::NodeHandle& nh, const float& frq):nh_(nh),interval_(1.0/frq)
{
  last_pub_msg_time_ = ros::Time::now();
  img_pub_ = nh_.advertise<sensor_msgs::Image>("/front/color/lower_frq_image_raw", 10, true);
  
  img_sub_ = nh_.subscribe("/front/color/image_raw", 1, &AdjustImageFrequency::imgCb, this);
  
}

AdjustImageFrequency::~AdjustImageFrequency()
{
  
}

void AdjustImageFrequency::imgCb(const sensor_msgs::ImageConstPtr& img_msg)
{
  ros::Duration tmp = img_msg->header.stamp - last_pub_msg_time_;
  
  if(tmp.toSec() > interval_)
  {
    last_pub_msg_time_ = img_msg->header.stamp;
    img_pub_.publish(img_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "adjust_image_frequency_node");
  ros::NodeHandle nh;
  string temp_frq = argv[1];
  AdjustImageFrequency btt(nh,std::atof(temp_frq.c_str()));
  ROS_INFO("AdjustImageFrequency start!");

  ros::spin();

  return 0;
}