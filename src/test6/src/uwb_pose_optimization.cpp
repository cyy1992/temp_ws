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

#include "uwb_pose_optimization.h"
#include <cartographer/transform/timestamped_transform.h>
#include <cartographer/transform/transform_interpolation_buffer.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

using namespace std;
using namespace cartographer;
using namespace cv;
Kalman::Kalman()
{
  kf_ = KalmanFilter(2,1,0);
  state_ = Mat(2, 1, CV_32F);
  measurement_ = Mat::zeros(1, 1, CV_32F);
  setIdentity(kf_.measurementMatrix);
  setIdentity(kf_.processNoiseCov, Scalar::all(10));
  setIdentity(kf_.measurementNoiseCov, Scalar::all(1e2));
  setIdentity(kf_.errorCovPost, Scalar::all(1));
  
  initialised_ = false;
}

double Kalman::Filter(const double& det_time, const double& distance)
{
  if(!initialised_)
  {
    auto min_dis = std::min_element(initial_distances_.begin(), initial_distances_.end());
    auto max_dis = std::max_element(initial_distances_.begin(), initial_distances_.end());
    if(!initial_distances_.empty() && initial_distances_.size() > 5)
    {
      if(fabs(distance - *min_dis) < 1.0 && 
        fabs(distance - *max_dis) < 1.0)
      {
        initialised_ = true;
        state_.at<float>(0) = distance;
        state_.at<float>(1) = 0;
        kf_.statePost = state_;
        last_distance_ = distance;
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
  
  if( fabs((last_distance_ - distance)/det_time) > 5.0 || fabs(last_distance_ - distance) > 3.5)
    return -1;
  kf_.transitionMatrix = (Mat_<float>(2, 2) << 1, det_time, 0, 1);
  measurement_ = distance;
  Mat prediction = kf_.predict();
  
  kf_.correct(measurement_);
  last_distance_ = distance;
  return (double)(kf_.statePost.at<float>(0));

}


uwbPoseOptimization::uwbPoseOptimization(const std::string& path)
{
  map_path_ = path;
  loadCloud(path);
  wall_timer_ = nh_.createWallTimer(::ros::WallDuration(0.2),
                          &uwbPoseOptimization::display, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("all_cloud", 1);
  uwb_pub_ = nh_.advertise<sensor_msgs::PointCloud>("uwb_cloud", 1);
}

uwbPoseOptimization::~uwbPoseOptimization()
{

}
void uwbPoseOptimization::display(const ros::WallTimerEvent& unused_timer_event)
{
  if(uwb_msg_.points.size() > 0)
  {
    cloud_msg_.header.stamp = ros::Time::now();
    cloud_msg_.header.frame_id = "map";
    uwb_msg_.header = cloud_msg_.header;
    cloud_pub_.publish(cloud_msg_);
    uwb_pub_.publish(uwb_msg_);
  }
  waitKey(2);
}

void uwbPoseOptimization::loadCloud(const std::string& filename)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudDS(new pcl::PointCloud<pcl::PointXYZI>);
  //*打开点云文件
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename + "/map.pcd", *cloud) == -1) {
      PCL_ERROR("Couldn't read file map.pcd\n");
  }
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
  downSizeFilterSurf.setLeafSize(1.0, 1.0, 1.0);
  downSizeFilterSurf.setInputCloud(cloud);
    downSizeFilterSurf.filter(*cloudDS);
  pcl::toROSMsg(*cloudDS, cloud_msg_);
  ifstream fi;
  fi.open(filename+ "/uwb_datas.txt");
  if(fi.is_open())
  {
    while(fi.good())
    {
      int64 time;
      fi >> time;
      Eigen::Vector4d data;
      fi >> data(0) >> data(1) >> data(2)>>data(3);
      uwb_datas_[common::FromUniversal(time)].push_back(data);
      id_with_distances_[static_cast<int>(data(0))].push_back({data(1), data(2),data(3)});
    }
  }
  fi.close();
  cout << uwb_datas_.begin()->second[0] <<endl;
  ifstream fi2;
  fi2.open(filename+ "/node_poses.txt");
  if(fi2.is_open())
  {
    while(fi2.good())
    {
      int64 time;
      fi2 >> time;
      double px,py,pz,qw,qx,qy,qz;
      fi2 >> px >> py >> pz>>qw >> qx >> qy>>qz;
      node_datas_[common::FromUniversal(time)] = transform::Rigid3d({px,py,pz},{qw,qx,qy,qz});
    }
  }
  fi2.close();
  cout << node_datas_.begin()->second <<endl;
  
  
  for(auto& it:id_with_distances_)
  {
    ofstream outFile1;
    outFile1.open("/home/cyy/distances_"+to_string(it.first)+ ".txt", std::ios::out);
    for(auto it2:it.second)
      outFile1 << it2(0) << " " << it2(1) <<" " << it2(2)<< endl;
    outFile1.close();
  }
//   cout << uwb_datas_.size() << ", " <<  uwb_poses_.size() <<endl;
//   cout << uwb_poses_.begin()->second <<endl;
  
  std::map<int, set<double>> init_distances;
  std::map<int, bool> id_initialised_;
  std::map<int, int> id_k;
  map<int,vector<cv::Point>> filter_points;
  map<int,vector<cv::Point>> origin_points;
  map<int, double> nearest_distance;
  for(auto it= uwb_datas_.begin(); it != uwb_datas_.end();)
  {
    for(vector<Eigen::Vector4d>::iterator it2 = it->second.begin();it2!=it->second.end();)
    {
      int cur_id = static_cast<int>((*it2)(0));
      double distance = (*it2)(1);
      double det_time = 0.1;
      if(last_times_.find(cur_id) != last_times_.end())
        det_time = common::ToSeconds(it->first - last_times_[cur_id]);
      
      if(distance > 50)
      {
        it2 = it->second.erase(it2);
        continue;
      }
      if(id_k.find(cur_id) != id_k.end() &&(det_time < 0.1 )){
        it2 = it->second.erase(it2);
        continue;
      }
      double filter_distance = kalman_filters_[cur_id].Filter(det_time,distance);
      if(filter_distance == -1)
        it2 = it->second.erase(it2);
      else
      {
        if(id_k.find(cur_id) == id_k.end())
          id_k[cur_id] = 0;
        else
          id_k[cur_id]++;
        
        if(nearest_time_.find(cur_id) == nearest_time_.end() || distance < nearest_distance[cur_id])
        {
          nearest_time_[cur_id] = it->first;
          nearest_distance[cur_id] = distance;
        }
        
        (*it2)(1) = filter_distance;
        
        filter_points[cur_id].push_back(Point(id_k[cur_id], it2->y() * 5.0));
        origin_points[cur_id].push_back(Point(id_k[cur_id], distance * 5.0));
        ++it2;
        last_times_[cur_id] = it->first;
      }
    }
    
    if(it->second.empty())
      it = uwb_datas_.erase(it);
    else
      ++it;
  }
  
  cout << "filter down!" <<endl;
  cout << uwb_datas_.size() <<endl;
  
  for(auto it:uwb_datas_)
  {
    map<common::Time, cartographer::transform::Rigid3d>::iterator it1 = node_datas_.begin();
    map<common::Time, cartographer::transform::Rigid3d>::iterator it2 = node_datas_.begin();
    ++it2;
    common::Time cur_t = it.first;
    for(; it2 != node_datas_.end(); ++it1,++it2)
    {
      common::Time t11 = it1->first;
      common::Time t22 = it2->first;
      if(cur_t > t11 && cur_t < t22){
        cartographer::transform::Rigid3d transform2 = cartographer::transform::Interpolate(  
            cartographer::transform::TimestampedTransform{t11, it1->second},
            cartographer::transform::TimestampedTransform{t22, it2->second},
            cur_t).transform;
        uwb_poses_[cur_t] = transform2;
        break;
      }
    }
  }
  cout << "uwb_poses size:" << uwb_poses_.size() <<endl;
  cout << uwb_poses_.rbegin()->second <<endl;
  optimizePoses(uwb_datas_,uwb_poses_);
  
  for(auto it: filter_points)
  {
    if(show_imgs_.find(it.first) == show_imgs_.end())
      show_imgs_[it.first] = Mat(1000, 1000, CV_8UC3);
    for(auto it2:it.second)
    {
      circle(show_imgs_[it.first], it2, 1,Scalar(0,0,255));
    }
  }
  for(auto it: origin_points)
  {
    for(auto it2:it.second)
    {
      circle(show_imgs_[it.first], it2, 1,Scalar(255,255,0));
    }
    imshow("img"+ to_string(it.first), show_imgs_[it.first]);
    
  }
  waitKey(2);
}

void uwbPoseOptimization::optimizePoses(
  const map< common::Time, vector< Eigen::Vector4d > >& uwb_datas,
  const map< common::Time, transform::Rigid3d >& uwb_poses)
{
  std::map<int, Eigen::Vector3d> ids;
  ceres::Problem problem;
  for(const auto& it:uwb_poses)
  {
    const common::Time time = it.first;
    transform::Rigid3d global_pose = it.second;
    vector<Eigen::Vector4d> uwb_data = uwb_datas.find(time)->second;

    for(Eigen::Vector4d node:uwb_data)
    {
      int id = static_cast<int>(node(0));
      if(ids.find(id) == ids.end())
      {
        const common::Time tmp_time = nearest_time_[id];
        Eigen::Vector3d t = uwb_poses.at(tmp_time).translation();
        ids[id] = t;
        cout << "id: " << id<< ", " << t.transpose()<<endl;
      }
      double weight = 10.0 ;
      double z = ids[id](2);
      problem.AddResidualBlock(
          DistanceMarkJzCostFunction::CreateAutoDiffCostFunction(node(1), weight, z, global_pose.translation()),
          nullptr, ids[id].data());
    }
  }
  ceres::Solver::Options options;
  options.max_num_iterations = 20000;
  options.linear_solver_type = ceres::DENSE_QR;
  options.function_tolerance = 1e-8;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout <<"final_cost:  " <<summary.final_cost <<std::endl;
  std::cout <<"final_cost:  " <<summary.FullReport() <<std::endl;
  for(auto id:ids)
  {
    cout <<id.first <<", " << id.second <<endl;
    Eigen::Vector3d t = id.second;
    geometry_msgs::Point32 p;
    p.x = t(0); p.y = t(1); p.z = t(2);
    uwb_msg_.points.push_back(p);
  }
  
  // save result
  boost::property_tree::ptree p_landmark_info;
  boost::property_tree::ptree p_neighbour_list;
  
  boost::property_tree::read_xml(map_path_ + "/landmark.xml", p_neighbour_list, 
     boost::property_tree::xml_parser::trim_whitespace, std::locale());
  for (const auto& it : ids)
  {
    int id = it.first;
    Eigen::Vector3d translation = it.second;
    p_landmark_info.put("ns", "Uwb");
    p_landmark_info.put("id", id);
    p_landmark_info.put("visible", 1);
    string data = "";
    for (int i = 0; i < 3; i++)
      data = data + std::to_string(translation[i]) + " ";
    Eigen::Quaterniond q(1,0,0,0);
    for (int i = 0; i < 4; i++)
      data = data + std::to_string(q.coeffs()[i]) + " ";
    p_landmark_info.put("transform", data);
    data.clear();
    p_neighbour_list.add_child("landmark", p_landmark_info);
    data.clear();
  }
  boost::property_tree::xml_writer_settings<std::string> setting(' ', 2);
  boost::property_tree::write_xml(map_path_ + "/landmark.xml", p_neighbour_list, std::locale(),
                                  setting);
  
 
  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "uwb_pose_optimization");
  string path = std::string(getenv("HOME"))+"/map/" + argv[1];
  uwbPoseOptimization pcl(path);
  ros::spin();
  return 1;
}