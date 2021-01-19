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

#include "point_cloud_trans.h"
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visual_servo_msgs/Model3DPoseArray.h>
using namespace std;
inline Rigid3d ToRigid3d(const geometry_msgs::TransformStamped& tf_pose)
{
  Rigid3d pose;
  pose.t = Eigen::Vector3d(tf_pose.transform.translation.x, 
                           tf_pose.transform.translation.y,
                           tf_pose.transform.translation.z);
  pose.q = Eigen::Quaterniond(tf_pose.transform.rotation.w, 
                              tf_pose.transform.rotation.x,
                           tf_pose.transform.rotation.y,
                           tf_pose.transform.rotation.z);
  return pose;
}
inline geometry_msgs::TransformStamped FromRigid3d(const Rigid3d& pose)
{
  geometry_msgs::TransformStamped tf_pose;
  tf_pose.transform.translation.x = pose.t.x();
  tf_pose.transform.translation.y = pose.t.y();
  tf_pose.transform.translation.z = pose.t.z();
  tf_pose.transform.rotation.x = pose.q.x();
  tf_pose.transform.rotation.y = pose.q.y();
  tf_pose.transform.rotation.z = pose.q.z();
  tf_pose.transform.rotation.w = pose.q.w();
  return tf_pose;
}
PointcloudTrans::PointcloudTrans(const ros::NodeHandle& n):nh_(n),tfBuffer_(ros::Duration(20.)),tfListener_(tfBuffer_),initialised_(false)
{
  cloud2_sub_ = nh_.subscribe("/pointcloud_back",10, &PointcloudTrans::HandleCloudBack,this);
//   cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_back1",10);
  cloud_all_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/merge_back1",10);
  target_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/target_cloud",10);
  source_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/source_cloud",10);
  polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/polygon_in_ref",10);
  model_pose_pub_ = nh_.advertise<visual_servo_msgs::Model3DPoseArray>("/servo_guard/model_3d_pose",10);
  down_sample_.setLeafSize(0.1f, 0.1f, 0.1f);
  points_.reset(new pcl::PointCloud<PointType>());
  servers_.push_back(nh_.advertiseService("/point_cloud_trans/set_target_pose", &PointcloudTrans::SetTargetPose, this));
  servers_.push_back(nh_.advertiseService("/point_cloud_trans/set_model_name", &PointcloudTrans::SetModelName, this));
  ros::Rate rate(10);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>); 

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/cyy/finalMap.pcd", *cloud1) == -1) 
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n"); 
  }
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.setInputCloud (cloud1);
  target_points_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  std::cout<<"\n target.size()="<<cloud1->size()<<std::endl;
  sor.filter(*target_points_);
  std::cout<<"\n targetFilter.size()="<<target_points_->size()<<std::endl;
  pcl::toROSMsg(*target_points_, target_msg_);
  
  wall_timer_ = nh_.createWallTimer(::ros::WallDuration(0.02),
                          &PointcloudTrans::PubTf, this);
  model_name = "load";
//   while(1){
//     
// //     try {
// //         lidar2base_tf_ = tfBuffer_.lookupTransform("base_footprint", "lidar_link2",
// //                 ros::Time(0));
// // //         tf2::doTransform(point_cloud_origin, point_cloud, transformStamped);
// // 
// //   //          pcl_ros::transformPointCloud("robotarm", cloud_in, cloud_out, tfListener);
// //         break;
// // 
// //     } catch (tf2::TransformException &ex) {
// //       rate.sleep();
// //     }
//   }
  ref_x = -12.50;ref_y = 0.8; ref_z = 1.5;
  max_x_ = 2.5; max_y_ = 2.5; max_z_ = 2;
}

PointcloudTrans::~PointcloudTrans()
{

}

void PointcloudTrans::HandleCloudBack(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if(!initialised_)
    return;
  sensor_msgs::PointCloud2 point_cloud_origin;
  sensor_msgs::PointCloud2 point_cloud,point_cloud_in_base;
  point_cloud_origin = (*msg);
  ros::Time cur_time = msg->header.stamp  /*+ ros::Duration(1610179077.576312);*/;
  geometry_msgs::TransformStamped base2odom_tf, lidar2base;
  ros::Rate rate(10);
  cout << cur_time  <<"," << ros::Time::now()<<endl;
  int kkk = 0;
  Rigid3d base2odom;
  Rigid3d base2ref_odom;
  geometry_msgs::TransformStamped base2ref_odom_tf;
  while(1){
    try {
      base2odom_tf = tfBuffer_.lookupTransform("odom", "base_footprint", cur_time);
      lidar2base = tfBuffer_.lookupTransform("base_footprint", msg->header.frame_id, ros::Time(0));
      base2odom = ToRigid3d(base2odom_tf);
      base2ref_odom = ref_odom2odom_.inverse() * base2odom;
      base2ref_odom_tf = FromRigid3d(base2ref_odom);
      
      tf2::doTransform(point_cloud_origin, point_cloud_in_base, lidar2base);
      tf2::doTransform(point_cloud_in_base, point_cloud, base2ref_odom_tf);
      break;
  //          pcl_ros::transformPointCloud("robotarm", cloud_in, cloud_out, tfListener);

    } catch (tf2::TransformException &ex) {
      rate.sleep();
      if(kkk++ > 50){
        cout << "can not get tf" <<endl;
        return;
      }
    }
  }
  
  int range_size = point_cloud.width * point_cloud.height;
  if(range_size > 0)
  {
    sensor_msgs::PointCloud2Iterator<float> iter_x0(point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y0(point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z0(point_cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity0(point_cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float> iter_x_base(point_cloud_in_base, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y_base(point_cloud_in_base, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z_base(point_cloud_in_base, "z");
    sensor_msgs::PointCloud2Iterator<int> iter_i_base(point_cloud_in_base, "reflectivity");
//     sensor_msgs::PointCloud2 cloud_msg;
// //     cloud_msg.header = msg->header;
//     cloud_msg.header.stamp = cur_time;
//     cloud_msg.header.frame_id = "base_footprint";
//     cloud_msg.height = 1;
//     cloud_msg.width = range_size;

//     sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
//     modifier.setPointCloud2Fields(4,
//                                   "x", 1, sensor_msgs::PointField::FLOAT32,
//                                   "y", 1, sensor_msgs::PointField::FLOAT32, 
//                                   "z", 1, sensor_msgs::PointField::FLOAT32,
//                                   "intensity", 1, sensor_msgs::PointField::FLOAT32);
//     modifier.resize(range_size);
//     sensor_msgs::PointCloud2Iterator<float> iter_x1(cloud_msg, "x");
//     sensor_msgs::PointCloud2Iterator<float> iter_y1(cloud_msg, "y");
//     sensor_msgs::PointCloud2Iterator<float> iter_z1(cloud_msg, "z");
//     sensor_msgs::PointCloud2Iterator<float> iter_intensity1(cloud_msg, "intensity");
//     vector<Eigen::Vector3d> temp_points;
//     temp_points.reserve(range_size);
    
    for(int i = 0; i < range_size; i++)
    {
      float temp_x = *iter_x_base;
      
      if(temp_x > -10 || (*iter_i_base) < 300 ){
        ++iter_x0;
        ++iter_y0;
        ++iter_z0;
        ++iter_intensity0;
        
//         ++iter_x1;
//         ++iter_y1;
//         ++iter_z1;
//         ++iter_intensity1;
//         
        ++iter_x_base;
        ++iter_y_base;
        ++iter_z_base;
        ++iter_i_base;
//         cout << temp_x <<endl;
        continue;
      }
//       *iter_x1 = *iter_x_base;
//       *iter_y1 = *iter_y_base;
//       *iter_z1 = *iter_z_base;
//       if(*iter_z0 > -0.02 && *iter_z0 < 0.02 )
//         *iter_intensity1 = 500;
//       else 
//         *iter_intensity1 = 0.;
//       *iter_intensity1 = *iter_intensity0;
//       temp_points.push_back(Eigen::Vector3d(*iter_x0, *iter_y0, *iter_z0));
      
      if((*iter_x0) > (ref_x - max_x_ )&& (*iter_x0) <  (ref_x + max_x_)&&
          (*iter_y0) > (ref_y - max_y_)&& (*iter_y0) < (ref_y + max_y_) &&
          (*iter_z0) > (ref_z - max_z_ )&& (*iter_z0) < (ref_z + max_z_))
      {
        PointType p;
//         p.x = *iter_x0;
//         p.y = *iter_y0;
//         p.z = *iter_z0;
        Eigen::Vector3d point_in_ref_odom(*iter_x0,*iter_y0,*iter_z0);
        Eigen::Vector3d point_in_obj = obj2ref_odom_.inverse() * point_in_ref_odom;
        p.x = point_in_obj.x();
        p.y = point_in_obj.y();
        p.z = point_in_obj.z();
        
        points_->points.push_back(p);
        
        
      }
      ++iter_x0;
      ++iter_y0;
      ++iter_z0;
      ++iter_intensity0;
      
//       ++iter_x1;
//       ++iter_y1;
//       ++iter_z1;
//       ++iter_intensity1;
//       
      ++iter_x_base;
      ++iter_y_base;
      ++iter_z_base;
      ++iter_i_base;
    }
//     cloud_msg.header.frame_id = "odom";
//     cloud_msg.header.stamp = cur_time;
//     cloud2_pub_.publish(cloud_msg);
    
    static int kk = 1;
    
//     cloud_points_.insert(cloud_points_.end(), temp_points.begin(), temp_points.end());
    if(kk%2 == 0)
    {
      sensor_msgs::PointCloud2 all_cloud_msg;
      pcl::PointCloud<PointType>::Ptr down_sample_points(new pcl::PointCloud<pcl::PointXYZ>);
      
      down_sample_.setInputCloud(points_);
      down_sample_.filter(*down_sample_points);
      pcl::toROSMsg(*down_sample_points, all_cloud_msg);
      all_cloud_msg.header.frame_id = "obj_link";
      all_cloud_msg.header.stamp = cur_time;
      source_pub_.publish(all_cloud_msg);
      
      Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
      ICPMatch(target_points_, down_sample_points, transform);
      cout << "result transform: " << transform <<endl;
      Eigen::Matrix4d target2obj_transform = transform.inverse();
      Eigen::Matrix3d rotation = target2obj_transform.block(0,0,3,3);
      Eigen::Quaterniond q_temp(rotation);
      Rigid3d target2obj = Rigid3d({target2obj_transform(0, 3),target2obj_transform(1, 3),target2obj_transform(2, 3)},q_temp);
//       Eigen::Quaternionf q = q_temp.cast<float>();
//       Eigen::Vector3d t_temp(target2obj(0, 3), target2obj(1, 3), target2obj(2, 3));
      
      target_msg_.header.stamp = cur_time;
      target_msg_.header.frame_id  = "target_link";
      target_pub_.publish(target_msg_);
      std::vector<geometry_msgs::TransformStamped> stamped_transforms;
      geometry_msgs::TransformStamped stamped_transform;
      stamped_transform.header.stamp = cur_time;
      
      stamped_transform.header.frame_id = "obj_link";
      stamped_transform.child_frame_id = "target_link";
      stamped_transform.transform.translation.x = target2obj_transform(0, 3);
      stamped_transform.transform.translation.y = target2obj_transform(1, 3);
      stamped_transform.transform.translation.z = target2obj_transform(2, 3);
      stamped_transform.transform.rotation.w = q_temp.w();
      stamped_transform.transform.rotation.x = q_temp.x();
      stamped_transform.transform.rotation.y = q_temp.y();
      stamped_transform.transform.rotation.z = q_temp.z();
      stamped_transforms.push_back(stamped_transform);
      
      stamped_transform.header.frame_id = "ref_odom";
      stamped_transform.child_frame_id = "base_footprint";
      stamped_transform.transform = base2ref_odom_tf.transform;
      stamped_transforms.push_back(stamped_transform);
      
      ref_odom2odom_tf_.header.stamp = ros::Time::now();
      stamped_transforms.push_back(ref_odom2odom_tf_);
      obj2ref_odom_tf_.header.stamp = ros::Time::now();
      stamped_transforms.push_back(obj2ref_odom_tf_);
      tf_broadcaster_.sendTransform(stamped_transforms);
      
      
      obj_polygon_.header.frame_id = "obj_link";
      obj_polygon_.header.stamp = ros::Time::now();
      polygon_pub_.publish(obj_polygon_);
      
      Rigid3d target2base = base2ref_odom.inverse() * obj2ref_odom_ * target2obj;
      visual_servo_msgs::Model3DPoseArray model_poses_msg;
      visual_servo_msgs::Model3DPose model_pose_msg;
      model_pose_msg.model_name = model_name;
      model_pose_msg.valid = true;
      model_pose_msg.pose.position.x = target2base.t.x();
      model_pose_msg.pose.position.y = target2base.t.y();
      model_pose_msg.pose.position.z = target2base.t.z();
      
      model_pose_msg.pose.orientation.x = target2base.q.x();
      model_pose_msg.pose.orientation.y = target2base.q.y();
      model_pose_msg.pose.orientation.z = target2base.q.z();
      model_pose_msg.pose.orientation.w = target2base.q.w();
      model_poses_msg.model_pose_array.push_back(model_pose_msg);
      model_pose_pub_.publish(model_poses_msg);
    }
    kk++;
  }

}

void PointcloudTrans::ICPMatch(const pcl::PointCloud<PointType>::Ptr &cloud_target, 
                 const pcl::PointCloud<PointType>::Ptr &cloud_source, 
                 Eigen::Matrix4d &transform ) //transform source2target
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr src = cloud_source;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt = cloud_target;
  
  cout << "src size: " << src->size() <<"\t target size: " << tgt->size() <<endl;
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(0.8);
  icp.setTransformationEpsilon(1e-10);
  icp.setEuclideanFitnessEpsilon(0.01);
  icp.setMaximumIterations(1000);  
  icp.setInputSource (src);
  icp.setInputTarget (tgt);
  icp.align (*src);
  if(icp.hasConverged())
  {
    cout << "icp success" <<endl;
    transform = icp.getFinalTransformation().cast<double>();
  }
  else
     cout << "icp failed" <<endl;
}

void PointcloudTrans::PubTf(const ros::WallTimerEvent& unused_timer_event)
{
  
}

bool PointcloudTrans::SetTargetPose(test6::SetPose::Request& request,
                                    test6::SetPose::Response& response)
{
  points_.reset(new pcl::PointCloud<PointType>());
  
  initialised_ = true;
  ref_x = request.pose.position.x;
  ref_y = request.pose.position.y;
  ref_z = request.pose.position.z;
  
  if(request.pose.orientation.x != 0)
    max_x_ = request.pose.orientation.x;
  if(request.pose.orientation.y != 0)
    max_y_ = request.pose.orientation.y;
  if(request.pose.orientation.z != 0)
    max_z_ = request.pose.orientation.z;
  
  obj2ref_odom_.t = Eigen::Vector3d(request.pose.position.x, request.pose.position.y,request.pose.position.z);
  obj2ref_odom_tf_ = FromRigid3d(obj2ref_odom_);
  obj2ref_odom_tf_.child_frame_id = "obj_link";
  obj2ref_odom_tf_.header.frame_id = "ref_odom";
  geometry_msgs::TransformStamped base2odom_tf;
  ros::Rate rate(10);
  int kkk = 0;
  while(1){
    try {
      base2odom_tf = tfBuffer_.lookupTransform("odom", "base_footprint", ros::Time(0));
      ref_odom2odom_ = ToRigid3d(base2odom_tf);
      break;
    } catch (tf2::TransformException &ex) {
      rate.sleep();
      if(kkk++ > 50){
        response.success = false;
        response.message = "can not get tf";
        return true;
      }
    }
  }
  setPolyGon();
  
  ref_odom2odom_tf_.header.frame_id =
      "odom";
  ref_odom2odom_tf_.child_frame_id =
      "ref_odom";
  ref_odom2odom_tf_.header.stamp = ros::Time::now();
  ref_odom2odom_tf_.transform = base2odom_tf.transform;
  
  response.success = true;
  return true;
}

bool PointcloudTrans::SetModelName(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  bool flag = request.data;
//   if(flag)
//     model_name = "";
}


void PointcloudTrans::setPolyGon()
{
  obj_polygon_.polygon.points.clear();
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
}

int main(int argc, char **argv)
{
//   google::InitGoogleLogging(argv[0]);
//   google::InstallFailureSignalHandler();
//   FLAGS_alsologtostderr = true;
//   FLAGS_colorlogtostderr = true;
  
  ros::init(argc, argv, "point_cloud_trans");

  ros::NodeHandle n;
  PointcloudTrans a(n);
  ros::spin();
  return 1;
}