#include <opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include<iostream>
#include <string>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>
#include <visual_servo_msgs/Model3DPoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
using namespace cv;
using namespace std;
using namespace std::chrono;
void fillVector(vector<float>& temp);
void HandleGps(const gps_common::GPSFix::ConstPtr& msg);
void setPolyGon();
void HandleModelPose(const visual_servo_msgs::Model3DPoseArray::ConstPtr& msg);
ros::Subscriber gps_sub_;
ros::Subscriber model_sub_;
ros::Publisher navsat_pub_,polygon_pub_;
int main(int argc, char** argv)
{
  ros::init(argc,argv,"test66");
  ros::NodeHandle n;
  navsat_pub_ = n.advertise<sensor_msgs::NavSatFix>("/jzhw/navsat", 1);
  polygon_pub_ = n.advertise<geometry_msgs::PolygonStamped>("/trailer_polygon", 1);
  gps_sub_ = n.subscribe("/jzhw/gps/fix", 5, HandleGps);
  model_sub_ = n.subscribe("/servo_guard/model_3d_pose", 5, HandleModelPose);
  setPolyGon();

  ros::spin();
  return 0;
}
void fillVector(vector<float>& temp)
{
  for(int i =0; i < 100000; i ++)
    temp.push_back(1.222 + i);
}


void HandleGps(const gps_common::GPSFix::ConstPtr& msg)
{
  sensor_msgs::NavSatFix fix_msg;
  fix_msg.header = msg->header;
  fix_msg.latitude = msg->latitude;
  fix_msg.longitude = msg->longitude;
  fix_msg.altitude = msg->altitude;
  fix_msg.position_covariance_type = 0;
  fix_msg.header.frame_id = "fix_link";
  navsat_pub_.publish(fix_msg);
  
}
geometry_msgs::PolygonStamped obj_polygon_;
float max_x_ = 4.2;
float max_y_ = 2.5; 
float max_z_ = 2;
void setPolyGon()
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

void HandleModelPose(const visual_servo_msgs::Model3DPoseArray::ConstPtr& msg)
{
  visual_servo_msgs::Model3DPose pose = msg->model_pose_array[0];
  geometry_msgs::TransformStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = "base_footprint";
  pose_stamped.child_frame_id = "trailer_link2";
  pose_stamped.transform.translation.x = pose.pose.position.x;
  pose_stamped.transform.translation.y = pose.pose.position.y;
  pose_stamped.transform.translation.z = pose.pose.position.z;
  
  pose_stamped.transform.rotation.x = pose.pose.orientation.x;
  pose_stamped.transform.rotation.y = pose.pose.orientation.y;
  pose_stamped.transform.rotation.z = pose.pose.orientation.z;
  pose_stamped.transform.rotation.w = pose.pose.orientation.w;
  static tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf_broadcaster_.sendTransform(pose_stamped);
  
  obj_polygon_.header.frame_id = "trailer_link2";
  obj_polygon_.header.stamp = ros::Time::now();
  polygon_pub_.publish(obj_polygon_);
}

//   vector<float> temp1, temp2, temp3;
//   steady_clock::time_point t1 = steady_clock::now();
//   fillVector(temp1);
//   steady_clock::time_point t2 = steady_clock::now();
//   duration<double> time_span = 
//       duration_cast<duration<double>>(t2 - t1);
//   cout << temp1.capacity()<<endl;
//   temp2 = temp1;    
//   temp1.reserve(200000);
//   cout << temp1.capacity()<<endl;
//   steady_clock::time_point t3 = steady_clock::now();
//   fillVector(temp1);
//   steady_clock::time_point t4 = steady_clock::now();
//   
//   duration<double> time_span2 = 
//       duration_cast<duration<double>>(t4 - t3);
//   
//   steady_clock::time_point t5 = steady_clock::now();
//   temp2.insert(temp2.end(), temp2.begin(), temp2.end());
//   steady_clock::time_point t6 = steady_clock::now();
//   
//   duration<double> time_span3 = 
//       duration_cast<duration<double>>(t6 - t5);
//   cout << time_span.count() <<", " << time_span2.count() <<", " << time_span3.count()<<endl;

//     Mat mask = Mat(Size(image.cols, image.rows),CV_8UC1,Scalar(255)); 
//     Point p1 = { 25, 60 };  Point p2 = { 50, 110 };  Point p4 = { 100, 60 }; Point p3 = { 100, 110 }; /*Point p5 = { 50, 10 };*/
//     vector<Point> contour;
//     contour.push_back(p1);
//     contour.push_back(p2);
//     contour.push_back(p3);
//     contour.push_back(p4);
// //     contour.push_back(p5);
//     vector<vector<Point> > contours;
//     contours.push_back(contour);
//     Point p11 = { 30,60 };  Point p21 = { 70, 110 };  Point p41 = { 100, 90 }; Point p31 = { 200, 150 };
//     cv::drawContours(mask, contours, -1, cv::Scalar::all(0),CV_FILLED);
//     vector<vector<Point> > contours1;
//     vector<Point> contour1;
//     contour1.push_back(p11);
//     contour1.push_back(p21);
//     contour1.push_back(p31);
//     contour1.push_back(p41);
//     contours1.push_back(contour1);
//     cv::drawContours(mask, contours1, -1, cv::Scalar::all(0),CV_FILLED);
//     imshow("maskRegion",mask);
//     waitKey();
//     Mat maskImage;
//     image.copyTo(maskImage, mask);
//     imshow("getMaskImage", maskImage);
//     waitKey();
//     return 0;
// }

// #include <iostream>
// #include <ros/ros.h>
// #include <ros/package.h>
// #include <sensor_msgs/LaserScan.h>
// #include <sensor_msgs/SetCameraInfo.h>
// #include <geometry_msgs/TwistStamped.h>
// #include <cv_bridge/cv_bridge.h>
// 
// #include <cartographer/io/proto_stream.h>
// #include <cartographer/io/proto_stream_deserializer.h>
// #include <cartographer/io/proto_stream_interface.h>
// #include <cartographer/mapping/pose_graph.h>
// #include <cartographer/mapping/3d/submap_3d.h>
// #include <std_msgs/String.h>
// #include <cartographer_ros_msgs/Submap3D.h>
// #include <sensor_msgs/PointCloud.h>
// #include <cartographer/sensor/internal/voxel_filter.h>
// #include <boost/property_tree/ptree.hpp>
// #include <boost/property_tree/xml_parser.hpp>
// using namespace std;
// int j =0;
// ros::Publisher pub_tmp_;
// void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
// {
// 	j++;
// 	if(j > 100)
// 	{
// 		int status = system("rosnode kill /test6");
//     assert(status != -1);
// 	}
// }
// cartographer::sensor::AdaptiveVoxelFilter* adaptive_voxel_filter_;
// void submapCb(const cartographer_ros_msgs::Submap3DPtr& msg)
// {
// 	std::string submap_data = msg->submap;
// 	cartographer::mapping::proto::TrajectoryNodeData proto;
// 
// 	proto.ParseFromString(submap_data);
// 	
// 	cout << proto.has_local_pose() <<endl;
// 	cout << proto.has_gravity_alignment() <<endl;
// 	cout << proto.has_high_resolution_point_cloud() <<endl;
// // 	cout <<proto.TrajectoryNodeData::has_local_pose()<<endl;
// // 	cartographer::mapping::proto::TrajectoryNodeData proto_trajectory_data = proto.trajectory_data();
// // 	cartographer::mapping::TrajectoryNode::Data tmpdata = cartographer::mapping::FromProto(proto_trajectory_data);
// 	std::shared_ptr<cartographer::mapping::TrajectoryNode::Data>
// 	trajectory_data_ = std::make_shared<cartographer::mapping::TrajectoryNode::Data>(
// 		cartographer::mapping::FromProto(proto));
// 	sensor_msgs::PointCloud tmp;
// 	cartographer::sensor::PointCloud high_tmp = adaptive_voxel_filter_->Filter(trajectory_data_->high_resolution_point_cloud);
// 	
// 	for(int i=0; i<high_tmp.size();i++)
// 	{
// 		cartographer::sensor::RangefinderPoint point = high_tmp[i];
// 		geometry_msgs::Point32 pub_point;
// 		pub_point.x = point.position(0);
// 		pub_point.y = point.position(1);
// 		pub_point.z = point.position(2);
// 		tmp.points.push_back(pub_point);
// 	}
// 	tmp.header.frame_id = "base_footprint";
// 	tmp.header.stamp = msg->header.stamp;
// 	
// 	pub_tmp_.publish(tmp);
// // 	bool need_debug_ = false;
// // 	bool submap_valid_ = msg->submap_valid;
// // 	
// // 	cout <<"trajectory_data_->local_pose: " << trajectory_data_->local_pose <<endl;
// // 	cout <<"trajectory_data_->gravity_alignment: " << trajectory_data_->gravity_alignment.toRotationMatrix() <<endl;
// }
// int main(int argc, char **argv)
// {
// 	ros::init(argc, argv,"test6");
// 	ros::NodeHandle n;
// 	std::string path_tmp = ros::package::getPath("calib_scans");
// 	cout << path_tmp <<endl;
// 	return 1;
// 	string path = "/home/cyy/shareFiles";
// 	boost::property_tree::ptree pt,pt_poses;
//   boost::property_tree::read_xml(path + "/data.xml", pt);
// 	pt_poses = pt.get_child("submap_poses");
// 	std::string data;
// 	data = pt_poses.get<std::string>("submap_global_pose");
// 	float pose[7];
//   sscanf(data.c_str(), "%f %f %f %f %f %f %f", &pose[0], &pose[1], &pose[2],
//          &pose[3], &pose[4], &pose[5], &pose[6]);
// 
//   Eigen::Quaternionf q = Eigen::Quaternionf(pose[6], pose[3], pose[4], pose[5]);
// 	Eigen::Matrix3f rotMat = q.matrix();
//   Eigen::Isometry3d submap2global_pose; // submap2global_pose
// 	submap2global_pose.setIdentity();
//   for (int i = 0; i < 3; i++)
//     for (int j = 0; j < 3; j++)
//       submap2global_pose(i, j) = rotMat(i, j);
//   submap2global_pose(0, 3) = pose[0];
//   submap2global_pose(1, 3) = pose[1];
//   submap2global_pose(2, 3) = pose[2];
// 	cout << submap2global_pose.matrix() <<endl;
// 	data.clear();
// 	data = pt_poses.get<std::string>("mid_node_global_pose");
// 	sscanf(data.c_str(), "%f %f %f %f %f %f %f", &pose[0], &pose[1], &pose[2],
//          &pose[3], &pose[4], &pose[5], &pose[6]);
// 
//   q = Eigen::Quaternionf(pose[6], pose[3], pose[4], pose[5]);
// 	rotMat = q.matrix();
//   Eigen::Isometry3d node2global_pose; // submap2global_pose
// 	node2global_pose.setIdentity();
//   for (int i = 0; i < 3; i++)
//     for (int j = 0; j < 3; j++)
//       node2global_pose(i, j) = rotMat(i, j);
//   node2global_pose(0, 3) = pose[0];
//   node2global_pose(1, 3) = pose[1];
//   node2global_pose(2, 3) = pose[2];
// 	cout << node2global_pose.matrix() <<endl;
// 	return 1;
// 	cartographer::sensor::proto::AdaptiveVoxelFilterOptions
//       adaptive_voxel_filter_options;
//   adaptive_voxel_filter_options.set_max_length(
//       0.2);
//   adaptive_voxel_filter_options.set_min_num_points(
//       3000);
//   adaptive_voxel_filter_options.set_max_range(20);
//   adaptive_voxel_filter_ =
//       new cartographer::sensor::AdaptiveVoxelFilter(adaptive_voxel_filter_options);
// 	ros::Subscriber sub = n.subscribe("submap3D_local",1,submapCb);
// 
// 			
// 	pub_tmp_ = n.advertise<sensor_msgs::PointCloud>("test_point_cloud",100);
// 	ros::spin();
// 	string file_path = "/home/cyy/map/test212/frames/1/1.pbstream";
// 	cartographer::io::ProtoStreamReader stream(file_path);
// 	std_msgs::String tmp;
// // 	cartographer::io::ProtoStreamDeserializer deserializer(&stream);
// // 	cartographer::mapping::proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
// 	cartographer::mapping::proto::SerializedData proto;
//   
// // 	MapById<cartographer::mapping::SubmapId, cartographer::mapping::proto::Submap> submap_id_to_submap;
//   while(stream.ReadProto(&proto)) {
//     if  (proto.data_case()==cartographer::mapping::proto::SerializedData::kSubmap) {
// //         proto.mutable_submap()->mutable_submap_id()->set_trajectory_id(
// //             trajectory_remapping.at(
// //                 proto.submap().submap_id().trajectory_id()));
// //         submap_id_to_submap.Insert(
// //             SubmapId{proto.submap().submap_id().trajectory_id(),
// // 										 proto.submap().submap_id().submap_index()},
// //             proto.submap());
// 				std::shared_ptr<const cartographer::mapping::Submap3D> submap_ptr =
// 					std::make_shared<const cartographer::mapping::Submap3D>(proto.submap().submap_3d());
// 				int num_range_data = submap_ptr->num_range_data();
// 				bool finished = submap_ptr->insertion_finished();
// 				cartographer::transform::Rigid3d local_pose = submap_ptr->local_pose();
// 				
// 				cout << "num_range_data: " << num_range_data <<endl;
// 				cout << "finished: " << finished <<endl;
// 				cout << "local_pose: " << local_pose <<endl;
// 		}
// 		else
// 		{
// 			cout << proto.data_case() <<endl;
// 		}
// 	}
// // 	proto.ParseFromString("");
// 	
// // 	ros::Duration a(5);
// // 	ros::Time t1 = ::ros::Time::now();
// // 	cout << t1 <<endl;
// // 	sleep(2);
// // 	ros::Time t2 = t1 +a;
// // 	cout << t1 -t2 <<endl;
// // 	ros::Subscriber sub = n.subscribe("/scan",1,ScanCallback);
// // 	ros::Publisher pub;
// // 	pub = n.advertise<geometry_msgs::TwistStamped>("/cmd",1);
// // 	ros::spin();
// // // 	ros::ServiceClient client = n.serviceClient("")
// // 	
// // 	
// // 	
// // 	while(1){
// // 		geometry_msgs::TwistStamped pose;
// // 		pose.header.stamp = ros::Time::now();
// // 		pose.twist.linear.x = 0;
// // 		pub.publish(pose);
// // 	}
// 
// 	cout << "**********************************************"<<endl;
// 	return 1;
// 	
// }

// #include <iostream>
// #include <ros/ros.h>
// #include <ros/package.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/SetCameraInfo.h>
// #include <geometry_msgs/TwistStamped.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <vtr_msgs/GlobalLocalizationPose.h>
// #include <cartographer/common/time.h>
// #include <cartographer_ros_msgs/SetSwitch.h>
// using namespace std;
// using namespace cv;
// ros::Subscriber sub_;
// ros::Publisher pub_;
// ros::ServiceClient client_;

// ::ros::Time ToRos(::cartographer::common::Time time)
// {
//   int64 uts_timestamp = ::cartographer::common::ToUniversal(time);
//   int64 ns_since_unix_epoch =
//       (uts_timestamp -
//        ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
//            10000000ll) *
//       100ll;
//   ::ros::Time ros_time;
//   ros_time.fromNSec(ns_since_unix_epoch);
//   cout << ros_time <<endl;
//   return ros_time;
// }
// 
// ::cartographer::common::Time FromRos(const ::ros::Time& time)
// {
//   // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
//   // exactly 719162 days before the Unix epoch.
//   return ::cartographer::common::FromUniversal(
//       (time.sec +
//        ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
//           10000000ll +
//       (time.nsec + 50) / 100); // + 50 to get the rounding correct.
// }
// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "temp_test");
//   ros::NodeHandle n;
//   client_ = n.serviceClient<cartographer_ros_msgs::SetSwitch>("/mark_localization/set_mark_switch");
//   int i =0;
//   ros::Rate rate(5);
//   while(ros::ok())
//   {
//     rate.sleep();
//     cartographer_ros_msgs::SetSwitch srv;
//     if(i % 2 == 0)
//     {
//       srv.request.type = "LaserScanOdom";
//       srv.request.flag = true;
//       client_.call(srv);
//       srv.request.type = "StripLocalization";
//       srv.request.flag = false;
//       client_.call(srv);
//     }
//     else
//     {
//       srv.request.type = "LaserScanOdom";
//       srv.request.flag = false;
//       client_.call(srv);
//       srv.request.type = "StripLocalization";
//       srv.request.flag = true;
//       client_.call(srv);
//     }
//     cout <<"i:" <<i <<endl;
//     i++;
//     
//     ros::spinOnce();
//   }
//   return 1;
//   
// }

// ros::Subscriber global_pose_sub_;
// void handleImage(const sensor_msgs::ImageConstPtr& msg )
// {
//   Mat origin_image = cv_bridge::toCvShare(msg, "mono8")->image;
//   Mat resize_image;
//   resize(origin_image,resize_image,Size(origin_image.cols/2,origin_image.rows/2));
//   sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(msg->header, "mono8", resize_image).toImageMsg();
//   pub_.publish(msg1);
// }
// void handlePose(const vtr_msgs::GlobalLocalizationPoseConstPtr& msg )
// {
//   cout << "\033[32m sensor2odom: \n" << msg->sensor2odom.position.x << "," 
//   << msg->sensor2odom.position.y << ","
//   << msg->sensor2odom.position.z << ","
//   << msg->sensor2odom.orientation.w << ","
//   << msg->sensor2odom.orientation.x << ","
//   << msg->sensor2odom.orientation.y << ","
//   << msg->sensor2odom.orientation.z << endl;
//   
//   cout << "\033[33m sensor2reference: \n" << msg->sensor2reference.pose.position.x << "," 
//   << msg->sensor2reference.pose.position.y << ","
//   << msg->sensor2reference.pose.position.z << ","
//   << msg->sensor2reference.pose.orientation.w << ","
//   << msg->sensor2reference.pose.orientation.x << ","
//   << msg->sensor2reference.pose.orientation.y << ","
//   << msg->sensor2reference.pose.orientation.z <<"\033[37m"<< endl;
// }
// int main(int argc, char **argv)
// {
//   
// //   Mat imag = imread("/home/cyy/map/sdpx_gps/show_map.png");
// //   imwrite("/home/cyy/map/sdpx_gps/show_map.jpg",imag);
// //   return 1;
//   ros::init(argc, argv, "ImageResize");
//   ros::NodeHandle n;
//   global_pose_sub_ = n.subscribe<vtr_msgs::GlobalLocalizationPose>("/vtr/global_localisation/response",10,handlePose);
//   ros::spin();
//   return 1;
//   string pub_img_topic;
//   string sub_img_topic;
//   if(!ros::param::get("~sub_img_topic",sub_img_topic))
//   {
//     cout << "Can not get params! " <<endl;
//   }
//   if(!ros::param::get("~pub_img_topic",pub_img_topic))
//   {
//     cout << "Can not get params! " <<endl;
//   }
//   sub_ = n.subscribe( sub_img_topic, 20, handleImage);
//   pub_ = n.advertise<sensor_msgs::Image>(pub_img_topic,1);
//   ros::spin();
//   return 1;
// }
// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/console/print.h>
// #include <pcl/console/parse.h>
// #include <pcl/console/time.h>
// #include <pcl/io/vtk_lib_io.h>
// #include <pcl/io/vtk_io.h>
// #include <vtkPolyData.h>
// #include <vtkSmartPointer.h>
// #include <pcl/visualization/cloud_viewer.h>  
//  #include <pcl/conversions.h>
// #include <opencv2/opencv.hpp>
// #include <stdio.h>
// using namespace pcl;
// using namespace pcl::io;
// using namespace pcl::console;
//  using namespace std;
// int main()
// {
//   FILE *fp;
//   char buffer[20];
// 
//   fp = popen("cat /home/cyy/jz_total_*.txt |grep calib | awk -F'==' '{print $2}'", "r");
//   if (fp != NULL)
//   {
//     while (fgets(buffer, 20, fp) != NULL)
//     {}
//     pclose(fp);
//   }
//   
//   cout << temp <<"."<<endl;
  // pcl::PolygonMesh mesh;

  // if (pcl::io::loadPLYFile("/media/cyy/CYY_DISK/others/bags/gps/out1.bag_points.ply", mesh))
  // {
  //   std::cout << "文件读取失败！";
  // }
  
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer window"));
  // viewer->addPolygonMesh(mesh, "mesh");
  // while (!viewer->wasStopped())
  // {
  //   viewer->spinOnce();
  // }
  // return 0;

//     pcl::PCLPointCloud2 point_cloud2;
//     pcl::PLYReader reader;
//     reader.read("/media/cyy/CYY_DISK/others/bags/gps/out.bag_points.ply", point_cloud2);
//     pcl::PointCloud<pcl::PointXYZI> point_cloud;
//     pcl::fromPCLPointCloud2( point_cloud2, point_cloud);
//     std::cout << "start reading!" <<std::endl;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     if (pcl::io::loadPLYFile<pcl::PointXYZ>("/media/cyy/CYY_DISK/others/bags/gps/out.bag_points.ply", *cloud) == -1) {       
// //         PCL_ERROR("Couldnot read file.\n");
//         std::cout << "read error!" <<std::endl;
// //         pause();
//         return(-1);
//     }
//     
//     std::cout << "read done!" <<std::endl;
//     pcl::visualization::CloudViewer viewer("Cloud Viewer");
//     viewer.showCloud(cloud);
//     
//     pause();
//     pcl::PCDWriter writer;
//     writer.writeASCII("/media/cyy/CYY_DISK/others/bags/gps/data.pcd", point_cloud2);  
//     return 0;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "temp_test");
//   ros::NodeHandle n;
//   client_ = n.serviceClient<cartographer_ros_msgs::SetSwitch>("/mark_localization/set_mark_switch");
//   int i =0;
//   ros::Rate rate(5);
//   while(ros::ok())
//   {
//     rate.sleep();
//     cartographer_ros_msgs::SetSwitch srv;
//     if(i % 2 == 0)
//     {
//       srv.request.type = "LaserScanOdom";
//       srv.request.flag = true;
//       client_.call(srv);
//       srv.request.type = "StripLocalization";
//       srv.request.flag = false;
//       client_.call(srv);
//     }
//     else
//     {
//       srv.request.type = "LaserScanOdom";
//       srv.request.flag = false;
//       client_.call(srv);
//       srv.request.type = "StripLocalization";
//       srv.request.flag = true;
//       client_.call(srv);
//     }
//     cout <<"i:" <<i <<endl;
//     i++;
//     
//     ros::spinOnce();
//   }
//   return 1;
//   
// }
