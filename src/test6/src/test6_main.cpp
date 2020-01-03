// #include <opencv2/core/core.hpp>
// #include<opencv2/opencv.hpp>
// #include<iostream>
// #include <string>
// using namespace cv;
// using namespace std;
// int main()
// {
//     Mat image=imread("/home/cyy/map/2dtest3/bool_image.png");
//     if(image.empty())
//     {
//         cout<<"image is empty"<<endl;
//         return 0;
//     }
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

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <geometry_msgs/TwistStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// #include <cartographer/common/time.h>
using namespace std;
using namespace cv;
ros::Subscriber sub_;
ros::Publisher pub_;


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
//   cartographer::common::Time time(cartographer::common::Duration(636325483280538183)) ;
//   
//   ToRos(time);
//   cartographer::common::Time t2{cartographer::common::Duration(636325483280510890)};
//   ToRos(t2);
//   return 1;
//   
// }
void handleImage(const sensor_msgs::ImageConstPtr& msg )
{
  Mat origin_image = cv_bridge::toCvShare(msg, "mono8")->image;
  Mat resize_image;
  resize(origin_image,resize_image,Size(origin_image.cols/2,origin_image.rows/2));
  sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(msg->header, "mono8", resize_image).toImageMsg();
  pub_.publish(msg1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ImageResize");
  ros::NodeHandle n;
  string pub_img_topic;
  string sub_img_topic;
  if(!ros::param::get("~sub_img_topic",sub_img_topic))
  {
    cout << "Can not get params! " <<endl;
  }
  if(!ros::param::get("~pub_img_topic",pub_img_topic))
  {
    cout << "Can not get params! " <<endl;
  }
  sub_ = n.subscribe( sub_img_topic, 20, handleImage);
  pub_ = n.advertise<sensor_msgs::Image>(pub_img_topic,1);
  ros::spin();
  return 1;
}