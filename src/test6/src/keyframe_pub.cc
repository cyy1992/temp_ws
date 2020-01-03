#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>

#include <glog/logging.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visual_msgs/Keyframe.h>

#include <opencv2/opencv.hpp>
#include "AprilTags/TagDetection.h"
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

using namespace cv;
using namespace std;

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
    sensor_msgs::Image> MySyncPolicy4;

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
    sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy5;

AprilTags::TagDetector* tag_detector_;
double tags_size_[6];
double grids_size_[6];
double dm_code_size_ = 0.2;
class KeyframePub
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KeyframePub(ros::NodeHandle& n);
  ~KeyframePub();

  void imgSyncCb4(const sensor_msgs::ImageConstPtr& img0_msg,
                 const sensor_msgs::ImageConstPtr& img1_msg,
                 const sensor_msgs::ImageConstPtr& img2_msg,
                 const sensor_msgs::ImageConstPtr& img3_msg);

  void imgSyncCb5(const sensor_msgs::ImageConstPtr& img0_msg,
                 const sensor_msgs::ImageConstPtr& img1_msg,
                 const sensor_msgs::ImageConstPtr& img2_msg,
                 const sensor_msgs::ImageConstPtr& img3_msg,
                  const sensor_msgs::ImageConstPtr& img4_msg);

private:
  void getIntrinsicFromYaml(const std::string& path);
  void distortPoints(const std::vector<cv::Point2f> & src, std::vector<cv::Point2f> & dst, 
                     const cv::Mat & cameraMatrix, const cv::Mat & distorsionMatrix);
  void analyzeImages(const std::vector<Mat>& imgs, visual_msgs::Keyframe& frame_info);
  ros::NodeHandle nh_;
  ::tf2_ros::Buffer tf_buffer_;
  ::tf2_ros::TransformListener tf_listener_;
  message_filters::Subscriber<sensor_msgs::Image> img0_sub_;
  message_filters::Subscriber<sensor_msgs::Image> img1_sub_;
  message_filters::Subscriber<sensor_msgs::Image> img2_sub_;
  message_filters::Subscriber<sensor_msgs::Image> img3_sub_;
  message_filters::Subscriber<sensor_msgs::Image> img4_sub_;
  ros::Publisher keyframe_pub_;

  Eigen::Isometry3d lastSensor2odom_;
  int cnt_;
  
  vector<cv::Mat> intrinsic_, distortion_;
};

KeyframePub::KeyframePub(ros::NodeHandle& n) : nh_(n), tf_listener_(tf_buffer_)
{
  cnt_ = 0;
  lastSensor2odom_.setIdentity();
  getIntrinsicFromYaml("/home/cyy/projects/test6/test6/config/intrinsic_params.yaml"); //yaml path
//  img0_sub_.subscribe(nh_, "/stereo_g2/left/resize/image_raw", 10);
//  img1_sub_.subscribe(nh_, "/stereo_g2/right/resize/image_raw", 10);
//  img2_sub_.subscribe(nh_, "/stereo_g3/left/resize/image_raw", 10);
//  img3_sub_.subscribe(nh_, "/stereo_g3/right/resize/image_raw", 10);
//  message_filters::Synchronizer<MySyncPolicy4>* sync;
//  MySyncPolicy4 policy(10);
//  policy.setMaxIntervalDuration(ros::Duration(0, 7e6));
//  sync = new message_filters::Synchronizer<MySyncPolicy4>(
//      MySyncPolicy4(policy), img0_sub_, img1_sub_, img2_sub_, img3_sub_);
//  sync->registerCallback(
//      boost::bind(&KeyframePub::imgSyncCb4, this, _1, _2, _3, _4));

  img0_sub_.subscribe(nh_, "/stereo/left/resize/image_raw", 10);
  img1_sub_.subscribe(nh_, "/stereo/right/resize/image_raw", 10);
  img2_sub_.subscribe(nh_, "/around/left/resize/image_raw", 10);
  img3_sub_.subscribe(nh_, "/around/right/resize/image_raw", 10);
  img4_sub_.subscribe(nh_, "/around/rear/resize//image_raw", 10);
  message_filters::Synchronizer<MySyncPolicy5>* sync;
  MySyncPolicy5 policy(10);
  policy.setMaxIntervalDuration(ros::Duration(0, 7e6));
  sync = new message_filters::Synchronizer<MySyncPolicy5>(
      MySyncPolicy5(policy), img0_sub_, img1_sub_, img2_sub_, img3_sub_, img4_sub_);
  sync->registerCallback(
      boost::bind(&KeyframePub::imgSyncCb5, this, _1, _2, _3, _4, _5));

  keyframe_pub_ =
      nh_.advertise<visual_msgs::Keyframe>("/visual_odom/key_frame", 3, true);
}

KeyframePub::~KeyframePub() {}

void KeyframePub::getIntrinsicFromYaml(const string& path)
{
  FileStorage fs(path, FileStorage::READ);
  const int camera_num = fs["camera_number"];
  intrinsic_.resize(camera_num);
  distortion_.resize(camera_num);

  for(int k=0; k < camera_num; k++)
  {
    vector<uchar> intrinsic;
    /*FileNode intrinsic =*/ 
    intrinsic_[k] = cv::Mat::zeros(3,3,CV_8UC1);
    distortion_[k] = cv::Mat::zeros(4,1,CV_8UC1);
    fs["intrinsic_" + to_string(k+1)] >>intrinsic_[k];
    fs["distortion_" + to_string(k+1)] >>distortion_[k];
    cout << "intrinsic_" << k+1 <<": " << endl;
    cout << intrinsic_[k]  <<endl;
    cout << "distortion_" << k+1 <<": " << endl;
    cout << distortion_[k]  <<endl;
  }
}

void KeyframePub::distortPoints(const std::vector<cv::Point2f> & src, std::vector<cv::Point2f> & dst, 
                     const cv::Mat & cameraMatrix, const cv::Mat & distorsionMatrix)
{
  dst.clear();
  double fx = cameraMatrix.at<double>(0,0);
  double fy = cameraMatrix.at<double>(1,1);
  double ux = cameraMatrix.at<double>(0,2);
  double uy = cameraMatrix.at<double>(1,2);

  double k1 = distorsionMatrix.at<double>(0, 0);
  double k2 = distorsionMatrix.at<double>(0, 1);
  double p1 = distorsionMatrix.at<double>(0, 2);
  double p2 = distorsionMatrix.at<double>(0, 3);
  double k3 = distorsionMatrix.at<double>(0, 4);
  //BOOST_FOREACH(const cv::Point2d &p, src)
  for (unsigned int i = 0; i < src.size(); i++)
  {
    const cv::Point2f &p = src[i];
    double x = p.x;
    double y = p.y;
    double xCorrected, yCorrected;
    //Step 1 : correct distorsion
    {     
      double r2 = x*x + y*y;
      //radial distorsion
      xCorrected = x * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
      yCorrected = y * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

      //tangential distorsion
      //The "Learning OpenCV" book is wrong here !!!
      //False equations from the "Learning OpenCv" book
      //xCorrected = xCorrected + (2. * p1 * y + p2 * (r2 + 2. * x * x)); 
      //yCorrected = yCorrected + (p1 * (r2 + 2. * y * y) + 2. * p2 * x);
      //Correct formulae found at : http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
      xCorrected = xCorrected + (2. * p1 * x * y + p2 * (r2 + 2. * x * x));
      yCorrected = yCorrected + (p1 * (r2 + 2. * y * y) + 2. * p2 * x * y);
    }
    //Step 2 : ideal coordinates => actual coordinates
    {
      xCorrected = xCorrected * fx + ux;
      yCorrected = yCorrected * fy + uy;
    }
    dst.push_back(cv::Point2f(xCorrected, yCorrected));
  }
}

void KeyframePub::imgSyncCb4(const sensor_msgs::ImageConstPtr& img0_msg,
                            const sensor_msgs::ImageConstPtr& img1_msg,
                            const sensor_msgs::ImageConstPtr& img2_msg,
                            const sensor_msgs::ImageConstPtr& img3_msg)
{
  ros::Time stamp = img0_msg->header.stamp;

  if (stamp != img1_msg->header.stamp || stamp != img2_msg->header.stamp ||
      stamp != img3_msg->header.stamp)
  {
    ROS_WARN("Sync Image Required!");
    return;
  }

  geometry_msgs::TransformStamped sensor2odom_stamped;
  while(1)
  {
    try
    {
      sensor2odom_stamped =
          tf_buffer_.lookupTransform("odom", "imu_link", stamp);
    }
    catch (tf2::TransformException& ex)
    {
      //ROS_WARN_THROTTLE(1, "sensor2odom_stamped: %s", ex.what());
      if(ros::Time::now() - stamp > ros::Duration(0.5))
      {
        ROS_WARN("sensor2odom_stamped: %s", ex.what());
        return;
      }
      else
      {
        usleep(1000);
        continue;
      }
    }
    break;
  }

  Eigen::Isometry3d sensor2odom_eigen, sensor2lastSensor;
  tf::transformMsgToEigen(sensor2odom_stamped.transform, sensor2odom_eigen);
  sensor2lastSensor = lastSensor2odom_.inverse() * sensor2odom_eigen;
  double dt = sensor2lastSensor.translation().norm();
  double dr = Eigen::AngleAxisd(sensor2lastSensor.rotation()).angle();

//  LOG(INFO) << "lastSensor2odom_:\n" << lastSensor2odom_.matrix();
//  LOG(INFO) << "sensor2odom_eigen:\n" << sensor2odom_eigen.matrix();

  if(dt < 0.5 && dr < 10.*M_PI/180.)
    return;

//  LOG(WARNING) << "lastSensor2odom_:\n" << lastSensor2odom_.matrix();
//  LOG(WARNING) << "sensor2odom_eigen:\n" << sensor2odom_eigen.matrix();

  cout << "============= Keyframe " << cnt_ << "==========" <<endl;

  visual_msgs::Keyframe kf;
  kf.header.seq = cnt_++;
  kf.header.stamp = stamp;
  kf.raw_images.push_back(*img0_msg);
  kf.raw_images.push_back(*img1_msg);
  kf.raw_images.push_back(*img2_msg);
  kf.raw_images.push_back(*img3_msg);
  kf.sensor2odom = sensor2odom_stamped.transform;
  keyframe_pub_.publish(kf);

  lastSensor2odom_ = sensor2odom_eigen;
  cout << "====================================" <<endl;
}

void KeyframePub::analyzeImages(const std::vector<Mat>& imgs, visual_msgs::Keyframe& frame_info)
{
  for(int i = 0; i < imgs.size();i ++)
  {
    const Mat& img = imgs[i];
    Mat img1;
    if (img.type() != CV_8UC1)
      cvtColor(img, img1, CV_BGR2GRAY);
    else
      img1 = img.clone();
    std::vector<AprilTags::TagDetection>  
      tag_detections = tag_detector_->extractTags(img1);
    if(tag_detections.empty())
      continue;
    
    visual_msgs::ImageInfos image_info;
    image_info.camera_id = i;
    
    map<int,vector<Point2f>> pts_img;
    map<int,vector<Point3f>> pts_obj;
    set<int> ids;
    for (unsigned int i = 0; i < tag_detections.size(); i++)
    {
      AprilTags::TagDetection detect_tmp = tag_detections[i];
      int id = detect_tmp.id;
      if(id < 30)
      {
        int num1 = id /5;
        int num2 = id % 5;
        cout << "id: " << id <<" \t";
        cout << "tags_size_: " << tags_size_[num1] <<" \t";
        double tmp_half_tag_size = 0.5 * tags_size_[num1];
        double tmp_grid_size = grids_size_[num1];
        double tmp_x,tmp_y;
        if(num2 == 2){
          tmp_x = 0;tmp_y = 0;
        }
        else if(num2 == 0)
        {
          tmp_x = -tmp_grid_size;tmp_y = tmp_grid_size;
        }
        else if(num2 == 1)
        {
          tmp_x = tmp_grid_size;tmp_y = tmp_grid_size;
        }
        else if(num2 == 3)
        {
          tmp_x = -tmp_grid_size;tmp_y = -tmp_grid_size;
        }
        else if(num2 == 4)
        {
          tmp_x = tmp_grid_size;tmp_y = -tmp_grid_size;
        }
        cout << "num1: " << num1  <<endl;
        vector<Point2f> undistort_points,project_points;
        undistort_points.push_back(Point2f(detect_tmp.p[0].first, detect_tmp.p[0].second));
//         pts_img[num1].push_back(project_point);
        pts_obj[num1].push_back(Point3f(tmp_x - tmp_half_tag_size ,tmp_y + tmp_half_tag_size, 0));

        undistort_points.push_back(Point2f(detect_tmp.p[1].first, detect_tmp.p[1].second));
//         pts_img[num1].push_back(project_point);
        pts_obj[num1].push_back(Point3f(tmp_x - tmp_half_tag_size ,tmp_y - tmp_half_tag_size, 0));

        undistort_points.push_back(Point2f(detect_tmp.p[2].first, detect_tmp.p[2].second));
//         pts_img[num1].push_back(project_point);
        pts_obj[num1].push_back(Point3f(tmp_x + tmp_half_tag_size ,tmp_y - tmp_half_tag_size, 0));

        undistort_points.push_back(Point2f(detect_tmp.p[3].first, detect_tmp.p[3].second));
//         pts_img[num1].push_back(project_point);
        pts_obj[num1].push_back(Point3f(tmp_x + tmp_half_tag_size ,tmp_y + tmp_half_tag_size, 0));
        
        distortPoints(undistort_points,project_points,intrinsic_[i],distortion_[i]);
        pts_img[num1].insert(pts_img[num1].end(),project_points.begin(),project_points.end());
        ids.insert(num1);
      }
      else{
        vector<Point2f> pt_img,undistort_points;
        vector<Point3f> pt_obj;
        undistort_points.push_back(Point2d(detect_tmp.p[0].first, detect_tmp.p[0].second));
        pt_obj.push_back(Point3f(0,0, 0));

        undistort_points.push_back(Point2d(detect_tmp.p[1].first, detect_tmp.p[1].second));
        pt_obj.push_back(Point3f(dm_code_size_+0, 0, 0));

        undistort_points.push_back(Point2d(detect_tmp.p[2].first, detect_tmp.p[2].second));
        pt_obj.push_back(Point3f(dm_code_size_+0, dm_code_size_+0, 0));

        undistort_points.push_back(Point2d(detect_tmp.p[3].first, detect_tmp.p[3].second));
        pt_obj.push_back(Point3f(0, dm_code_size_, 0));
        
        distortPoints(undistort_points,pt_img,intrinsic_[i],distortion_[i]);
        pts_img[id] = pt_img;
        pts_obj[id] = pt_obj;
        ids.insert(id);
      }
    }

    assert(ids.size() == pts_img.size());
    cout << "ids.size(): " <<ids.size() <<endl;
    for (std::set<int>::iterator it = ids.begin(); it != ids.end(); ++it)
    {

      vector<Point2f> pt_img;
      vector<Point3f> pt_obj;
      int tmp_id = *it;
      pt_img = pts_img[tmp_id];
      pt_obj = pts_obj[tmp_id];
      vector<Point2f>::iterator it1=pt_img.begin();
      vector<Point3f>::iterator it2=pt_obj.begin();
      visual_msgs::TagInfos tag_info;
      tag_info.tag_id = tmp_id;
      for(;it1!=pt_img.end();it1++,it2++)
      {
        //u v x y z
        visual_msgs::Point p;
        p.u = it1->x; p.v = it1->y;
        p.x = it2->x; p.y = it2->y; p.z = it2->z;
        tag_info.points.push_back(p);
      }
      image_info.tag_infos.push_back(tag_info);
    }
    frame_info.image_infos.push_back(image_info);
  }
  
}

void KeyframePub::imgSyncCb5(const sensor_msgs::ImageConstPtr& img0_msg,
                            const sensor_msgs::ImageConstPtr& img1_msg,
                            const sensor_msgs::ImageConstPtr& img2_msg,
                            const sensor_msgs::ImageConstPtr& img3_msg,
                             const sensor_msgs::ImageConstPtr& img4_msg)
{
  ros::Time stamp = img0_msg->header.stamp;

  if (stamp != img1_msg->header.stamp || stamp != img2_msg->header.stamp ||
      stamp != img3_msg->header.stamp || stamp != img4_msg->header.stamp)
  {
    ROS_WARN("Sync Image Required!");
    return;
  }

  geometry_msgs::TransformStamped sensor2odom_stamped;
//   while(1)
//   {
//     try
//     {
//       sensor2odom_stamped =
//           tf_buffer_.lookupTransform("odom", "imu_link", stamp);
//     }
//     catch (tf2::TransformException& ex)
//     {
//       //ROS_WARN_THROTTLE(1, "sensor2odom_stamped: %s", ex.what());
//       if(ros::Time::now() - stamp > ros::Duration(0.5))
//       {
//         ROS_WARN("sensor2odom_stamped: %s", ex.what());
//         return;
//       }
//       else
//       {
//         usleep(1000);
//         continue;
//       }
//     }
//     break;
//   }

  Eigen::Isometry3d sensor2odom_eigen, sensor2lastSensor;
  tf::transformMsgToEigen(sensor2odom_stamped.transform, sensor2odom_eigen);
  sensor2lastSensor = lastSensor2odom_.inverse() * sensor2odom_eigen;
  double dt = sensor2lastSensor.translation().norm();
  double dr = Eigen::AngleAxisd(sensor2lastSensor.rotation()).angle();

//  LOG(INFO) << "lastSensor2odom_:\n" << lastSensor2odom_.matrix();
//  LOG(INFO) << "sensor2odom_eigen:\n" << sensor2odom_eigen.matrix();

   if(dt < 0.5 && dr < 10.*M_PI/180.)
     return;

//  LOG(WARNING) << "lastSensor2odom_:\n" << lastSensor2odom_.matrix();
//  LOG(WARNING) << "sensor2odom_eigen:\n" << sensor2odom_eigen.matrix();
  
  cout << "============= Keyframe " << cnt_ << "==========" <<endl;
  std::vector<Mat> images;
  images.push_back(cv_bridge::toCvShare(img0_msg)->image);
  images.push_back(cv_bridge::toCvShare(img1_msg)->image);
  images.push_back(cv_bridge::toCvShare(img2_msg)->image);
  images.push_back(cv_bridge::toCvShare(img3_msg)->image);
  images.push_back(cv_bridge::toCvShare(img4_msg)->image);
//  imshow("img0",images[0] );
//  waitKey(2);
  visual_msgs::Keyframe kf;
  kf.header.seq = cnt_++;
  kf.header.stamp = stamp;
   kf.raw_images.push_back(*img0_msg);
   kf.raw_images.push_back(*img1_msg);
   kf.raw_images.push_back(*img2_msg);
   kf.raw_images.push_back(*img3_msg);
   kf.raw_images.push_back(*img4_msg);
  kf.sensor2odom = sensor2odom_stamped.transform;
  analyzeImages(images,kf);
  keyframe_pub_.publish(kf);

  lastSensor2odom_ = sensor2odom_eigen;
  cout << "====================================" <<endl;
}

int main(int argc, char** argv)
{
   google::InitGoogleLogging(argv[0]);
   google::InstallFailureSignalHandler();
   FLAGS_alsologtostderr = true;
   FLAGS_colorlogtostderr = true;
  
  ros::init(argc, argv, "KeyframePub");
  ros::NodeHandle nh;
  tag_detector_ = new AprilTags::TagDetector(AprilTags::tagCodes36h11);
  {
    tags_size_[0] = 0.066664;tags_size_[1] = 0.066664;
    tags_size_[2] = 0.111;tags_size_[3] = 0.111;
    tags_size_[4] = 0.222;tags_size_[5] = 0.222;
    
    grids_size_[0] = 0.079164;grids_size_[1] = 0.079164;
    grids_size_[2] = 0.12775;grids_size_[3] = 0.12775;
    grids_size_[4] = 0.2555;grids_size_[5] = 0.2555;
  }
  
  KeyframePub kp(nh);
  ROS_INFO("KeyframePub start!");

  //  ros::MultiThreadedSpinner spinner(2);
  //  spinner.spin();
  ros::spin();
  delete tag_detector_;
  return 0;
}
