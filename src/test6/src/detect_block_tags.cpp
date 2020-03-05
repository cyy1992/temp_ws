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

#include "detect_block_tags.h"
#include "Eigen/Eigen"
using namespace std;
using namespace cv;
detectBlockTags::detectBlockTags(const ros::NodeHandle& nh, 
                                 const std::string& cam_type):
    nh_(nh),camera_type_(cam_type)
{
  string cam_prefix = "/jzhw/calib/camera/" + camera_type_ +"/";
  
  intrinsic_ = Mat::eye(3, 3, CV_64FC1);
  ros::param::get(cam_prefix+"fx",intrinsic_.at<double>(0, 0));
  ros::param::get(cam_prefix+"fy",intrinsic_.at<double>(1, 1));
  ros::param::get(cam_prefix+"cx",intrinsic_.at<double>(0, 2));
  ros::param::get(cam_prefix+"cy",intrinsic_.at<double>(1, 2));
  distortion_ = Mat::zeros(4, 1, CV_64FC1);
  ros::param::get(cam_prefix+"k1",distortion_.at<double>(0));
  ros::param::get(cam_prefix+"k2",distortion_.at<double>(1));
  ros::param::get(cam_prefix+"k3",distortion_.at<double>(2));
  ros::param::get(cam_prefix+"k4",distortion_.at<double>(3));
  
  ros::param::get("/tag_codec_type",tag_type_);
  ros::param::get("~TAGS_PER_LINE", tags_per_line_);
  ros::param::get("~GRID_SIZE", grid_size_);
  ros::param::get("~TAG_SIZE", tag_size_);
  
  show_color_image_ = false;
  ros::param::get("~show_image",show_color_image_);
  
  perception_pub_ =
      nh_.advertise<geometry_msgs::Pose>("/block_tags_pose", 3);
  
  string camera_topic;
  ros::param::get("/algo/property/cam_5_arm_docking_topic", camera_topic);
  img_sub_ = nh_.subscribe<sensor_msgs::Image>(
    camera_topic, 1, &detectBlockTags::PerceptionCB, this);
  
  if (tag_type_ == 0)
  {
    tf_ = tag36h11_create();
  }
  else if (tag_type_ == 1)
  {
    tf_ = tag36h10_create();
  }
  else
  {
    exit(-1);
  }
  
  td_ = apriltag_detector_create();
  apriltag_detector_add_family(td_, tf_);
  td_->quad_decimate = 2.0;  // Decimate input image by this factor
  td_->quad_sigma = 0.0;     // Apply low-pass blur to input
  td_->nthreads = 4;         // Use this many CPU threads
  td_->debug = 0;            // Enable debugging output (slow)
  td_->refine_edges = 1;     // Spend more time trying to align edges of tags

  cout << "intrinsic_: " <<intrinsic_ <<endl;
  cout << "distortion_: " <<distortion_ <<endl;
  cout << "tag_type: " <<tag_type_ <<endl;
  cout << "camera_topic: " <<camera_topic <<endl;
}

detectBlockTags::~detectBlockTags()
{

}

void detectBlockTags::PerceptionCB(
  const boost::shared_ptr< const sensor_msgs::Image >& sensor_msg)
{
  int tag_id;
  int detect_tags = 0;

  if (sensor_msg->data.size() != sensor_msg->height * sensor_msg->step)
  {
    cout << "img_msg->data.size() != img_msg->height*img_msg->step" <<endl;
    return;
  }

  cv_bridge::CvImagePtr p_img = cv_bridge::toCvCopy(sensor_msg);
  
  Mat srcImg;
  if (p_img->image.type() != CV_8UC1)
    cvtColor(p_img->image, srcImg, CV_BGR2GRAY);
  else
    srcImg = p_img->image;
 
  tagToPose2(srcImg, tag_id, detect_tags);
}

bool detectBlockTags::tagToPose2(const cv::Mat& srcImg, 
                                 int& tag_id, int& detect_tags)
{
  if (!init_param_)
  {
    cv::initUndistortRectifyMap(intrinsic_, distortion_, cv::noArray(),
                                intrinsic_, srcImg.size(), CV_16SC2,
                                undist_map1_, undist_map2_);
    mask_img_ = Mat::zeros(srcImg.size(), CV_8UC1);
    init_param_ = true;
  }
  Mat rot_mat, rvec, tvec;
  Mat projImg;

  cv::remap(srcImg, projImg, undist_map1_, undist_map2_, CV_INTER_LINEAR);

  projImg.setTo(0, mask_img_);

  int total_detections = 0;
  image_u8_t im8 = {.width = projImg.cols,
                    .height = projImg.rows,
                    .stride = projImg.cols,
                    .buf = projImg.data };
  zarray_t* detections = apriltag_detector_detect(td_, &im8);
  total_detections += zarray_size(detections);
  if (total_detections == 0)
  {
    mask_img_.setTo(0);
    apriltag_detections_destroy(detections);

    sensor_msgs::ImagePtr p_image_msg;
    p_image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", projImg).toImageMsg();
//     undistort_img_pub_.publish(p_image_msg);
    return false;
  }

  vector<Point2f> pt_img;
  vector<Point3f> pt_obj;
//   int true_id1 = 0, true_id2 = 0;
  
  for (int i = 0; i < total_detections; i++)
  {
    apriltag_detection_t* det;  // c[2], p[4][2](clockwise)
    zarray_get(detections, i, &det);
    int id = det->id;

    if (tag_type_ == 0)
    {
      if (100 < id && id < 587)
      {
//         true_id1 = id;
        id = 12;
      }
      else if (1800 <= id && id <= 2318)
      {
        if (1825 <= id && id <= 2071)
        {
//           true_id1 = id - 1825;
          id = 1807;
        }
        else if (2072 <= id && id <= 2318)
        {
//           true_id2 = id - 2072;
          id = 1812;
        }
        id = id - 1800;
      }
    }
    else if (tag_type_ == 2)
    {
//       true_id1 = id / 4;
      id = id % 4;
    }
    else
    {
//       true_id1 = 0;
//       true_id2 = 0;
    }

    int row = id / tags_per_line_;
    int col = id % tags_per_line_;

    // counter-clockwise around the quad, starting at -1,-1.
    for (int i = 0; i < 4; i++)
    {
      Point2d project_point;
      int ic = (i == 1 || i == 2) ? 1 : 0;
      int ir = (i == 0 || i == 1) ? 1 : 0;

      project_point = Point2d(det->p[i][0], det->p[i][1]);
      pt_img.push_back(project_point);
      pt_obj.push_back(Point3f(col * grid_size_ + ic * tag_size_,
                               row * grid_size_ + ir * tag_size_, 0));
    }
  }

  detect_tags = pt_img.size() / 4;

  // world in camera
  solvePnP(pt_obj, pt_img, intrinsic_, cv::noArray(), rvec, tvec);

  float reprojectError = 0;
  vector<Point2f> reprojectPixeles;
  projectPoints(pt_obj, rvec, tvec, intrinsic_, cv::noArray(),
                reprojectPixeles);
  for (size_t i = 0; i < reprojectPixeles.size(); i++)
  {
    float dx = reprojectPixeles[i].x - pt_img[i].x;
    float dy = reprojectPixeles[i].y - pt_img[i].y;
    reprojectError += sqrt(dx * dx + dy * dy);
  }
  reprojectError /= reprojectPixeles.size();

  if (true)
  {
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 2;
    int thickness = 2;
    int baseline;

    Mat color;
    cvtColor(projImg, color, CV_GRAY2RGB);
    for (int i = 0; i < total_detections; i++)
    {
      apriltag_detection_t* det;  // c[2], p[4][2](clockwise)
      zarray_get(detections, i, &det);
      int id = det->id;
      line(color, Point2d(det->p[0][0], det->p[0][1]),
           Point2d(det->p[1][0], det->p[1][1]), CV_RGB(255, 0, 0), 2);
      line(color, Point2d(det->p[1][0], det->p[1][1]),
           Point2d(det->p[2][0], det->p[2][1]), CV_RGB(0, 255, 0), 2);
      line(color, Point2d(det->p[2][0], det->p[2][1]),
           Point2d(det->p[3][0], det->p[3][1]), CV_RGB(0, 0, 255), 2);
      line(color, Point2d(det->p[3][0], det->p[3][1]),
           Point2d(det->p[0][0], det->p[0][1]), CV_RGB(255, 255, 255), 2);

      string id_string = std::to_string(id);
      cv::Size text_size =
        cv::getTextSize(id_string, font_face, font_scale, thickness, &baseline);

      cv::Point center;
      center.x = (det->p[0][0] + det->p[2][0]) / 2 - text_size.width / 2;
      center.y = (det->p[0][1] + det->p[2][1]) / 2 + text_size.height / 2;

      putText(color, id_string, center, CV_AA, 2, CV_RGB(255, 255, 0), 1);
    }
    if(show_color_image_)
    {
      imshow("color_img",color);
      waitKey(2);
    }
//     sensor_msgs::ImagePtr p_image_msg;
//     p_image_msg =
//       cv_bridge::CvImage(std_msgs::Header(), "rgb8", color).toImageMsg();
//     undistort_img_pub_.publish(p_image_msg);
  }

  apriltag_detections_destroy(detections);
  Rodrigues(rvec, rot_mat);
  
  cout <<"tag2cam_t: " << tvec.at<double>(0) <<", " <<tvec.at<double>(1)
    <<", " <<tvec.at<double>(2) << endl;
  Mat cam2world_tvec;
  cam2world_tvec = -rot_mat.t() * tvec;
  
  // camera in world
  Eigen::Matrix3d rot;
  rot(0, 0) = rot_mat.at<double>(0, 0);
  rot(0, 1) = rot_mat.at<double>(0, 1);
  rot(0, 2) = rot_mat.at<double>(0, 2);
  rot(1, 0) = rot_mat.at<double>(1, 0);
  rot(1, 1) = rot_mat.at<double>(1, 1);
  rot(1, 2) = rot_mat.at<double>(1, 2);
  rot(2, 0) = rot_mat.at<double>(2, 0);
  rot(2, 1) = rot_mat.at<double>(2, 1);
  rot(2, 2) = rot_mat.at<double>(2, 2);
  Eigen::Quaterniond q(rot.transpose());
  
  //pub camera2world
  geometry_msgs::Pose pose;
  pose.position.x = cam2world_tvec.at<double>(0);
  pose.position.y = cam2world_tvec.at<double>(1);
  pose.position.z = cam2world_tvec.at<double>(2);
  
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  perception_pub_.publish(pose);
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detectBlockTags");
  ros::NodeHandle n;
  detectBlockTags detect(n,"arm");
  ros::spin();
  return 1;
}