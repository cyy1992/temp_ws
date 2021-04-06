#include "detect_tag.h"
#include "iostream"
#include <unistd.h>
#include <dirent.h>
// #include <AprilTags/TagDetection.h>
// #include <AprilTags/Tag36h11.h>
#include <chrono>
using namespace std;
using namespace cv;
// #define DM_CODE_LINE_NUM 2
// #define DM_TOTAL_NUM 4
vector<string> getFiles(string cate_dir)
{
  vector<string> files;//存放文件名
  DIR *dir;
  struct dirent *ptr;
  char base[1000];
  
  if ((dir=opendir(cate_dir.c_str())) == NULL)
  {
    perror("Open dir error...");
    exit(1);
  }
  
  while ((ptr=readdir(dir)) != NULL)
  {
    if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
      continue;
    else if(ptr->d_type == 8)    ///file
      //printf("d_name:%s/%s\n",basePath,ptr->d_name);
      files.push_back(ptr->d_name);
    else if(ptr->d_type == 10)    ///link file
      //printf("d_name:%s/%s\n",basePath,ptr->d_name);
      continue;
    else if(ptr->d_type == 4)    ///dir
    {
      files.push_back(ptr->d_name);
    }
  }
  closedir(dir);
  return files;
}
std::map<int,std::vector<string>> getFineNames(const std::string& filepath)
{
  ifstream fi;
  fi.open(filepath+ "/frame_idx.txt");
  std::map<int,std::vector<string>> result;
  if(fi.is_open())
  {
    while(fi.good())
    {
      string image_name;
      fi >> image_name;
      int camera_id;
      fi >> camera_id;
      result[camera_id].push_back(image_name);
      cout << image_name << ", " << camera_id <<endl;
    }
  }
  cout << result.size()<<endl;
  fi.close();
  return result;
}
detect_tag::detect_tag() : connected_(false)
{
  intrinsic_ = Mat::zeros(3, 3, CV_64FC1);
  distortion_ = Mat::zeros(4, 1, CV_64FC1);
  distortion_zero_= Mat::zeros(4, 1, CV_64FC1);
      dm_code_size_ = 19.7556;
    dm_grid_size_ = 28.2222;

//   std::map<int,std::vector<string> > id_fileNames = getFineNames("/home/cyy/shareFiles/_0/");
  std::cout << "camera_topic: " << camera_topic_ << std::endl;
  std::cout << "tag_size: " << dm_code_size_ << std::endl;
  
//   const char* famname = "tag36h11"; // Tag family to use
 
//   string home_path = "/home/cyy/shareFiles/_0/resources/raw_images/";
//   cout  << "home_path: " <<home_path <<endl;
  intrinsic_.at<double>(0, 0) = 478.9138 ;
  intrinsic_.at<double>(1, 1) = 478.8228 ;
  intrinsic_.at<double>(0, 2) = 322.3874;
  intrinsic_.at<double>(1, 2) = 229.5064;
  intrinsic_.at<double>(2, 2) = 1.0;
  distortion_.at<double>(0) = -0.3586;
  distortion_.at<double>(1) = 0.1301;
  distortion_.at<double>(2) = 0;
  distortion_.at<double>(3) = 0;
  cout <<"\033[34m" << "intrinsic_: " <<  intrinsic_ <<endl;
  cout <<"\033[34m" <<"distortion_: " <<  distortion_ <<endl;
  cout <<"\033[34m" << "dm_code_size_: " <<dm_code_size_ <<endl;
  
  Mat extrinsicR_ = (Mat_<double>(3, 3) << 0.9998 ,   0.0002 ,  -0.0176,
    -0.0000 ,   0.9999  ,  0.0124,
    0.0176 ,  -0.0124  ,  0.9998  );
  Mat extrinsicT_ = (Mat_<double>(3, 1) << -33.730323291904966,-55.01432000164834,86.10216083784688);
  projWidth_ = 472;
  projHeight_ = 300;
  objWidth_ = projWidth_ *1.0 / 2.78;
  objHeight_ = projHeight_*1.0  / 2.78;
  width_ = 752;
  height_ = 480;
  
  p_ip_ = new ImageProjector(intrinsic_, distortion_, extrinsicR_, extrinsicT_,
                  width_, height_, projWidth_, projHeight_, objWidth_,
                  objHeight_);
  name_id_ = 0;
  std::string tag_name = "tag36h11";
  const char *famname = tag_name.c_str();
  tf_ = apriltag_family_create(famname);
  if (!tf_)
  {
//     cout << ("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
    exit(-1);
  }
//   tf_->black_border = 1; // Set tag family border size

  td_ = apriltag_detector_create();
  td_->quad_decimate = 2.0;  // Decimate input image by this factor
  td_->quad_sigma = 0.0;     // Apply low-pass blur to input
  td_->nthreads = 1;         // Use this many CPU threads
  td_->debug = 0;            // Enable debugging output (slow)
  td_->refine_edges = 1;     // Spend more time trying to align edges of tags
  td_->qtp.min_white_black_diff = 5;
  apriltag_detector_add_family(td_, tf_);
  
  Mat img = imread(std::string(getenv("HOME")) +"/image.png",CV_LOAD_IMAGE_GRAYSCALE);
  cout << img.cols << ", " <<img.rows <<endl;
//   if(img.empty())
//     cout <<"***************" <<endl;
  
  
  while(1)
  {
    std::chrono::steady_clock::time_point t_point_ = std::chrono::steady_clock::now();
    imgCb(img);
//     if(waitKey(2) == 27)
//       break;
    std::chrono::duration<double> time_elapse_ = std::chrono::steady_clock::now() - t_point_;
    cout <<  std::chrono::duration_cast<std::chrono::nanoseconds>(time_elapse_).count() * 1.0e-9 <<endl;
  }
  
}

detect_tag::~detect_tag()
{
//   if (initialized_)
//   {
    apriltag_detector_destroy(td_);
    apriltag_family_destroy(tf_);
//   }
}

void detect_tag::imgCb(const Mat& srcImg)
{
  
  detection_cnt_++;
  
//   Mat img,img1;
//   if (srcImg.type() != CV_8UC1)
//     cvtColor(srcImg, img1, CV_BGR2GRAY);
//   else
//     img1 = srcImg.clone();
  
  // 	imshow("2",img);waitKey(0);
  bool result = tagToPose(srcImg);
}

bool detect_tag::tagToPose(const Mat& srcImg)
{
  // 	cv::Mat img(srcImg.cols, srcImg.rows, CV_8UC3);
//   cv::Mat img = srcImg.clone();
//   Mat show(img.cols,img.rows,CV_8UC3);
//   Mat save_show(img.cols,img.rows,CV_8UC3);
//   cvtColor(img, show, CV_GRAY2RGB);
//   cvtColor(img, save_show, CV_GRAY2RGB);
//   imshow("temp",srcImg); waitKey(0);
  Mat projImg;
  p_ip_->projectImage(srcImg, projImg);
//   imshow("temp2",projImg);
//   waitKey(0);
//   std::vector<AprilTags::TagDetection>  tag_detections = tag_detector_.extractTags(projImg);
//    cout << "tag_detections size: " << tag_detections.size() <<endl;
//   double add_w = 0;
//   Scalar cl =  cv::mean(save_show);
  int total_detections = 0;
  image_u8_t im8 = {.width = projImg.cols,
                    .height = projImg.rows,
                    .stride = projImg.cols,
                    .buf = projImg.data };
//   image_u8_t im8 = cv2im8(projImg);
  zarray_t *detections = apriltag_detector_detect(td_, &im8);
  total_detections += zarray_size(detections);
//   map<int,vector<Point2f>> pts_img;
//   map<int,vector<Point3f>> pts_obj;
//   set<int> ids;
  
  map<int, vector<Point2f>> pt_img;
  map<int, vector<Point3f>> pt_obj;
  std::set<int> landmark_ids;
  double bias = dm_code_size_ + 0.5 * (dm_grid_size_ - dm_code_size_);
  for (unsigned int i = 0; i < total_detections; i++)
  {
    apriltag_detection_t* det;  // c[2], p[4][2](clockwise)
    zarray_get(detections, i, &det);
//     int id = detect_tmp.id;
    
    int total_num_ = 4;
    int code_line_num_ = 2;
    int landmark_id = det->id / total_num_;
//     cout << "id: " << det->id <<endl;
    if(det->id >= 44032)
    {
      
      total_num_ = 4;
      code_line_num_ = 2;
      landmark_id = det->id / total_num_;
      landmark_id = landmark_id * total_num_;
    }
//     cout << detect_tmp.p
    landmark_ids.insert(landmark_id);
    int id = det->id % total_num_;
    int row = id / code_line_num_;
    int col = id % code_line_num_;
    
    
    Point2d project_point;
    Point2d distort_point;
    for (int j = 0; j < 4; j++)
    {
//       Point2d distort_point, project_point;
      int ic = (j == 1 || j == 2) ? 1 : 0;
      int ir = (j == 0 || j == 1) ? 1 : 0;
      project_point = Point2d(det->p[j][0] , det->p[j][1]);
      p_ip_->project2distortPoint(project_point, &distort_point);
      pt_img[landmark_id].push_back(distort_point);
      Point3f obj_p(col * dm_grid_size_ + ic * dm_code_size_ - bias, 
                               row * dm_grid_size_ + ir *dm_code_size_ - bias, 0);
      pt_obj[landmark_id].push_back(obj_p);
//       cout <<j<<", " << ic <<", " <<ir <<endl;
//       cout << distort_point.x <<", " << distort_point.y <<endl;
//       cout << obj_p.x <<", " << obj_p.y <<endl;
    }
  }
  
//   Mat show_img ;
//   cvtColor(srcImg, show_img, CV_GRAY2BGR);
  
  Mat rvec,tvec;
  for(auto landmark_id: landmark_ids)
  {
    solvePnP(pt_obj[landmark_id], pt_img[landmark_id], intrinsic_, distortion_, rvec, tvec);
    // check reprojectError
    float reprojectError = 0;
    vector<Point2f> reprojectPixeles;
    projectPoints(pt_obj[landmark_id], rvec, tvec, intrinsic_, distortion_,
                  reprojectPixeles);
    for (int i = 0; i < reprojectPixeles.size(); i++)
    {
      float dx = reprojectPixeles[i].x - pt_img[landmark_id][i].x;
      float dy = reprojectPixeles[i].y - pt_img[landmark_id][i].y;
      reprojectError += sqrt(dx * dx + dy * dy);
    }
    reprojectError /= reprojectPixeles.size();

    if (reprojectError > 3.)
    {
      
//       return false;
    }
//     cout << "Loc failed:ReprojectError " << reprojectError <<endl;
//       for(auto point : reprojectPixeles)
//       {
//         circle(show_img, point, 3, Scalar(255,255,0), -1);
//       }
//       for(auto point: pt_img[landmark_id])
//       {
//         circle(show_img, point, 3, Scalar(0,0,255) , -1);
//       }
  }
  
//   imshow("temp23", show_img);
//   waitKey(0);
//   cout << tvec <<endl;
    
  return true;
}
int main(int argc, char** argv)
{
  cout <<"start" <<endl;
  detect_tag detectTag;
  cout  << "done" <<endl;
  return 1;
}
