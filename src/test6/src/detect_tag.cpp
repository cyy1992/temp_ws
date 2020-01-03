#include "detect_tag.h"
#include "iostream"
#include <unistd.h>
#include <dirent.h>
#include <AprilTags/TagDetection.h>
#include <AprilTags/Tag36h11.h>
#include <nav_msgs/Odometry.h>
using namespace std;
using namespace cv;
ros::Publisher pub_odom_;
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
			/*
		        memset(base,'\0',sizeof(base));
		        strcpy(base,basePath);
		        strcat(base,"/");
		        strcat(base,ptr->d_nSame);
		        readFileList(base);
			*/
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
detect_tag::detect_tag(ros::NodeHandle nh) : nh_(nh), connected_(false),tag_detector_(AprilTags::tagCodes36h11)
{
  intrinsic_ = Mat::zeros(3, 3, CV_64FC1);
  distortion_ = Mat::zeros(4, 1, CV_64FC1);
  distortion_zero_= Mat::zeros(4, 1, CV_64FC1);
  dm_code_size_ = 0.2;
  std::map<int,std::vector<string> > id_fileNames = getFineNames("/home/cyy/shareFiles/_0/");
  std::cout << "camera_topic: " << camera_topic_ << std::endl;
  std::cout << "tag_size: " << dm_code_size_ << std::endl;

  const char* famname = "tag36h11"; // Tag family to use
  
  {
    tags_size_[0] = 0.066664;tags_size_[1] = 0.066664;
    tags_size_[2] = 0.111;tags_size_[3] = 0.111;
    tags_size_[4] = 0.222;tags_size_[5] = 0.222;
    
    grids_size_[0] = 0.079164;grids_size_[1] = 0.079164;
    grids_size_[2] = 0.12775;grids_size_[3] = 0.12775;
    grids_size_[4] = 0.2555;grids_size_[5] = 0.2555;
  }

	string home_path = "/home/cyy/shareFiles/_0/resources/raw_images/";
  cout  << "home_path: " <<home_path <<endl;
  for(int i=0; i < 5; i++)
  {
    if(i == 0)
    {
      intrinsic_.at<double>(0, 0) = 605.2751 ;
      intrinsic_.at<double>(1, 1) = 605.8662 ;
      intrinsic_.at<double>(0, 2) = 367.8815;
      intrinsic_.at<double>(1, 2) = 285.1665;
      intrinsic_.at<double>(2, 2) = 1.0;
      distortion_.at<double>(0) = -0.0515;
      distortion_.at<double>(1) = 0.0691;
      distortion_.at<double>(2) = 0;
      distortion_.at<double>(3) = 0;
      cout <<"\033[34m" << "intrinsic_: " <<  intrinsic_ <<endl;
      cout <<"\033[34m" <<"distortion_: " <<  distortion_ <<endl;
      cout <<"\033[34m" << "dm_code_size_: " <<dm_code_size_ <<endl;
    }
    else if(i == 1)
    {
      intrinsic_.at<double>(0, 0) = 602.0540 ;
      intrinsic_.at<double>(1, 1) = 602.6372;
      intrinsic_.at<double>(0, 2) = 372.7750;
      intrinsic_.at<double>(1, 2) = 300.8095;
      intrinsic_.at<double>(2, 2) = 1.0;
      distortion_.at<double>(0) = -0.0538;
      distortion_.at<double>(1) = 0.0680;
      distortion_.at<double>(2) = 0;
      distortion_.at<double>(3) = 0;
      cout <<"\033[34m" <<"intrinsic_: " <<  intrinsic_ <<endl;
      cout <<"\033[34m" <<"distortion_: " <<  distortion_ <<endl;
      cout <<"\033[34m" << "dm_code_size_: " <<dm_code_size_ <<endl;
    }
    else if(i == 2)
    {
      intrinsic_.at<double>(0, 0) = 252.6114 ;
      intrinsic_.at<double>(1, 1) = 252.7636;
      intrinsic_.at<double>(0, 2) = 255.0063;
      intrinsic_.at<double>(1, 2) = 198.3917;
      intrinsic_.at<double>(2, 2) = 1.0;
      distortion_.at<double>(0) = -0.2007;
      distortion_.at<double>(1) = 0.0666;
      distortion_.at<double>(2) = 0;
      distortion_.at<double>(3) = 0;
      cout <<"\033[34m" <<"intrinsic_: " <<  intrinsic_ <<endl;
      cout <<"\033[34m" <<"distortion_: " <<  distortion_ <<endl;
      cout <<"\033[34m" << "dm_code_size_: " <<dm_code_size_ <<endl;
    }
    else if(i == 3)
    {
      intrinsic_.at<double>(0, 0) = 251.9695 ;
      intrinsic_.at<double>(1, 1) = 252.1665;
      intrinsic_.at<double>(0, 2) = 251.4842;
      intrinsic_.at<double>(1, 2) = 197.6835;
      intrinsic_.at<double>(2, 2) = 1.0;
      distortion_.at<double>(0) = -0.2049;
      distortion_.at<double>(1) = 0.0719;
      distortion_.at<double>(2) = 0;
      distortion_.at<double>(3) = 0;
      cout <<"\033[34m" <<"intrinsic_: " <<  intrinsic_ <<endl;
      cout <<"\033[34m" <<"distortion_: " <<  distortion_ <<endl;
      cout <<"\033[34m" << "dm_code_size_: " <<dm_code_size_ <<endl;
    }
    else if(i == 4)
    {
      intrinsic_.at<double>(0, 0) = 252.2614 ;
      intrinsic_.at<double>(1, 1) = 252.5372;
      intrinsic_.at<double>(0, 2) = 253.9566;
      intrinsic_.at<double>(1, 2) = 199.7416;
      intrinsic_.at<double>(2, 2) = 1.0;
      distortion_.at<double>(0) = -0.2001;
      distortion_.at<double>(1) = 0.0688;
      distortion_.at<double>(2) = 0;
      distortion_.at<double>(3) = 0;
      cout <<"\033[34m" <<"intrinsic_: " <<  intrinsic_ <<endl;
      cout <<"\033[34m" <<"distortion_: " <<  distortion_ <<endl;
      cout <<"\033[34m" << "dm_code_size_: " <<dm_code_size_ <<endl;
    }
    
    auto it = id_fileNames.find(i);
    cout << "hh : " <<(it == id_fileNames.end()) <<endl;
    std::vector<string> files = it->second;
    cout << "i: " <<i <<endl;
    for(auto& file:files)
    {
//       file = file.substr(0, file.rfind("."));
      cout << file <<endl;
      Mat img = imread(home_path + file + ".pgm",CV_LOAD_IMAGE_UNCHANGED);
      cout << "path: " << home_path + file + ".pgm" <<endl;
      if(img.empty())
      {
        
        cout << "empty!" <<endl;
        return;
      }
      
      imgCb(img,"/home/cyy/shareFiles/_0/resources/temp/"+file + ".txt");
    }
    
  }

  cout << "done!" <<endl;
	exit(1);
	name_id_ = 0;
	
}

detect_tag::~detect_tag()
{
}

void detect_tag::imgCb(const Mat& srcImg/*const sensor_msgs::ImageConstPtr& img_msg*/,const string& file_name)
{

  detection_cnt_++;

  Mat img,img1;
  if (srcImg.type() != CV_8UC1)
    cvtColor(srcImg, img1, CV_BGR2GRAY);
	else
		img1 = srcImg.clone();
	ROS_INFO_THROTTLE(10,"change to undistort!");
	cout <<"\033[34m" <<"intrinsic_: " <<  intrinsic_ <<endl;
      cout <<"\033[34m" <<"distortion_: " <<  distortion_ <<endl;
	undistort(img1,img,intrinsic_,distortion_);
// 	imshow("2",img);waitKey(0);
  bool result = tagToPose(img,file_name);
}

bool detect_tag::tagToPose(const Mat& srcImg,const string& file_name)
{
// 	cv::Mat img(srcImg.cols, srcImg.rows, CV_8UC3);
	cv::Mat img = srcImg.clone();
	Mat show(img.cols,img.rows,CV_8UC3);
	Mat save_show(img.cols,img.rows,CV_8UC3);
	cvtColor(img, show, CV_GRAY2RGB);
	cvtColor(img, save_show, CV_GRAY2RGB);
  
  std::vector<AprilTags::TagDetection>  tag_detections = tag_detector_.extractTags(srcImg);
 
	double add_w = 0;
	Scalar cl =  cv::mean(save_show);
	ofstream fi;
	fi.open(file_name);
  if(tag_detections.empty()){
    fi.close();
    return false;
  }
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
      Point2d project_point;
      project_point = Point2d(detect_tmp.p[0].first, detect_tmp.p[0].second);
      pts_img[num1].push_back(project_point);
      pts_obj[num1].push_back(Point3f(tmp_x - tmp_half_tag_size ,tmp_y + tmp_half_tag_size, 0));
//       cout << project_point.x << ", " << project_point.y  <<" \t";
//       cout << pts_obj[num1].back().x  << ", " << pts_obj[num1].back().y  <<" \t";
//       project_point = Point2d(det->p[1][0], det->p[1][1]);
      project_point = Point2d(detect_tmp.p[1].first, detect_tmp.p[1].second);
      pts_img[num1].push_back(project_point);
      pts_obj[num1].push_back(Point3f(tmp_x - tmp_half_tag_size ,tmp_y - tmp_half_tag_size, 0));
//       cout << project_point.x << ", " << project_point.y  <<" \t";
//       cout << pts_obj[num1].back().x  << ", " << pts_obj[num1].back().y  <<" \t";
      
//       project_point = Point2d(det->p[2][0], det->p[2][1]);
      project_point = Point2d(detect_tmp.p[2].first, detect_tmp.p[2].second);
      pts_img[num1].push_back(project_point);
      pts_obj[num1].push_back(Point3f(tmp_x + tmp_half_tag_size ,tmp_y - tmp_half_tag_size, 0));
//       cout << project_point.x << ", " << project_point.y  <<" \t";
//       cout << pts_obj[num1].back().x  << ", " << pts_obj[num1].back().y  <<" \t";
//       project_point = Point2d(det->p[3][0], det->p[3][1]);
      project_point = Point2d(detect_tmp.p[3].first, detect_tmp.p[3].second);
      pts_img[num1].push_back(project_point);
      pts_obj[num1].push_back(Point3f(tmp_x + tmp_half_tag_size ,tmp_y + tmp_half_tag_size, 0));
      ids.insert(num1);
//       cout << project_point.x << ", " << project_point.y  <<endl;;
//       cout << pts_obj[num1].back().x  << ", " << pts_obj[num1].back().y  <<" \t";
    }
    else{
      vector<Point2f> pt_img;
      vector<Point3f> pt_obj;
      Point2d project_point;
//       project_point = Point2d(det->p[0][0], det->p[0][1]);
      project_point = Point2d(detect_tmp.p[0].first, detect_tmp.p[0].second);
      pt_img.push_back(project_point);
      pt_obj.push_back(Point3f(0,0, 0));

//       project_point = Point2d(det->p[1][0], det->p[1][1]);
      project_point = Point2d(detect_tmp.p[1].first, detect_tmp.p[1].second);
      pt_img.push_back(project_point);
      pt_obj.push_back(Point3f(dm_code_size_+0, 0, 0));

//       project_point = Point2d(det->p[2][0], det->p[2][1]);
      project_point = Point2d(detect_tmp.p[2].first, detect_tmp.p[2].second);
      pt_img.push_back(project_point);
      pt_obj.push_back(Point3f(dm_code_size_+0, dm_code_size_+0, 0));

//       project_point = Point2d(det->p[3][0], det->p[3][1]);
      project_point = Point2d(detect_tmp.p[3].first, detect_tmp.p[3].second);
      pt_img.push_back(project_point);
      pt_obj.push_back(Point3f(0, dm_code_size_, 0));
      
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
    if(pt_img.size() == 0){
      fi.close();
        return false;
    }
		Mat  rvec, tvec;
		solvePnP(pt_obj, pt_img, intrinsic_, distortion_zero_, rvec, tvec);
		Mat R;
		Rodrigues(rvec,R);

    vector<Point2f>::iterator it1=pt_img.begin();
    vector<Point3f>::iterator it2=pt_obj.begin();
    fi << "id: " <<  tmp_id <<endl;
    fi << "tagPoints_uvxyz: " <<endl;
    fi << pt_img.size()<<endl;
    for(; it1!=pt_img.end();it1++,it2++)
    {
      fi << it1->x<<"  " << it1->y <<"  " << it2->x <<"  "<< it2->y <<"  "<< it2->z <<std::endl;
    }
		if(1){
			float reprojectError = 0;
			vector<Point2f> reprojectPixeles;
			projectPoints(pt_obj, rvec, tvec, intrinsic_, distortion_zero_,
									reprojectPixeles);
			
			for (int i = 0; i < reprojectPixeles.size(); i++)
			{
				float dx = reprojectPixeles[i].x - pt_img[i].x;
				float dy = reprojectPixeles[i].y - pt_img[i].y;
				reprojectError += sqrt(dx * dx + dy * dy);
			}
			cout << "tag2cam_t: " <<std::endl<< tvec.at<double>(0) <<"  " << tvec.at<double>(1) <<"  " <<tvec.at<double>(2) <<std::endl;
      cout << "tag2cam_r: " <<std::endl<< rvec.at<double>(0)<< "  " <<rvec.at<double>(1)<<"  " <<rvec.at<double>(2)<<std::endl;
			reprojectError /= reprojectPixeles.size();
			if(reprojectError > 3.)
			{
				cout << "reprojectPixeles is bigger than 3." <<endl;
			}
			else{
				
				fi << "tag2cam_t: " <<std::endl<< tvec.at<double>(0) <<"  " << tvec.at<double>(1) <<"  " <<tvec.at<double>(2) <<std::endl;
				fi << "tag2cam_r: " <<std::endl<< rvec.at<double>(0)<< "  " <<rvec.at<double>(1)<<"  " <<rvec.at<double>(2)<<std::endl;
				fi << "tag2cam_R: " <<std::endl<< R.at<double>(0,0) <<"  " <<R.at<double>(0,1) <<"  " <<R.at<double>(0,2) <<"  " <<std::endl
				<< R.at<double>(1,0) <<"  " <<R.at<double>(1,1) <<"  " <<R.at<double>(1,2) <<"  " <<std::endl
				<< R.at<double>(2,0) <<"  " <<R.at<double>(2,1) <<"  " <<R.at<double>(2,2) <<"  " <<std::endl<<std::endl<<std::endl;
			}
				cvtColor(img, show, CV_GRAY2RGB);
			int n = reprojectPixeles.size()/4;
			for (int i =0; i < n;i++)
			{
				line(show,reprojectPixeles[0+4*i],reprojectPixeles[1+4*i],Scalar(0,0,255),1,CV_AA);
				line(show,reprojectPixeles[1+4*i],reprojectPixeles[2+4*i],Scalar(0,255,255),1,CV_AA);
				line(show,reprojectPixeles[2+4*i],reprojectPixeles[3+4*i],Scalar(255,0,255),1,CV_AA);
				line(show,reprojectPixeles[3+4*i],reprojectPixeles[0+4*i],Scalar(0,255,0),1,CV_AA);
			}
			imshow("show",show);
			waitKey(2);
//       nav_msgs::Odometry laserOdometry;
//       laserOdometry.header.stamp = ros::Time::now();
//       laserOdometry.header.frame_id = "world";
//       Eigen::Vector3d t(tvec.at<double>(0),tvec.at<double>(1),tvec.at<double>(2));
//       Eigen::Matrix3d R_eigen;
//       R_eigen << R.at<double>(0,0) ,R.at<double>(0,1) ,R.at<double>(0,2),
//       R.at<double>(1,0) ,R.at<double>(1,1) ,R.at<double>(1,2),
//       R.at<double>(2,0) ,R.at<double>(2,1) ,R.at<double>(2,2);
//       Eigen::Quaterniond q(R_eigen);
//       Eigen::Quaterniond q_inv(R_eigen.transpose());
//       Eigen::Vector3d t_inv = -(q_inv * t);
//     laserOdometry.pose.pose.orientation.x = q_inv.x();
//     laserOdometry.pose.pose.orientation.y = q_inv.y();
//     laserOdometry.pose.pose.orientation.z = q_inv.z();
//     laserOdometry.pose.pose.orientation.w = q_inv.w();
//     laserOdometry.pose.pose.position.x = t_inv[0];
//     laserOdometry.pose.pose.position.y = t_inv[1];
//     laserOdometry.pose.pose.position.z = t_inv[2];
//     pub_odom_.publish(laserOdometry);
//     waitKey(2);
		}
	}
	fi.close();

  return true;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tag_detector");
  ros::NodeHandle nh;
  ROS_INFO("Tag detection start!");
//   pub_odom_ =
//     nh.advertise<nav_msgs::Odometry>("/visual_temp_odom", 5);
	detect_tag detectTag(nh);
  

#if 1
  ros::spin();
#else
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
#endif

  return 1;
}
