#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "imreg_fmt/image_registration.h"
#include <ImageProjector.h>
#include <thread>
#include<malloc.h>
#include <boost/ptr_container/ptr_vector.hpp> 

#include "iostream"
using namespace std;
using namespace cv;
class test{
public:
  test(){}
  ~test(){cout << "1111!" <<endl;}
  void compute(/*const cv::Mat& image*/);
  
  void setImg(const cv::Mat& img){im0 = img;};
private:
  
  boost::shared_ptr<ImageRegistration> p_ir_;
  cv::Mat im0;
  
  boost::mutex db_mutex_;
  
};

void test::compute(/*const cv::Mat& image*/)
{
  boost::mutex::scoped_lock lock(db_mutex_);
//   cout << "1111!" <<endl;
  if(p_ir_ == nullptr)
  {  
//    
//     p_ir_.reset(new ImageRegistration(im0.rows, im0.cols));
//     p_ir_->initialize(im0);
  }
  Mat im1;
  im0.copyTo(im1);
//   cout << "2222!" <<endl;
//   cv::Mat cur2ref_R, cur2ref_T;
//   cur2ref_R = cv::Mat::eye(3, 3, CV_64FC1);
// 
//   //cur2ref_R = (cv::Mat_<double>(3,3) << -1, 0, 0, 0, -1, 0, 0, 0, 1);
// 
//   cur2ref_T = cv::Mat::zeros(3, 1, CV_64FC1);
//   double overlap = 0., double_check = -1.;
//   p_ir_->registerImage(im0, cur2ref_R, cur2ref_T, overlap, double_check, true, false);
//   cout << "333!" <<endl;
}

int main()
{
  Mat extrinsicR_ = (Mat_<double>(3, 3) << 0.9998 ,   0.0002 ,  -0.0176,
    -0.0000 ,   0.9999  ,  0.0124,
    0.0176 ,  -0.0124  ,  0.9998  );
  Mat extrinsicT_ = (Mat_<double>(3, 1) << -33.730323291904966,-55.01432000164834,86.10216083784688);
  int projWidth_ = 1200;
  int projHeight_ = 1200;
  double objWidth_ = projWidth_ *1.0 / 2.78;
  double objHeight_ = projHeight_*1.0  / 2.78;
  int width_ = 752;
  double height_ = 480;
  Mat intrinsic_ = Mat::zeros(3, 3, CV_64FC1);
  Mat distortion_ = Mat::zeros(4, 1, CV_64FC1);
  intrinsic_.at<double>(0, 0) = 478.9138 ;
  intrinsic_.at<double>(1, 1) = 478.8228 ;
  intrinsic_.at<double>(0, 2) = 322.3874;
  intrinsic_.at<double>(1, 2) = 229.5064;
  intrinsic_.at<double>(2, 2) = 1.0;
  distortion_.at<double>(0) = -0.3586;
  distortion_.at<double>(1) = 0.1301;
  distortion_.at<double>(2) = 0;
  distortion_.at<double>(3) = 0;
  ImageProjector* p_ip_ = new ImageProjector(intrinsic_, distortion_, extrinsicR_, extrinsicT_,
                  width_, height_, projWidth_, projHeight_, objWidth_,
                  objHeight_);
  Mat im01 = cv::imread(std::string(getenv("HOME")) + "/image.png", CV_LOAD_IMAGE_GRAYSCALE);
  Mat im0,im1;
  p_ip_->projectImage(im01, im0);
  p_ip_->projectImage(im01, im1);
  int jj = 0;
  while(waitKey(20) !='32'){
    shared_ptr<test> t(new test);

    t->setImg(im0);
    int ii =0;
//     cout << "11111!" <<endl;
    while(ii++ < 10)
    {
      boost::ptr_vector<boost::thread> p_threads;
      for(int i =0;i <10; i++) 
      {
        p_threads.push_back(new boost::thread(&test::compute,t.get()));
//         t->compute();
      }
      
      for(int i =0; i < p_threads.size(); i++) 
      {
        if(p_threads[i].joinable())
          p_threads[i].join();
        else
          cout << "22222222!" <<endl;
      }
    }
    cout << jj++  <<", " << t.use_count()<<endl;
    malloc_trim(0);
  }
  return 1;
}