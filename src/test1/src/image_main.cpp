#include <imreg_fmt/image_registration.h>
#include <imreg_fmt/awesome_clock.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ImageProjector.h>
using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    cv::Mat im01, im11;

//     if (argc < 3)
//     {
//         std::cout << "specify two image files" << std::endl;
//         return 1;
//     }
    im01 = cv::imread(std::string(getenv("HOME")) + "/image.png", CV_LOAD_IMAGE_GRAYSCALE);
    im11 = cv::imread(std::string(getenv("HOME")) + "/image.png", CV_LOAD_IMAGE_GRAYSCALE);

    //im0(Rect(0,150,300,150)).setTo(255);

//    cv::Mat translateMatrix = (cv::Mat_<float>(2, 3) << 1.0, 0.0, -28, 0.0, 1.0, -200);
//    cv::warpAffine(im0, im1, translateMatrix, im0.size());

//    Mat im00 = Mat::zeros(im0.rows*2, im0.cols*2, CV_8UC1);
//    im0.copyTo(im00(Rect(im0.cols/2,im0.rows/2,im0.cols,im0.rows)));
//    Mat im11 = Mat::zeros(im1.rows*2, im1.cols*2, CV_8UC1);
//    im1.copyTo(im11(Rect(im1.cols/2,im1.rows/2,im1.cols,im1.rows)));

//    cv::imshow("im0", im0);
//    cv::imshow("im1", im1);
    Mat extrinsicR_ = (Mat_<double>(3, 3) << 0.9998 ,   0.0002 ,  -0.0176,
    -0.0000 ,   0.9999  ,  0.0124,
    0.0176 ,  -0.0124  ,  0.9998  );
  Mat extrinsicT_ = (Mat_<double>(3, 1) << -33.730323291904966,-55.01432000164834,86.10216083784688);
  int projWidth_ = 300;
  int projHeight_ = 300;
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
  Mat im0;
  p_ip_->projectImage(im01, im0);
while(1){
    AwesomeClock t;
    t.start();
    Mat im1;
    p_ip_->projectImage(im11, im1);
    ImageRegistration image_registration(im0.rows, im0.cols);
    image_registration.initialize(im0);

    

    // x, y, rotation, scale
    cv::Mat cur2ref_R, cur2ref_T;
    cur2ref_R = cv::Mat::eye(3, 3, CV_64FC1);

    //cur2ref_R = (cv::Mat_<double>(3,3) << -1, 0, 0, 0, -1, 0, 0, 0, 1);

    cur2ref_T = cv::Mat::zeros(3, 1, CV_64FC1);
    double overlap = 0., double_check = -1.;
    image_registration.registerImage(im1, cur2ref_R, cur2ref_T, overlap, double_check, true, false);

    double time = t.stop();
    std::cout << "Time used: " << time*1e3 << " ms.\n";
}
    return 0;
}
