// #include <iostream>
// #include <string>
// #include <mutex>
// #include <thread>
// #include <chrono>
// #include <opencv2/opencv.hpp>
// #include <Eigen/Eigen>
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/point_cloud2_iterator.h>
// using namespace std;
// using namespace cv;
// RNG g_rng(12345);//毛大大的博客里看到的生成随机数，用于生成随机颜色
// bool is_rect = false;//不可避免地还是要定义几个全局变量，伤心
// Point start_position=Point(-1,-1);
// Point end_position= Point(-1, -1);
// Mat last_img, point_img_;
// Mat bool_img_,last_bool_img;
// vector<int> rect_manu;
// ros::Subscriber pointcloud_sub_;
// vector<Point3f> scan_points_;
// std::mutex m_lock_;
// Mat3b canvas;
// string buttonText("START COMPUTE!"); 
// Rect button;
// 
// float resolution_;
// void cvFitPlane(const CvMat* points, float* plane)
// {
//   int nrows = points->rows;
//   int ncols = points->cols;
//   int type = points->type;
//   CvMat* centroid = cvCreateMat(1, ncols, type);
//   cvSet(centroid, cvScalar(0));
//   for (int c = 0; c<ncols; c++){
//     for (int r = 0; r < nrows; r++)
//     {
//       centroid->data.fl[c] += points->data.fl[ncols*r + c];
//     }
//     centroid->data.fl[c] /= nrows;
//   }
//   // Subtract geometric centroid from each point.  
//   CvMat* points2 = cvCreateMat(nrows, ncols, type);
//   for (int r = 0; r<nrows; r++)
//   for (int c = 0; c<ncols; c++)
//     points2->data.fl[ncols*r + c] = points->data.fl[ncols*r + c] - centroid->data.fl[c];
//   // Evaluate SVD of covariance matrix.  
//   CvMat* A = cvCreateMat(ncols, ncols, type);
//   CvMat* W = cvCreateMat(ncols, ncols, type);
//   CvMat* V = cvCreateMat(ncols, ncols, type);
//   cvGEMM(points2, points, 1, NULL, 0, A, CV_GEMM_A_T);
//   cvSVD(A, W, NULL, V, CV_SVD_V_T);
//   // Assign plane coefficients by singular vector corresponding to smallest singular value.  
//   plane[ncols] = 0;
//   for (int c = 0; c<ncols; c++){
//     plane[c] = V->data.fl[ncols*(ncols - 1) + c];
//     plane[ncols] += plane[c] * centroid->data.fl[c];
//   }
//   // Release allocated resources.  
//   cvReleaseMat(&centroid);
//   cvReleaseMat(&points2);
//   cvReleaseMat(&A);
//   cvReleaseMat(&W);
//   cvReleaseMat(&V);
// }
// bool isPointValid(const Point3f& point)
// {
//   if(bool_img_.cols == 1)// no bool image
//     return false;
//   int x,y;
//   x = point.x / resolution_ +0.5 + bool_img_.cols / 2;
//   y = point.y / resolution_ +0.5 + bool_img_.rows / 2;
//   
//   if(x > bool_img_.cols-1 || y > bool_img_.rows-1 || x <0  || y <0)
//   {
// //     cout  << "This point is out of img: " <<max_box_[0]<< "," << max_box_[1]<< "," <<point(0)<< "," << point(1)<< "," << img_x << "," << img_y <<endl;
//     return false;
//   }
//   if(bool_img_.at<uchar>(y,x) ==0){
//     point_img_.at<uchar>(y,x) = 0;
//     return true;
//   }
//   return false;
// }
// void computeRollPitch()
// {
//   
//   point_img_ = Mat(Size(point_img_.cols, point_img_.rows),CV_8UC1,Scalar(255));
//   vector<Point3f> worldsInCamera_total;
//   for(auto& point: scan_points_)
//   {
//     if(isPointValid(point))
//       worldsInCamera_total.push_back(point);
//   }
//   if(worldsInCamera_total.size() == 0)
//     return;
//   CvMat* points_mat = cvCreateMat(worldsInCamera_total.size(), 3, CV_32FC1); 
//   for (unsigned int i = 0; i < worldsInCamera_total.size(); ++i)
//   {
//     points_mat->data.fl[i * 3 + 0] = worldsInCamera_total[i].x;
//     points_mat->data.fl[i * 3 + 1] = worldsInCamera_total[i].y;
//     points_mat->data.fl[i * 3 + 2] = worldsInCamera_total[i].z;
//   }
//   float plane12[4] = { 0 };
// 
//   cvFitPlane(points_mat, plane12); 
// 
//   float A = plane12[0];
//   float B = plane12[1];
//   float C = plane12[2];
//   float D = plane12[3];
//   float sum_squar = A * A + B * B + C * C;
// 
//   double sum;
//   Point3d p0(0, 0, D / C), p1(10, 10, (D - 10 * A - 10 * B) / C);
//   Point3d vx = p1 - p0;
//   sum = sqrt(vx.x*vx.x + vx.y*vx.y + vx.z*vx.z);
//   vx = vx * (1. / sum);
//   Point3d vz(A, B, C);
//   sum = sqrt(vz.x*vz.x + vz.y*vz.y + vz.z*vz.z);
//   Point3d vy;
//   vy = vz.cross(vx);
//   sum = sqrt(vy.x*vy.x + vy.y*vy.y + vy.z*vy.z);
//   vy = vy * (1. / sum);
// 
//   
//   Mat new2camera_rot = (Mat_<double>(3, 3) <<
//     vx.x, vy.x, vz.x,
//     vx.y, vy.y, vz.y,
//     vx.z, vy.z, vz.z);
//   Eigen::Matrix3f R_new2cam;
//   R_new2cam << vx.x, vy.x, vz.x,
//     vx.y, vy.y, vz.y,
//     vx.z, vy.z, vz.z;
//   cout << "new2camera_rot: " << R_new2cam.transpose().eulerAngles(2,1,0).transpose() <<endl;
//   
//   
//   
//   Point3d p00(0, 10, (D-10*B) / C), p10(10, 10, (D - 10 * A - 10 * B) / C);
//   Point3d vx0 = p10 - p00;
//   sum = sqrt(vx0.x*vx0.x + vx0.y*vx0.y + vx0.z*vx0.z);
//   vx0 = vx0 * (1. / sum);
//   Point3d vz0(A, B, C);
//   sum = sqrt(vz0.x*vz0.x + vz0.y*vz0.y + vz0.z*vz0.z);
//   Point3d vy0;
//   vy0 = vz0.cross(vx0);
//   sum = sqrt(vy0.x*vy0.x + vy0.y*vy0.y + vy0.z*vy0.z);
//   vy0 = vy0 * (1. / sum);
// 
//   Eigen::Matrix3f R_new2cam1;
//   R_new2cam1 << vx0.x, vy0.x, vz0.x,
//     vx0.y, vy0.y, vz0.y,
//     vx0.z, vy0.z, vz0.z;
//   cout << "new2camera_rot: " << R_new2cam1.transpose().eulerAngles(2,1,0).transpose() <<endl;
//   
// }
// void on_mouse(int event, int x, int y, int flags, void *ustc)
// //event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号    
// {
//   Mat& image = *(cv::Mat*) ustc;//这样就可以传递Mat信息了，很机智
//   char temp[16];
//   switch (event) {
//     case CV_EVENT_LBUTTONDOWN://按下左键
//     {
//       last_img = image.clone();
//       last_bool_img = bool_img_.clone();
// //       sprintf(temp, "(%d,%d)", x, y);
// //       putText(image, temp, Point(x, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0, 255));
//       is_rect = true;
//       start_position= Point(x, y);
//       
//       if (button.contains(Point(x, y)))
//       {
//         cout << "Start compute roll pitch!" << endl;
//         rectangle(canvas, button, Scalar(0,0,255), 2);
//         computeRollPitch();
//       }
//     } break;
//     case CV_EVENT_FLAG_RBUTTON:
//     {
//       bool_img_ = Mat(Size(bool_img_.cols, bool_img_.rows),CV_8UC1,Scalar(255));
//       break;
//     }
//     case CV_EVENT_MOUSEMOVE://移动鼠标
//     {
//       end_position = Point(x, y);
//       if (is_rect)
//       { }
//     }break;
//     case CV_EVENT_LBUTTONUP:
//     {
//       is_rect = false;
// //       sprintf(temp, "(%d,%d)", x, y);
// //       putText(image, temp, Point(x, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0, 255));
//       //调用函数进行绘制
//       cv::rectangle(image,start_position, end_position, cv::Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255)));
//       rectangle(canvas, button, Scalar(200, 200, 200), 2);
//       for(int i = start_position.x; i < end_position.x; i++)
//       {
//         for(int j = start_position.y; j < end_position.y; j++)
//         {
//           bool_img_.at<uchar>(j,i) = 0;
//         }
//       }
//       
//     }break;
//     
//     case CV_EVENT_MBUTTONDOWN:
//     {
//       image = last_img;
//       bool_img_ = last_bool_img;
//     }break;
//   }
// }
// 
// void drawBlack(vector<int> rect_manu)
// {
//   for(auto a : rect_manu)
//   {
//     cout << a <<endl;
//   }
//   
//   for(int i = rect_manu[0]; i < rect_manu[2]; i++)
//   {
//     for(int j = rect_manu[1]; j < rect_manu[3]; j++)
//     {
//       bool_img_.at<uchar>(j,i) = 0;
//     }
//   }
// }
// void handlePointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
// {
//   sensor_msgs::PointCloud2 point_cloud;
//   point_cloud = (*msg);
//   int range_size = point_cloud.width * point_cloud.height;
//   if(range_size > 0)
//   {
//     m_lock_.lock();
//     vector<Point3f>().swap(scan_points_);
//     sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud, "x");
//     sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud, "y");
//     sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud, "z");
// //      sensor_msgs::PointCloud points;
// //      points.header = msg->header;
//     for(int i = 0; i < range_size; i++)
//     {
//       Point3f point;
//       point.x = *iter_x;
//       point.y = *iter_y;
//       point.z = *iter_z;
//       ++iter_x;
//       ++iter_y;
//       ++iter_z;      
//       scan_points_.push_back(point);
//     }
//     m_lock_.unlock();
//   }
// }
// void show_image()
// {
//   Mat img(600,600,CV_8UC3,Scalar(255,255,255));
//   button = Rect(0,600,img.cols, 50);
//   namedWindow("img");
//   bool_img_ = Mat(Size(img.cols, img.rows),CV_8UC1,Scalar(255)); 
//   point_img_ = Mat(Size(img.cols, img.rows),CV_8UC1,Scalar(255)); 
//     // The canvas
//   
//   
//   canvas = Mat3b(img.rows + button.height, img.cols, Vec3b(0,0,0));
// 
//   // Draw the button
//   canvas(button) = Vec3b(200,200,200);
//   putText(canvas(button), buttonText, Point(button.width*0.35, button.height*0.65), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0));
//   img.copyTo(canvas(Rect(0, 0, img.cols, img.rows)));
// //   imshow("temp",canvas);
// //   waitKey(0);
//   Mat temp;
//   img.copyTo(temp);
//   while(ros::ok())
//   {
//     Mat show_img = canvas.clone();
//     
//     m_lock_.lock();
//     for(auto point: scan_points_)
//     {
//       int x,y;
//       x = point.x / resolution_ +0.5 + bool_img_.cols / 2;
//       y = point.y / resolution_ +0.5 + bool_img_.rows / 2;
//       if( x > 0&& x < 600 && y > 0 && y < 600 )
//       {
//         show_img.at<Vec3b>(y,x) = Vec3b(0, 0, 255);
//       }
//     }
//     m_lock_.unlock();
//     
//     setMouseCallback("img", on_mouse, (void*)&show_img); 
//     
//     if(is_rect)
//       rectangle(show_img, start_position, end_position, cv::Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255)));
//     putText(show_img,"("+std::to_string(end_position.x)+","+std::to_string(end_position.y)+")" , end_position, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0, 255));
// //     imshow("temp",temp);
//     imshow("bool_img", bool_img_);
//     imshow("img",show_img);
//     imshow("point",point_img_);
//     waitKey(3);
// //     std::this_thread::sleep_for(std::chrono::microseconds(10));
//   }
// }
// int main(int argc, char **argv) {
//   ros::init(argc,argv,"calib_roll_pitch");
//   ros::NodeHandle nh;
//   pointcloud_sub_ = nh.subscribe("/scan_pointcloud",10,handlePointCloud);
//   resolution_ = 0.05;
//   
//   
//   std::thread mthread(show_image);
//   mthread.detach();
//   ros::spin();
// //   std::cout << "Hello, world!" << std::endl;
// //   string map_name = argv[1];
// //   double display_ratio = atof(argv[2]);
// //   string img_path = std::string(getenv("HOME")) + "/map/" +  map_name +"/";
// //   
// //   Mat img = imread(img_path + "map.png");
// // //     cout << (img.type() == CV_8UC3 )<<endl;
// //   Mat img_resize,result;
// //   Size origin_size = Size(img.cols, img.rows);
// //   cout << "origin_size: " << img.cols <<", " << img.rows <<endl;
// // //     threshold(img, result, 30, 200.0, CV_THRESH_BINARY);
// //   Size ResImgSiz = Size(img.cols*display_ratio, img.rows*display_ratio);
// //   resize(img, img_resize,ResImgSiz);
// //   last_img = img_resize;
// //   Mat temp1;
// //   int key_value;
// //   Mat origin_bool_image = imread(img_path + "bool_image.png",CV_8UC1);
// //   if(origin_bool_image.empty())
// //   {
// //     bool_img =  Mat(ResImgSiz,CV_8UC1,Scalar(255)); 
// //   }
// //   else{
// //     cout <<( origin_bool_image.type() == CV_8UC1) <<endl;
// //     resize(origin_bool_image, bool_img, ResImgSiz);
// //   }
// // //     bool_img =  Mat(ResImgSiz,CV_8UC1,Scalar(255)); 
// //   last_bool_img = bool_img.clone();
// //   while ( key_value != 's' && key_value != 'S') {
// //     img_resize.copyTo(temp1);
// //     namedWindow("img");
// //     setMouseCallback("img", on_mouse, (void*)&img_resize);  
// //     if(is_rect)
// //       rectangle(temp1, start_position, end_position, cv::Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255)));
// // //       putText(temp1,"("+std::to_string(end_position.x)+","+std::to_string(end_position.y)+")" , end_position, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0, 255));
// //     imshow("img",temp1);
// //     imshow("bool_img", bool_img);
// //     key_value = waitKey(30);
// //     if(key_value == 32)
// //     {
// //       rect_manu.clear();
// //       cout << "please input rect num: " <<endl;
// //       int b;
// //       while(cin>>b)
// //       {
// //         rect_manu.push_back(b*display_ratio);
// //         cout << b <<endl;
// //         if (cin.get() == '\n') 
// //           break;
// //       }
// //       drawBlack(rect_manu);
// //     }
// //   }
// //   
// //   resize(bool_img, result,origin_size);
// //   imwrite(img_path + "bool_image.png",result);
// //   cout << "result size: " << result.cols <<", " << result.rows <<endl;
// //   cout << "write "+ img_path+"bool_image.png done! " <<endl;
// //     imshow("1",img_resize);
// //     waitKey(0);
//   return 0;
// }
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;

// static void help()
// {
//     cout << "\nThis program demonstrates kmeans clustering.\n"
//             "It generates an image with random points, then assigns a random number of cluster\n"
//             "centers and uses kmeans to move those cluster centers to their representitive location\n"
//             "Call\n"
//             "./kmeans\n" << endl;
// }

int main( int /*argc*/, char** /*argv*/ )
{
  const int MAX_CLUSTERS = 5;
  Scalar colorTab[] =     //因为最多只有5类，所以最多也就给5个颜色
  {
    Scalar(0, 0, 255),
    Scalar(0,255,0),
    Scalar(255,100,100),
    Scalar(255,0,255),
    Scalar(0,255,255)
  };

  Mat img(500, 500, CV_8UC3);
  RNG rng(12345);

  for(;;)
  {
    int k, clusterCount = 1;
    int i, sampleCount = 100;
    Mat points(100, 1, CV_32FC2), labels;  
    clusterCount = MIN(clusterCount, sampleCount);
    Mat centers(clusterCount, 1, points.type());    
    
  /* generate random sample from multigaussian distribution */
    for( k = 0; k < clusterCount; k++ ) //产生随机数
    {
      Point center;
      center.x = rng.uniform(0, img.cols);
      center.y = rng.uniform(0, img.rows);
      Mat pointChunk = points.rowRange(k*sampleCount/clusterCount,
                                        k == clusterCount - 1 ? sampleCount :
                                        (k+1)*sampleCount/clusterCount);   
      
      //最后一个类的样本数不一定是平分的，
      //剩下的一份都给最后一类
      //每一类都是同样的方差，只是均值不同而已
      rng.fill(pointChunk, CV_RAND_NORMAL, Scalar(center.x, center.y), Scalar(img.cols*0.05, img.rows*0.05));
    }
    points.at<Vec2f>(0)[0] = 100;
    points.at<Vec2f>(0)[1] = 200;
    points.at<Vec2f>(99)[0] = 101;
    points.at<Vec2f>(99)[1] = 201;
    cout << points <<endl;
     //因为要聚类，所以先随机打乱points里面的点，注意points和pointChunk是共用数据的。
    randShuffle(points, 1, &rng);  
    
    kmeans(points, clusterCount, labels,
          TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
          3, KMEANS_PP_CENTERS, centers);  //聚类3次，取结果最好的那次，聚类的初始化采用PP特定的随机算法。

    img = Scalar::all(0);

    for( i = 0; i < sampleCount; i++ )
    {
      int clusterIdx = labels.at<int>(i);
      Point ipt = points.at<Point2f>(i);
      cv::circle( img, ipt, 2, colorTab[clusterIdx], CV_FILLED, CV_AA );
    }

    imshow("clusters", img);
    char key = (char)waitKey();     //无限等待
    if( key == 27 || key == 'q' || key == 'Q' ) // 'ESC'
      break;
  }

return 0;
}
