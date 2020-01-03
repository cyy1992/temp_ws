#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/opencv.hpp>
#include <stdio.h>  
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "params.h"
#include <memory>
using namespace cv;  
using namespace std;
cv::Mat org,dst,img,tmp;  
static Point pre_pt = Point(0,0);
static Point cur_pt = Point(0,0); 
static std::vector<vector<Point>> contours(1);
Point init_point,center_point;
bool initialized = false;
double img_ratio = 0.5;
inline Vec3f rotationMatrixToEulerAngles(Mat &R)
{

//     assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);
}
void on_mouse(int event,int x,int y,int flags,void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号  
{  
	char temp[16];  
	if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取初始坐标，并在图像上该点处划圆  
	{  
		if(!initialized)
		{
			initialized = true;
			init_point = Point(x,y);
			pre_pt = Point(x,y); 
			center_point = Point(0,0); 
// 			org.copyTo(img);
		}
		
		 
		circle(img,pre_pt,2,Scalar(255,0,0,0),CV_FILLED,CV_AA,0);
		imshow("img",img);
	}
	else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))//左键没有按下的情况下鼠标移动的处理函数  
	{  
			img.copyTo(tmp);//将img复制到临时图像tmp上，用于显示实时坐标  
			sprintf(temp,"(%d,%d)",x,y);  
			cur_pt = Point(x,y);  
			putText(tmp,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255));//只是实时显示鼠标移动的坐标  
			imshow("img",tmp);  
	}  
	else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))//左键按下时，鼠标移动，则在图像上划矩形  
	{  
			img.copyTo(tmp);  
			sprintf(temp,"(%d,%d)",x,y);  
			cur_pt = Point(x,y); 
			
			putText(tmp,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255));  
			line(tmp,pre_pt,cur_pt,Scalar(0,255,0,0),1,8,0);//在临时图像上实时显示鼠标拖动时形成的矩形  
			imshow("img",tmp);  
	}  
	else if (event == CV_EVENT_LBUTTONUP)//左键松开，将在图像上划矩形  
	{  
		//org.copyTo(img);  
		sprintf(temp,"(%d,%d)",pre_pt.x,pre_pt.y);  
// 		sprintf(temp,"(%d,%d)",x,y);  
		 
		//cv::putText(img,temp,pre_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255),1,8); 
		cur_pt = Point(x,y); 
		if(fabs(cur_pt.x - init_point.x) < 10 &&fabs(cur_pt.y - init_point.y) < 10&&contours[0].size() ==1)
			return;
		line(img,pre_pt,cur_pt,Scalar(0,255,0,0),1,8,0);
			contours[0].push_back(Point(cur_pt.x /img_ratio,cur_pt.y / img_ratio ));
			
			center_point.x = center_point.x + contours[0].back().x;
		 center_point.y = center_point.y + contours[0].back().y;
// 		cout << "cur_pt.x - init_point.x" <<cur_pt.x - init_point.x<<endl;
		if(fabs(cur_pt.x - init_point.x) < 10 &&fabs(cur_pt.y - init_point.y) < 10&& contours[0].size() > 1)
		{
// 			center_point.x = center_point.x + cur_pt.x;
// 			center_point.y = center_point.y + cur_pt.y;
			cout << "center: " << center_point.x <<"," << center_point.y <<endl;
			double size_contours = contours[0].size();
			double ratio = 1/size_contours;
			cv::Point2d p_tmp;
			p_tmp.x = ratio * center_point.x;
			p_tmp.y = ratio * center_point.y;
			center_point.x =  p_tmp.x;
			center_point.y =  p_tmp.y;
			line(img,pre_pt,init_point,Scalar(0,255,0,0),1,8,0);
			
			Mat roi = Mat::zeros(org.size(),CV_8U);
			Mat roi2 = Mat::zeros(org.size(),CV_8U);
			std::vector<vector<Point>> temp_contours = contours;
			cout << "center: " << center_point.x <<"," << center_point.y <<endl; 
			for(auto& it:temp_contours[0])
			{
				cout << "before: " << it.x <<"," << it.y <<endl; 
				it = it + 0.5*(it-center_point);
				cout << "after: " << it.x <<"," << it.y <<endl; 
			}
			drawContours(roi2,temp_contours,0,Scalar(255,255,255),-1);
			drawContours(roi,contours,0,Scalar(255,255,255),-1);
			Mat img1 = org.clone();
			img1.copyTo(dst,roi2);
			imshow("dst",dst);
			Mat result,result2;
			Mat kernel = getStructuringElement(cv::MORPH_RECT,Size(3,3));
			erode(dst,result,kernel);
			result.copyTo(result2,roi);
			
// 			
// 			imshow("fff",org);
			result2.copyTo(org,roi);
// 			imshow("img",img);  
			//截取矩形包围的图像，并保存到dst中  
// 			dst = org(Rect(min(cur_pt.x,pre_pt.x),min(cur_pt.y,pre_pt.y),width,height));  
			namedWindow("org");  
			imshow("org",org);  
			waitKey(0);  
		}
		else{
		 pre_pt = cur_pt;
		 cout << "center: " << center_point.x <<"," << center_point.y <<endl;
		}
		

	}  
} 
Eigen::Quaterniond euler2quat(const Eigen::Vector3d& euler) //yaw, pitch, roll
{
	Eigen::Matrix3d rot;
	rot = Eigen::AngleAxisd(euler[0], ::Eigen::Vector3d::UnitZ())
						* Eigen::AngleAxisd(euler[1], ::Eigen::Vector3d::UnitY())
						* Eigen::AngleAxisd(euler[2], ::Eigen::Vector3d::UnitX());
	Eigen::Quaterniond q(rot);
  return q;
	cout << "R: " <<endl << rot<<endl;
	cout << "q: " <<endl << q.coeffs()<<endl; 
				
}
template <typename T>
Eigen::Matrix<T,3,1> quat2euler(const Eigen::Quaternion<T>& quat )
{
  Eigen::Quaternion<T> q_temp = quat.normalized();
  T aSinInput = -2 * (q_temp.x() * q_temp.z() - q_temp.w() * q_temp.y());
  if(aSinInput > 1)
    aSinInput = 1;
  Eigen::Matrix<T,3,1> euler;
  euler(0) = atan2(2*(q_temp.x()*q_temp.y()+q_temp.w()*q_temp.z()), 
                   q_temp.w() * q_temp.w() + q_temp.x() * q_temp.x()- q_temp.y()*q_temp.y() - q_temp.z() *q_temp.z());
  euler(1) = asin(aSinInput);
  cout << "aSinInput: " <<aSinInput <<endl;
  euler(2) = atan2(2*(q_temp.y() *q_temp.z()+q_temp.w()*q_temp.x()), 
                   q_temp.w() * q_temp.w() - q_temp.x() * q_temp.x() - q_temp.y()*q_temp.y() + q_temp.z() *q_temp.z() );
  return euler;
}

Eigen::Vector3d eulerTest1(Eigen::Vector3d  eul, Eigen::Vector3d t, Eigen::Vector3d p)
{
  float ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, tInY, tInZ;
  ctRoll = cos(eul(2));
    stRoll = sin(eul(2));

    ctPitch = cos(eul(1));
    stPitch = sin(eul(1));

    ctYaw = cos(eul(0));
    stYaw = sin(eul(0));

    tInX = t(0);
    tInY = t(1);
    tInZ = t(2);
    
  float x1 = p(0);
  float y1 = ctRoll * p(1) - stRoll * p(2);
  float z1 = stRoll * p(1) + ctRoll * p(2);
  
  float x2 = ctPitch * x1 + stPitch * z1 ;
  float y2 = y1 ;
  float z2 = -stPitch * x1 + ctPitch * z1 ;
  
  Eigen::Vector3d p2;
  p2(0) = ctYaw * x2 - stYaw * y2 + tInX;
  p2(1) = stYaw * x2 + ctYaw * y2 + tInY;
  p2(2) = z2 + tInZ;
  return p2;

}
std::shared_ptr<double> a_;
std::shared_ptr<int> b_;
vector<float> c_;
struct T2{
  std::shared_ptr<double> a;
  std::shared_ptr<int> b;
  vector<float> c;
};
T2 test2(int d)
{
  a_ = make_shared<double>(d + 1.2);
  b_ = make_shared<int>(d+ 3);
  c_ = {d+ 1,d+2};
  return T2{a_,b_,c_};
}

int main()  
{  
  vector<float> a1 = {1,2,3,4,5};
  vector<float> a2 = {5,6,7,8,16,17};
  a1.resize(5);
  a1.swap(a2);
  cout << a1.size();
  
  
  for(int i = 0; i < 4; i ++)
  {
    cout << a1[i] <<endl;
  }
  T2 bb = test2(1);
  T2 aa = test2(2);
  cout << *(bb.a) <<*(bb.b) << bb.c[0] <<bb.c[1] <<endl;
  return 1;
  Eigen::Vector3d eul1(1.2,0.1,0.3);
  Eigen::Vector3d t1(1.2,2.1,3.3);
  Eigen::Vector3d p1(4,5,6);
  cout << eulerTest1(eul1,t1,p1) <<endl;
  Eigen::Vector3d p3 = euler2quat(eul1) * p1 + t1;
  cout << p3 <<endl;
  return 1;
  cout << Eigen::Matrix<float, 3, 1>::UnitX() <<endl;
  Eigen::Quaterniond q_temp(1,0,-0.5,-0.2);/*q_temp = q_temp.normalized();*/
  cout << q_temp.normalized().coeffs() <<endl;
  cout << quat2euler(q_temp) <<endl;
  euler2quat(quat2euler(q_temp));
  cout <<setprecision(3)<<"q_temp matrix:" << q_temp.normalized().toRotationMatrix() <<endl;
  cout << q_temp.toRotationMatrix().eulerAngles(2,1,0) <<endl;
  setParam(10,20);
  cout << Horizon_SCAN <<endl;
// 	Eigen::Vector3d euler(0.000903,3.114623,1.579208);
// 	euler2quat(euler);
// 	string path = std::string(getenv("CMAKE_PREFIX_PATH"));
// 	cout << path <<endl;
//   cv::Mat a =(Mat_<double>(3,1)<<1,2,3);;
// 
//   cv::Mat b;
//   Rodrigues(a,b);
//   auto c = rotationMatrixToEulerAngles(b);
//   cout << b <<endl;cout <<c <<endl;
// 	return 1;
// 	vector<double > r1s,r2s;
// 	r1s.push_back(1);r1s.push_back(2);r1s.push_back(3);
// 	r2s.push_back(1.5);r2s.push_back(2.5);r2s.push_back(3.5);
// 	Eigen::MatrixX2d A1(r1s.size(), 2);
// 	for(int i =0;i < r1s.size();i++)
// 	{
// 		A1(i,0) = 1.;
// 		A1(i,1) = 2 * r1s[i];
// 	}
// 	std::cout << "get A" <<std::endl<<std::endl;
// 	std::cout << A1 <<endl;
// 	Eigen::VectorXd b1(r1s.size());
// 	for(int i =0; i < r1s.size(); i++)
// 	{
// 		double d1 = r2s[i] * r2s[i] - r1s[i] * r1s[i];
// 		b1(i) = d1;
// 	}
// 	std::cout << "get b: " <<b1.size() <<std::endl<<std::endl;
// 	std::cout << b1 <<endl;
// 	Eigen::Matrix2d aa = ((A1.transpose()) * A1);
// 	Eigen::Matrix2d aa_inv = aa.inverse();
// 	Eigen::VectorXd x = aa_inv*(A1.transpose()) *b1;
	
	org = imread("/home/cyy/1.png",CV_LOAD_IMAGE_UNCHANGED);
	Mat result(org.cols,org.rows,CV_16UC1);
	Mat kernel = getStructuringElement(cv::MORPH_RECT,Size(5,5));
// 	
// // 	contours[0].push_back(Point(10, 10));
// // 	contours[0].push_back(Point(10, 50));
// // 	contours[0].push_back(Point(50, 50));
// // 	contours[0].push_back(Point(50, 10));
// 	Rect rect  = cv::boundingRect(contours[0]);
// 	Mat roi, mask;
// 	org(rect).copyTo(roi);
// 	cv::threshold(roi, mask, 0, 255, CV_THRESH_OTSU);
// 	Scalar color = cv::mean(roi, mask);
// 	
// 	drawContours(org,contours,0,Scalar(255,255,255),CV_FILLED);
// 	rectangle(org, Point2d(10, 10), Point2d(70, 80), Scalar(0,255,0));
// // 	dilate(org,result,kernel);
	Mat org1 = org.clone();
	std::vector<cv::Point> zero_points;
	for (int i = 0; i < org1.rows; i++)
		for (int j = 0; j < org1.cols; j++)
		{
			int value = org1.at<uint16_t>(i, j);
			if (value == 0)
			{
				zero_points.push_back(cv::Point(i,j));
				org1.at<uint16_t>(i, j) = 255*128;
			}
		}
	
// 	imshow("org1",org1);waitKey(0);
	erode(org1,result,kernel);
	GaussianBlur(result,result,Size(5,5),1);
	
	for(auto &pt:zero_points)
		result.at<uint16_t>(pt.x, pt.y) = 0;
// 	dilate(result,result,kernel);
	vector<int> params;
	params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	params.push_back(0);
	imwrite("probability_grid0.png",result,params);
	
	resize(org,img,Size(org.cols * img_ratio,org.rows * img_ratio),0,0,cv::INTER_LINEAR);
// 	org.copyTo(img);  
	img.copyTo(tmp);  
	namedWindow("img");
	setMouseCallback("img",on_mouse,0);   
	imshow("img",result);  
	cv::waitKey(0);
  
	return 0;
}  
