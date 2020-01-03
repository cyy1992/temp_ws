#include <iostream>  
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>    
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

static std::vector<std::vector<cv::Point>> vctvctPoint;
cv::Mat org = cv::imread("/home/cyy/1.png");
cv::Mat dst, maskImage;
static std::vector<cv::Point> vctPoint;
static cv::Point ptStart = Point(-1, -1);
static cv::Point cur_pt = Point(-1, -1);  


bool handlePoints()
{
	int size_points = vctPoint.size();
	for(int i =0; i < size_points-1; i++)
	{
		Point p1 = vctPoint[i];
		Point p2 = vctPoint[i+1];
		int x1,x2,y1,y2;
		x1 = p1.x;y1 = p1.y;
		x2 = p2.x; y2 = p2.y;
		int min_x,max_x;
		min_x = std::min<int>(x1,x2);
		max_x = std::max<int>(x1,x2);
		double k1,c1;
		k1 = (y2-y1)*1.0/(x2-x1)*1.0;
		c1 = y1*1.0 - x1*k1*1.0;
		for(int j =0; j < size_points-1; j++)
		{
			if(j == i || j == i-1 || j==i+1)
			{
// 				j++;
				continue;
			}
			Point p3 = vctPoint[j];
			Point p4 = vctPoint[j+1];
			int x3,y3,x4,y4;
			double k2,c2;
			int min_x1,max_x1;
			x3 = p3.x;y3 = p3.y;
			x4 = p4.x; y4 = p4.y;
			k2 = (y4-y3)*1.0/(x4-x3)*1.0;
			c2 = y3*1.0- x3*k2*1.0;
			min_x1 = std::min<int>(x3,x4);
			max_x1 = std::max<int>(x3,x4);
			double x = (c2-c1)/(k1-k2);
// 			double y_dev = 1.0*((x2-x1)*(y4-y3) - (y2-y1)*(x4-x3));
// 			double y =temp_y/y_dev;
// 			double x = (p3.x + p4.x )*0.5;
// 			double y = (p3.y + p4.y )*0.5;
			if(x <= max_x&&x >= min_x&&x <= max_x1&&x >= min_x1)
			{
				return true;
			}
			
		}
	}
	return false;
}
void on_mouse(int event, int x, int y, int flags, void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号    
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		std::cout << "x:" << x << " y:" << y << std::endl;
		ptStart = cv::Point(x, y);
		vctPoint.push_back(ptStart);
    cv::circle(org, ptStart, 1, cv::Scalar(255, 0, 255), CV_FILLED, CV_AA, 0);
		cv::imshow("test", org);
		//cv::putText(tmp, temp, ptStart, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0, 0), 1, 8);
	}
	else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))
	{
		std::cout << "x:" << x << " y:" << y << std::endl;
		cur_pt = cv::Point(x, y);
		cv::line(org, vctPoint.back(), cur_pt, cv::Scalar(0, 255, 0, 0), 1, 8, 0);
		cv::circle(org, cur_pt, 1, cv::Scalar(255, 0, 255), CV_FILLED, CV_AA, 0);
		cv::imshow("test", org);
		vctPoint.push_back(cur_pt);
		//cv::putText(tmp, temp, cur_pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0, 0));
	}
	else if (event == CV_EVENT_LBUTTONUP)
	{	
		std::cout << "x:" << x << " y:" << y << std::endl;
// 		cur_pt = cv::Point(x, y);
// 		cv::line(org, ptStart, cur_pt, cv::Scalar(0, 255, 0, 0), 1, 8, 0);
// 		cv::circle(org, cur_pt, 1, cv::Scalar(255, 0, 255), CV_FILLED, CV_AA, 0);
		
// 		vctPoint.push_back(cur_pt);
// 		vctvctPoint.push_back(vctPoint);  
		std::string temp;
		if(handlePoints())
		{
			temp = "Yes";
		}
		else{
			temp = "NO";
		}
		cv::putText(org, temp, ptStart, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0, 0), 1, 8);
		cv::imshow("test", org);
		std::vector<cv::Point>().swap(vctPoint);
		cv::waitKey(0);
	}
}

int main()
{
	//鼠标点击
	cv::namedWindow("test");//定义一个img窗口    
	cv::setMouseCallback("test", on_mouse, 0);//调用回调函数    
	cv::imshow("test", org);
	cv::waitKey(0);
	return 0;
}