// test2.cpp : 定义控制台应用程序的入口点。
//

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <algorithm>
using namespace std;
using namespace cv;
using namespace Eigen;
Eigen::AlignedBox2d box_;
double resolution = 0.05;
double inv_resolution = 1 / resolution;

enum EXTREME_VALUE
{
	MIN, MAX
};

template <typename T> T GetYaw(const Eigen::Quaternion<T>& rotation)
{
	const Eigen::Matrix<T, 3, 1> direction =
		rotation * Eigen::Matrix<T, 3, 1>::UnitX();
	return atan2(direction.y(), direction.x());
}

double getExtremeValue(const vector<double>& values, const EXTREME_VALUE& type)
{
	if (type == MIN)
	{
		double min_value = values[0];
		for (auto& it:values)
		{
			if (min_value > it)
			{
				min_value = it;
			}
		}
		return min_value;
	}
	else
	{
		double max_value = values[0];
		for (auto& it : values)
		{
			if (max_value < it)
			{
				max_value = it;
			}
		}
		return max_value;
	}
}

int cvAdd4cMat_q(cv::Mat &dst, cv::Mat &scr, double scale)
{
	if (dst.channels() != 3 || scr.channels() != 4)
	{
		return true;
	}
	if (scale < 0.01)
		return false;
	std::vector<cv::Mat>scr_channels;
	std::vector<cv::Mat>dstt_channels;
	split(scr, scr_channels);
	split(dst, dstt_channels);
	CV_Assert(scr_channels.size() == 4 && dstt_channels.size() == 3);

	if (scale < 1)
	{
		scr_channels[3] *= scale;
		scale = 1;
	}
	for (int i = 0; i < 3; i++)
	{
		dstt_channels[i] = dstt_channels[i].mul(255.0 / scale - scr_channels[3], scale / 255.0);
		dstt_channels[i] += scr_channels[i].mul(scr_channels[3], scale / 255.0);
	}
	merge(dstt_channels, dst);
	return true;
}
double theta;
void transfromImg(const Mat& in_img, Mat& out_img, const Eigen::Isometry3d& pose, Eigen::AlignedBox2d& box) 
{
	Eigen::Vector3d p1, p2, p3, p4;
	double temp_min_x ,temp_min_y,temp_max_x,temp_max_y;
	temp_min_x = -in_img.cols + 8.568023* inv_resolution;
	temp_min_y = -in_img.rows + 1.897414* inv_resolution;
	temp_max_x = 8.568023* inv_resolution;
	temp_max_y = 1.897414* inv_resolution;
	
	p1 = Eigen::Vector3d(temp_min_x, temp_min_y,0);
	p2 = Eigen::Vector3d(temp_min_x, temp_max_y, 0);
	p3 = Eigen::Vector3d(temp_max_x, temp_min_y, 0);
	p4 = Eigen::Vector3d(temp_max_x, temp_max_y, 0);

	Eigen::Vector3d q1, q2, q3, q4;
	q1 = pose * p1;
	q2 = pose * p2;
	q3 = pose * p3;
	q4 = pose * p4;

	vector<double> set_x, set_y;
	set_x.push_back(q1(0)); set_x.push_back(q2(0)); set_x.push_back(q3(0)); set_x.push_back(q4(0));
	set_y.push_back(q1(1)); set_y.push_back(q2(1)); set_y.push_back(q3(1)); set_y.push_back(q4(1));
	double min_x, min_y, max_x, max_y;
	min_x = getExtremeValue(set_x, MIN);
	min_y = getExtremeValue(set_y, MIN);
	max_x = getExtremeValue(set_x, MAX);
	max_y = getExtremeValue(set_y, MAX);

	box = Eigen::AlignedBox2d(Eigen::Vector2d(min_x, min_y),
		Eigen::Vector2d(max_x, max_y));

	

	resize(out_img, out_img, Size(max_x -min_x,max_y-min_y), 0, 0, INTER_NEAREST);

	cv::Point2f center(in_img.cols -   1.897414* inv_resolution ,in_img.rows - 8.568023* inv_resolution);
	cv::Mat rot_mat = cv::getRotationMatrix2D(center, theta, 1.0);
	//imshow("22", in_img);
	cv::warpAffine(in_img, out_img, rot_mat, Size(max_x - min_x, max_y - min_y));
}

int main()
{
	Mat img1 = imread("1.png", CV_LOAD_IMAGE_UNCHANGED);
	Mat img2 = imread("3.png", CV_LOAD_IMAGE_UNCHANGED);
	box_ = Eigen::AlignedBox2d(Eigen::Vector2d(0, 0),
		Eigen::Vector2d(img1.cols, img1.rows));
	Eigen::Isometry3d submap2global= Eigen::Isometry3d::Identity();
	submap2global.pretranslate( inv_resolution * Eigen::Vector3d(0.168704,-14.542237,0.000000));
	Eigen::Quaterniond q(1.000000 ,-0.000000,- 0.000000, -0.000265);
	theta = GetYaw(q);
	submap2global.rotate(q);
	Eigen::Isometry3d submap2local = Eigen::Isometry3d::Identity();
	submap2local.pretranslate(inv_resolution * Eigen::Vector3d(0.168023, -14.552587, 0.000000));
	Eigen::Quaterniond q_submap2local(1.000000, -0.000000, -0.000000, -0.);
	theta = GetYaw(q_submap2local);
	submap2local.rotate(q_submap2local);

	Eigen::Isometry3d local2global = submap2global * submap2local.inverse();

	Mat out_img(100, 100, CV_8UC4, Scalar(128, 128, 128, 0));
	Eigen::AlignedBox2d tmp_box;
	transfromImg(img2, out_img, local2global, tmp_box);

	double min_x, min_y, max_x, max_y;

	min_x = MIN(tmp_box.min().x(), box_.min().x());
	min_y = MIN(tmp_box.min().y(), box_.min().y());
	max_x = MAX(tmp_box.max().x(), box_.max().x());
	max_y = MAX(tmp_box.max().y(), box_.max().y());

	box_ = Eigen::AlignedBox2d(Eigen::Vector2d(min_x, min_y),
		Eigen::Vector2d(max_x, max_y));
	cout << min_x << "," << min_y << endl;
	cout << max_x << "," << max_y << endl;
	Mat base_img(max_x-min_x+2, max_y-min_y+2, CV_8UC3, Scalar(128, 128, 128));
	cout << "base_img: "<<base_img.size() << endl;
	
	
	//Mat img1_roi = base_img(Rect(0, 0, img1.cols, img1.rows));
	Mat img1_t1(base_img, cvRect(0, 0, img1.cols, img1.rows));
	cvAdd4cMat_q(img1_t1, img1, 1.0);
	imshow("base_img1", base_img);
	//img1.copyTo(img1_roi, img1);
	/*Mat img2_roi = base_img(Rect(-tmp_box.max().y() + 40, -tmp_box.max().x() + 17.45*20,  out_img.cols, out_img.rows));
	out_img.copyTo(img2_roi,out_img);*/
	//img2.copyTo(base_img);
	Mat img2_t1(base_img, cvRect(-tmp_box.max().y() + 40, -tmp_box.max().x() + 17.45 * 20, out_img.cols, out_img.rows));
	cvAdd4cMat_q(img2_t1, out_img, 1.0);
	imshow("1", base_img);
	waitKey(0);
	return 0;
}

