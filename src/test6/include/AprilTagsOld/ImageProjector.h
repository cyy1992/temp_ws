#ifndef IMAGE_PROJECTOR_H
#define IMAGE_PROJECTOR_H

#include <AprilTagsOld/module.h>
#include <stdint.h>
#include <opencv2/opencv.hpp>
#ifdef HAVE_GPU
void projectImageGpu(cv::cuda::GpuMat& srcImg, cv::cuda::GpuMat& mapx_dev, cv::cuda::GpuMat& mapy_dev, cv::cuda::GpuMat& projImg, int projHeight, int projWidth);
#endif
class ImageProjector
{
public:	
	/* intrinsic			�ڲξ���double���ͣ�
	 * distortion			����������double���ͣ�
	 * extrinsicR			�����ת����double���ͣ�
	 * extrinsicT			���ƽ�ƾ���double���ͣ�
	 * srcImageWidth		ԭʼͼ����ȣ�4�ı�����
	 * srcImageHeight		ԭʼͼ��߶ȣ�4�ı�����
	 * projImageWidth		ͶӰͼ����ȣ�4�ı�����
	 * projImageHeight		ͶӰͼ��߶ȣ�4�ı�����
	 * worldRangeWidth		��������ϵ��Χ����
	 * worldRangeHeight		��������ϵ��Χ�߶�
	 * projImageROIWidth	ͶӰͼ��ROI���ȣ���ָ��ʱ����projImageWidth��
	 * projImageROIHeight	ͶӰͼ��ROI�߶ȣ���ָ��ʱ����projImageHeight��*/
	ImageProjector(const cv::Mat& intrinsic, const cv::Mat& distortion, const cv::Mat& extrinsicR, const cv::Mat& extrinsicT, int srcImageWidth, int srcImageHeight, int projImageWidth, int projImageHeight, double worldRangeWidth, double worldRangeHeight, int projImageROIWidth = -1, int projImageROIHeight = -1, int method = 0);
	~ImageProjector();

	/* distort_point	�����
	 * undistort_point	ȥ�����
	 * world_point		�����
	 * project_point	ͶӰ��*/
	//�����ת�����(z = 0)
	void distort2worldPoint(const cv::Point2d& distort_point, cv::Point3d* world_point);
	//�����תȥ�����
	void distort2undistortPoint(const cv::Point2d& distort_point, cv::Point2d* undistort_point);
	//�����תͶӰ��
	void distort2projectPoint(const cv::Point2d& distort_point, cv::Point2d* project_point);
	//ȥ�����ת�����
	void undistort2distortPoint(const cv::Point2d& undistort_point, cv::Point2d* distort_point);
	//ȥ�����תͶӰ��
	void undistort2projectPoint(const cv::Point2d& undistort_point, cv::Point2d* project_point);
	//��������תͶӰ��(z = 0)
	void world2projectPoint(const cv::Point3d& world_point, cv::Point2d* project_point);
	//ͶӰ��תȥ�����
	void project2undistortPoint(const cv::Point2d& project_point, cv::Point2d* undistort_point);
	//ͶӰ��ת�����
	void project2distortPoint(const cv::Point2d& project_point, cv::Point2d* distort_point);
	//ͶӰ��ת��������(z = 0)
	void project2worldPoint(const cv::Point2d& project_point, cv::Point3d* world_point);

	/* ����ͶӰͼ��
	 * srcImg			IN	ԭʼͼ��
	 * projImg			OUT	ͶӰͼ��
	 * centerDistortLoc	IN	ͶӰͼROI���ĵ��ڻ���ͼ�е����꣨��ָ��ʱ�õ���ͶӰͼ�����ģ�
	 * oriProjLoc		OUT	ͶӰͼ����ʼ��*/
	void projectImage(const cv::Mat& srcImg, cv::Mat& projImg, const cv::Point2d centerDistortLoc = cv::Point2d(-1, -1), cv::Point oriProjLoc = cv::Point());
	void projectImage(const int method, const cv::Mat& srcImg, cv::Mat& projImg, const cv::Point2d centerDistortLoc = cv::Point2d(-1, -1), cv::Point oriProjLoc = cv::Point());

private:
	

	void GetXYGivenZ(const double u, const double v, const double Z, double* X, double* Y);
	void calcObjCoeffs();
	void generateProjTable();
#ifdef HAVE_GPU
	cv::cuda::GpuMat mapx_dev, mapy_dev;
#endif
	double fx;
	double fx_;
	double fy;
	double fy_;
	double cx;
	double cy;
	double k1;
	double k2;
	double exParamR[9];
	double exParamR_[9];
	double exParamT[3];
	
	int srcWidth;
	int srcHeight;
	int projWidth;	
	int projHeight;
	double worldWidth;
	double worldHeight;
	int projROIWidth;
	int projROIHeight;
	int projWidth_2;
	int projHeight_2;
	int projROIWidth_2;
	int projROIHeight_2;
	
	double coeffs_x;
	double coeffs_y;
	double proj_ratio_x;
	double proj_ratio_x_;
	double proj_ratio_y;
	double proj_ratio_y_;

	struct InterLinearTable
	{
		uint16_t x;
		uint16_t y;
		uint16_t a;
		uint16_t b;
	};
	InterLinearTable *proj_table;

};

#endif
