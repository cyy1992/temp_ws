#include "ImageProjector.h"

using namespace cv;

ImageProjector::ImageProjector(const Mat& intrinsic, const Mat& distortion, const Mat& extrinsicR, const Mat& extrinsicT, int srcImageWidth, int srcImageHeight, int projImageWidth, int projImageHeight, double worldRangeWidth, double worldRangeHeight, int projImageROIWidth, int projImageROIHeight) :srcWidth(srcImageWidth), srcHeight(srcImageHeight), projWidth(projImageWidth), projHeight(projImageHeight), worldWidth(worldRangeWidth), worldHeight(worldRangeHeight)
{
	assert(intrinsic.type() == CV_64F);
	assert(distortion.type() == CV_64F);
	assert(extrinsicR.type() == CV_64F);
	assert(extrinsicT.type() == CV_64F);
	assert(intrinsic.rows == 3);
	assert(intrinsic.cols == 3);
	assert(distortion.rows == 4);
	assert(distortion.cols == 1);
	assert(extrinsicR.rows == 3);
	assert(extrinsicR.cols == 3);
	assert(extrinsicT.rows == 3);
	assert(extrinsicT.cols == 1);

	fx = intrinsic.at<double>(0);
	fy = intrinsic.at<double>(4);
	fx_ = 1. / fx;
	fy_ = 1. / fy;
	cx = intrinsic.at<double>(2);
	cy = intrinsic.at<double>(5);

	k1 = distortion.at<double>(0);
	k2 = distortion.at<double>(1);

	Mat extrinsicR_ = extrinsicR.inv();
	for (int i = 0; i < 9; i++)
	{
		exParamR[i] = extrinsicR.at<double>(i);
		exParamR_[i] = extrinsicR_.at<double>(i);
	}

	for (int i = 0; i < 3; i++)
		exParamT[i] = extrinsicT.at<double>(i);

	proj_ratio_x = worldWidth / projWidth;
	proj_ratio_y = worldHeight / projHeight;
	proj_ratio_x_ = 1. / proj_ratio_x;
	proj_ratio_y_ = 1. / proj_ratio_y;

	assert(projImageROIWidth < projWidth);
	assert(projImageROIHeight < projHeight);
	projROIWidth = projImageROIWidth == -1 ? projWidth : projImageROIWidth;
	projROIHeight = projImageROIHeight == -1 ? projHeight : projImageROIHeight;

	projWidth_2 = projWidth >> 1;
	projHeight_2 = projHeight >> 1;
	projROIWidth_2 = projROIWidth >> 1;
	projROIHeight_2 = projROIHeight >> 1;

	assert(srcImageWidth % 4 == 0);
	assert(srcImageHeight % 4 == 0);
	assert(projROIWidth % 4 == 0);
	assert(projROIHeight % 4 == 0);

	proj_table = new InterLinearTable[projWidth*projHeight];

	calcObjCoeffs();
	generateProjTable();
}

ImageProjector::~ImageProjector()
{
	delete[] proj_table;
}

//畸变点转世界坐标(z = 0)
void ImageProjector::distort2worldPoint(const Point2d& distort_point, Point3d* world_point)
{
	int i;
	double r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3, s;

	double u = distort_point.x;
	double v = distort_point.y;

	double y0 = (v - cy)*fy_;
	double x0 = (u - cx)*fx_;

	double x = x0;
	double y = y0;

	double r2, icdist;

	for (i = 0; i < 5; i++)
	{
		r2 = x*x + y*y;
		icdist = 1. / (1 + (k2*r2 + k1)*r2);
		x = x0*icdist;
		y = y0*icdist;
	}
	//x,y为去畸变点的摄像机坐标

	//计算该点的世界坐标
	r11 = exParamR_[0];
	r12 = exParamR_[1];
	r13 = exParamR_[2];
	r21 = exParamR_[3];
	r22 = exParamR_[4];
	r23 = exParamR_[5];
	r31 = exParamR_[6];
	r32 = exParamR_[7];
	r33 = exParamR_[8];

	t1 = exParamT[0];
	t2 = exParamT[1];
	t3 = exParamT[2];

	s = (r31*t1 + r32*t2 + r33*t3) / (r31*x + r32*y + r33);

	t1 = s*x - t1;
	t2 = s*y - t2;
	t3 = s - t3;

	world_point->x = r11*t1 + r12*t2 + r13*t3;
	world_point->y = r21*t1 + r22*t2 + r23*t3;
	world_point->z = 0;
}

//畸变点转去畸变点
void ImageProjector::distort2undistortPoint(const Point2d& distort_point, Point2d* undistort_point)
{
	int i;

	double u = distort_point.x;
	double v = distort_point.y;

	double y0 = (v - cy)*fy_;
	double x0 = (u - cx)*fx_;

	double x = x0;
	double y = y0;

	double r2, icdist;

	for (i = 0; i < 5; i++)
	{
		r2 = x*x + y*y;
		icdist = 1. / (1 + (k2*r2 + k1)*r2);
		x = x0*icdist;
		y = y0*icdist;
	}

	undistort_point->x = x*fx + cx;
	undistort_point->y = y*fy + cy;

	//计算有误差，用原始数据
	/*float dx = image_point_dst.x - image_point_src.x;
	float dy = image_point_dst.y - image_point_src.y;
	if (dx*dx + dy*dy >60)
	{
	image_point_dst = image_point_src;
	}*/
}

//畸变点转投影点
void ImageProjector::distort2projectPoint(const Point2d& distort_point, Point2d* project_point)
{
	int i;
	double r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3, s;

	double u = distort_point.x;
	double v = distort_point.y;

	double y0 = (v - cy)*fy_;
	double x0 = (u - cx)*fx_;

	double x = x0;
	double y = y0;

	double r2, icdist;

	for (i = 0; i < 5; i++)
	{
		r2 = x*x + y*y;
		icdist = 1. / (1 + (k2*r2 + k1)*r2);
		x = x0*icdist;
		y = y0*icdist;
	}
	//x,y为去畸变点的摄像机坐标

	//计算该点的世界坐标
	r11 = exParamR_[0];
	r12 = exParamR_[1];
	r13 = exParamR_[2];
	r21 = exParamR_[3];
	r22 = exParamR_[4];
	r23 = exParamR_[5];
	r31 = exParamR_[6];
	r32 = exParamR_[7];
	r33 = exParamR_[8];

	t1 = exParamT[0];
	t2 = exParamT[1];
	t3 = exParamT[2];

	s = (r31*t1 + r32*t2 + r33*t3) / (r31*x + r32*y + r33);

	t1 = s*x - t1;
	t2 = s*y - t2;
	t3 = s - t3;

	x = r11*t1 + r12*t2 + r13*t3;
	y = r21*t1 + r22*t2 + r23*t3;
	//x,y为该点的世界坐标

	project_point->x = (x - coeffs_x)*proj_ratio_x_;
	project_point->y = (y - coeffs_y)*proj_ratio_y_;
}

//去畸变点转畸变点
void ImageProjector::undistort2distortPoint(const Point2d& undistort_point, Point2d* distort_point)
{
	double x, y, r2, r4, xd, yd, cdist;

	//去畸变点转畸变点
	x = (undistort_point.x - cx)*fx_;
	y = (undistort_point.y - cy)*fy_;

	//去畸变点转畸变点
	r2 = x*x + y*y;
	r4 = r2*r2;

	cdist = 1 + k1*r2 + k2*r4;

	xd = x*cdist;
	yd = y*cdist;

	distort_point->x = xd*fx + cx;
	distort_point->y = yd*fy + cy;
}

//去畸变点转投影点
void ImageProjector::undistort2projectPoint(const Point2d& undistort_point, Point2d* project_point)
{
	double u = undistort_point.x;;
	double v = undistort_point.y;
	double X = 0;
	double Y = 0;

	GetXYGivenZ(u, v, 0, &X, &Y);

	project_point->x = (X - coeffs_x)*proj_ratio_x_;
	project_point->y = (Y - coeffs_y)*proj_ratio_y_;
}

//世界坐标转投影点(z = 0)
void ImageProjector::world2projectPoint(const Point3d& world_point, Point2d* project_point)
{
	project_point->x = (world_point.x - coeffs_x)*proj_ratio_x_;
	project_point->y = (world_point.y - coeffs_y)*proj_ratio_y_;
}

//投影点转去畸变点
void ImageProjector::project2undistortPoint(const Point2d& project_point, Point2d* undistort_point)
{
	double X, Y, Z, x, y, z;

	//投影点转去畸变点
	X = proj_ratio_x*project_point.x + coeffs_x;
	Y = proj_ratio_x*project_point.y + coeffs_y;
	Z = 0;

	x = exParamR[0] * X + exParamR[1] * Y + exParamR[2] * Z + exParamT[0];
	y = exParamR[3] * X + exParamR[4] * Y + exParamR[5] * Z + exParamT[1];
	z = exParamR[6] * X + exParamR[7] * Y + exParamR[8] * Z + exParamT[2];

	z = z ? 1. / z : 1.;
	x *= z; y *= z;

	undistort_point->x = x*fx + cx;
	undistort_point->y = y*fy + cy;
}

//投影点转畸变点
void ImageProjector::project2distortPoint(const Point2d& project_point, Point2d* distort_point)
{
	Point3d obj_point;
	double X, Y, Z, x, y, z;
	double r2, r4, xd, yd, cdist;

	//投影点转去畸变点
	obj_point.x = proj_ratio_x*project_point.x + coeffs_x;
	obj_point.y = proj_ratio_y*project_point.y + coeffs_y;
	obj_point.z = 0;

	X = obj_point.x, Y = obj_point.y, Z = obj_point.z;
	x = exParamR[0] * X + exParamR[1] * Y + exParamR[2] * Z + exParamT[0];
	y = exParamR[3] * X + exParamR[4] * Y + exParamR[5] * Z + exParamT[1];
	z = exParamR[6] * X + exParamR[7] * Y + exParamR[8] * Z + exParamT[2];

	z = z ? 1. / z : 1.;
	x *= z; y *= z;
	//x,y为去畸变点的摄像机坐标

	//去畸变点转畸变点
	r2 = x*x + y*y;
	r4 = r2*r2;

	cdist = 1 + k1*r2 + k2*r4;

	xd = x*cdist;
	yd = y*cdist;

	distort_point->x = xd*fx + cx;
	distort_point->y = yd*fy + cy;
}

//投影点转世界坐标(z = 0)
void ImageProjector::project2worldPoint(const Point2d& project_point, Point3d* world_point)
{
	world_point->x = proj_ratio_x*project_point.x + coeffs_x;
	world_point->y = proj_ratio_y*project_point.y + coeffs_y;
	world_point->z = 0;
}

//从去畸变点计算对应的世界坐标
void ImageProjector::GetXYGivenZ(const double u, const double v, const double Z, double* X, double* Y)
{
	double x0 = (u - cx)*fx_;
	double y0 = (v - cy)*fy_;

	//Mat A=(Mat_<float>(3,1)<<x0,y0,1);

	double r11 = exParamR_[0];
	double r12 = exParamR_[1];
	double r13 = exParamR_[2];
	double r21 = exParamR_[3];
	double r22 = exParamR_[4];
	double r23 = exParamR_[5];
	double r31 = exParamR_[6];
	double r32 = exParamR_[7];
	double r33 = exParamR_[8];

	double t1 = exParamT[0];
	double t2 = exParamT[1];
	double t3 = exParamT[2];

	double s = (Z + r31*t1 + r32*t2 + r33*t3) / (r31*x0 + r32*y0 + r33);

	//Mat obj_3D(3,1,CV_32FC1);
	//obj_3D=invR*(s*A-translation_vector);

	t1 = s*x0 - t1;
	t2 = s*y0 - t2;
	t3 = s - t3;

	*X = r11*t1 + r12*t2 + r13*t3;
	*Y = r21*t1 + r22*t2 + r23*t3;

	//X=obj_3D.at<float>(0,0);
	//Y=obj_3D.at<float>(1,0);
}

void ImageProjector::calcObjCoeffs()
{
	Point3d obj_lt, obj_lb, obj_rt, obj_rb;
	Point2d src_point, obj_center;

	//将四个顶点从畸变图像坐标系转换到为世界坐标（mm）
	src_point.x = 0;
	src_point.y = 0;
	distort2worldPoint(src_point, &obj_lt);
	src_point.y = srcHeight - 1;
	distort2worldPoint(src_point, &obj_lb);
	src_point.x = srcWidth - 1;
	distort2worldPoint(src_point, &obj_rb);
	src_point.y = 0;
	distort2worldPoint(src_point, &obj_rt);

	//计算空间坐标的中心点
	obj_center.x = (MIN(obj_lt.x, MIN(obj_lb.x, MIN(obj_rt.x, obj_rb.x))) + MAX(obj_lt.x, MAX(obj_lb.x, MAX(obj_rt.x, obj_rb.x))))*0.5;
	obj_center.y = (MIN(obj_lt.y, MIN(obj_lb.y, MIN(obj_rt.y, obj_rb.y))) + MAX(obj_lt.y, MAX(obj_lb.y, MAX(obj_rt.y, obj_rb.y))))*0.5;

	//中心点的偏移量(世界坐标平移系数）
	coeffs_x = obj_center.x - worldWidth*0.5;
	coeffs_y = obj_center.y - worldHeight*0.5;
}

void ImageProjector::generateProjTable()
{
	Point2d proj_point, distort_point;
	int i, j;
	uint16_t x_, y_;
	double x0, y0, a, b;

#ifdef CALC_DM_DISTORT_IMG_SIZE
	int k, l;
	uint16_t x_min, x_max, y_min, y_max, tmp;
	int16_t tmp1, tmp2, x_range = 0, y_range = 0;
#endif

	//生成投影表格,i是列,j是行
	for (i = 0; i < projWidth; i++)
	{
		for (j = 0; j < projHeight; j++)
		{
			proj_point.x = i;
			proj_point.y = j;
			project2distortPoint(proj_point, &distort_point);

			x0 = MIN(MAX(distort_point.x, 0.0), srcWidth - 1.01);
			y0 = MIN(MAX(distort_point.y, 0.0), srcHeight - 1.01);

			x_ = (uint16_t)x0;
			y_ = (uint16_t)y0;

			b = x0 - x_;			//b	
			a = y0 - y_;

			proj_table[j*projWidth + i].x = x_;
			proj_table[j*projWidth + i].y = y_;
			proj_table[j*projWidth + i].a = (uint16_t)(a * 65536);
			proj_table[j*projWidth + i].b = (uint16_t)(b * 65536);
		}
	}


#ifdef CALC_DM_DISTORT_IMG_SIZE
	//根据当前投影表的尺寸，计算畸变图所需的大小
	for (i = 0; i < DOWN_PROJ_IMG_SIZE1 - DOWN_PROJ_IMG_SIZE2 + 1; i++)
	{
		for (j = 0; j < DOWN_PROJ_IMG_SIZE1 - DOWN_PROJ_IMG_SIZE2 + 1; j++)
		{
			x_min = 9999, x_max = 0;
			y_min = 9999, y_max = 0;
			for (k = 0; k < DOWN_PROJ_IMG_SIZE2; k++)
			{
				for (l = 0; l < DOWN_PROJ_IMG_SIZE2; l++)
				{
					tmp = proj_table[(i + k)*DOWN_PROJ_IMG_SIZE1 + j + l].x;
					if (tmp > x_max)
						x_max = tmp;
					if (tmp < x_min)
						x_min = tmp;

					tmp = proj_table[(i + k)*DOWN_PROJ_IMG_SIZE1 + j + l].y;
					if (tmp > y_max)
						y_max = tmp;
					if (tmp < y_min)
						y_min = tmp;
				}
			}
			tmp1 = abs(x_min - proj_table[(i + DOWN_PROJ_IMG_SIZE2 / 2)*DOWN_PROJ_IMG_SIZE1 + j + DOWN_PROJ_IMG_SIZE2 / 2].x);
			tmp2 = abs(x_max - proj_table[(i + DOWN_PROJ_IMG_SIZE2 / 2)*DOWN_PROJ_IMG_SIZE1 + j + DOWN_PROJ_IMG_SIZE2 / 2].x);
			tmp = max(tmp1, tmp2);
			if (tmp > x_range)
				x_range = tmp;

			tmp1 = abs(y_min - proj_table[(i + DOWN_PROJ_IMG_SIZE2 / 2)*DOWN_PROJ_IMG_SIZE1 + j + DOWN_PROJ_IMG_SIZE2 / 2].y);
			tmp2 = abs(y_max - proj_table[(i + DOWN_PROJ_IMG_SIZE2 / 2)*DOWN_PROJ_IMG_SIZE1 + j + DOWN_PROJ_IMG_SIZE2 / 2].y);
			tmp = max(tmp1, tmp2);
			if (tmp > y_range)
			{
				y_range = tmp;
				//cout<<i<<'\t'<<j<<'\t'<<y_range<<endl;
			}
		}
	}
	x_range++;
	y_range++;

	printf("%d\t%d\n", (x_range + 1) * 2, (y_range + 1) * 2);
#endif
}

void ImageProjector::projectImage(const Mat& srcImg, Mat& projImg, const Point2d centerDistortLoc, Point oriProjLoc)
{
	int i, j, src_point_offset;
	int16_t x, y;
	uint16_t a, b, a_, b_, factor1, factor2;
	uint8_t g00, g01, g10, g11;
	Point2d point_project;
	uint8_t* current_point;
	uint8_t* src_point;
	InterLinearTable* current_table_point;
	double tmp_x, tmp_y;
	int jump_length;

	src_point = srcImg.data;

	if (projImg.data == NULL)
		projImg.create(projROIHeight, projROIWidth, CV_8UC1);
	else if (projImg.rows != projROIHeight || projImg.cols != projROIWidth || projImg.type() != CV_8UC1)
	{
		projImg.release();
		projImg.create(projROIHeight, projROIWidth, CV_8UC1);
	}

	if (centerDistortLoc.x < 0)
	{
		distort2projectPoint(centerDistortLoc, &point_project);

		tmp_x = point_project.x;
		tmp_y = point_project.y;

		if (tmp_x > projROIWidth_2)
		{
			if (tmp_x > projWidth - projROIWidth_2)
				point_project.x = projWidth - projROIWidth_2;
		}
		else
			point_project.x = projROIWidth_2;

		if (tmp_y > projROIHeight_2)
		{
			if (tmp_y > projHeight - projROIHeight_2)
				point_project.y = projHeight - projROIHeight_2;
		}
		else
			point_project.y = projROIHeight_2;

		oriProjLoc.x = (int)(point_project.x - projROIWidth_2 + 0.5);
		oriProjLoc.y = (int)(point_project.y - projROIHeight_2 + 0.5);
	}
	else
	{
		oriProjLoc.x = projWidth_2 - projROIWidth_2;
		oriProjLoc.y = projHeight_2 - projROIHeight_2;
	}

	current_table_point = proj_table + oriProjLoc.y*projWidth + oriProjLoc.x;
	current_point = projImg.data;
	jump_length = projWidth - projROIWidth;

	for (i = 0; i < projROIHeight; i++)
	{
		for (j = 0; j < projROIWidth; j++)
		{
			x = current_table_point->x;
			y = current_table_point->y;
			a = current_table_point->a;
			b = current_table_point->b;
			current_table_point++;

			assert(x >= 0 && x < srcWidth - 1);
			assert(y >= 0 && y < srcHeight - 1);

			src_point_offset = y*srcWidth + x;
			g00 = src_point[src_point_offset++];
			g01 = src_point[src_point_offset];
			src_point_offset += srcWidth;
			g11 = src_point[src_point_offset--];
			g10 = src_point[src_point_offset];

			a_ = 0xFFFF - a;
			b_ = 0xFFFF - b;

			factor1 = (a*g11 + a_*g01) >> 8;
			factor2 = (a*g10 + a_*g00) >> 8;
			*current_point = (b*factor1 + b_*factor2) >> 24;
			current_point++;

			//dstImg[i] = (uint8_t)(b*(a*g11+(1-a)*g01)+(1-b)*(a*g10+(1-a)*g00));
		}
		current_table_point += jump_length;
	}
}
