#include <imreg_fmt/image_registration.h>
#include <opencv2/core/eigen.hpp>

#include <imreg_fmt/awesome_clock.h>
#include <fstream>
#include <iostream>
// #include <glog/logging.h>
using namespace cv;

//#define TIME_TEST

// TODO: cvfix_rotatedRectangleIntersection is a replacement function for
// cv::rotatedRectangleIntersection, which has a bug due to float underflow
// When OpenCV version is upgraded to be >= 4.0,
// we can remove this replacement function.
// For anyone interested, here're the PRs on OpenCV:
// https://github.com/opencv/opencv/issues/12221
// https://github.com/opencv/opencv/pull/12222
int cvfix_rotatedRectangleIntersection(const cv::RotatedRect& rect1,
                                       const cv::RotatedRect& rect2,
                                       cv::OutputArray intersectingRegion)
{
  const float samePointEps = 0.00001f;
  // used to test if two points are the same

  cv::Point2f vec1[4], vec2[4];
  cv::Point2f pts1[4], pts2[4];

  std::vector<cv::Point2f> intersection;

  rect1.points(pts1);
  rect2.points(pts2);

  int ret = cv::INTERSECT_FULL;

  // Specical case of rect1 == rect2
  {
    bool same = true;

    for (int i = 0; i < 4; i++)
    {
      if (fabs(pts1[i].x - pts2[i].x) > samePointEps ||
          (fabs(pts1[i].y - pts2[i].y) > samePointEps))
      {
        same = false;
        break;
      }
    }

    if (same)
    {
      intersection.resize(4);

      for (int i = 0; i < 4; i++)
      {
        intersection[i] = pts1[i];
      }

      cv::Mat(intersection).copyTo(intersectingRegion);

      return cv::INTERSECT_FULL;
    }
  }

  // Line vector
  // A line from p1 to p2 is: p1 + (p2-p1)*t, t=[0,1]
  for (int i = 0; i < 4; i++)
  {
    vec1[i].x = pts1[(i + 1) % 4].x - pts1[i].x;
    vec1[i].y = pts1[(i + 1) % 4].y - pts1[i].y;

    vec2[i].x = pts2[(i + 1) % 4].x - pts2[i].x;
    vec2[i].y = pts2[(i + 1) % 4].y - pts2[i].y;
  }

  // Line test - test all line combos for intersection
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      // Solve for 2x2 Ax=b
      float x21 = pts2[j].x - pts1[i].x;
      float y21 = pts2[j].y - pts1[i].y;

      float vx1 = vec1[i].x;
      float vy1 = vec1[i].y;

      float vx2 = vec2[j].x;
      float vy2 = vec2[j].y;

      float det = vx2 * vy1 - vx1 * vy2;

      float t1 = (vx2 * y21 - vy2 * x21) / det;
      float t2 = (vx1 * y21 - vy1 * x21) / det;

      // This takes care of parallel lines
      if (cvIsInf(t1) || cvIsInf(t2) || cvIsInf(t1) || cvIsInf(t2))
      {
        continue;
      }

      if (t1 >= 0.0f && t1 <= 1.0f && t2 >= 0.0f && t2 <= 1.0f)
      {
        float xi = pts1[i].x + vec1[i].x * t1;
        float yi = pts1[i].y + vec1[i].y * t1;

        intersection.push_back(cv::Point2f(xi, yi));
      }
    }
  }

  if (!intersection.empty())
  {
    ret = cv::INTERSECT_PARTIAL;
  }

  // Check for vertices from rect1 inside recct2
  for (int i = 0; i < 4; i++)
  {
    // We do a sign test to see which side the point lies.
    // If the point all lie on the same sign for all 4 sides of the rect,
    // then there's an intersection
    int posSign = 0;
    int negSign = 0;

    float x = pts1[i].x;
    float y = pts1[i].y;

    for (int j = 0; j < 4; j++)
    {
      // line equation: Ax + By + C = 0
      // see which side of the line this point is at

      // float causes underflow!
      // Original version:
      // float A = -vec2[j].y;
      // float B = vec2[j].x;
      // float C = -(A * pts2[j].x + B * pts2[j].y);
      // float s = A * x + B * y + C;

      double A = -vec2[j].y;
      double B = vec2[j].x;
      double C = -(A * pts2[j].x + B * pts2[j].y);
      double s = A * x + B * y + C;

      if (s >= 0)
      {
        posSign++;
      }
      else
      {
        negSign++;
      }
    }

    if (posSign == 4 || negSign == 4)
    {
      intersection.push_back(pts1[i]);
    }
  }

  // Reverse the check - check for vertices from rect2 inside recct1
  for (int i = 0; i < 4; i++)
  {
    // We do a sign test to see which side the point lies.
    // If the point all lie on the same sign for all 4 sides of the rect,
    // then there's an intersection
    int posSign = 0;
    int negSign = 0;

    float x = pts2[i].x;
    float y = pts2[i].y;

    for (int j = 0; j < 4; j++)
    {
      // line equation: Ax + By + C = 0
      // see which side of the line this point is at

      // float causes underflow!
      // Original version:
      // float A = -vec1[j].y;
      // float B = vec1[j].x;
      // float C = -(A * pts1[j].x + B * pts1[j].y);
      // float s = A*x + B*y + C;

      double A = -vec1[j].y;
      double B = vec1[j].x;
      double C = -(A * pts1[j].x + B * pts1[j].y);
      double s = A * x + B * y + C;

      if (s >= 0)
      {
        posSign++;
      }
      else
      {
        negSign++;
      }
    }

    if (posSign == 4 || negSign == 4)
    {
      intersection.push_back(pts2[i]);
    }
  }

  // Get rid of dupes
  for (int i = 0; i < (int)intersection.size() - 1; i++)
  {
    for (size_t j = i + 1; j < intersection.size(); j++)
    {
      float dx = intersection[i].x - intersection[j].x;
      float dy = intersection[i].y - intersection[j].y;
      // can be a really small number, need double here
      double d2 = dx * dx + dy * dy;

      if (d2 < samePointEps * samePointEps)
      {
        // Found a dupe, remove it
        std::swap(intersection[j], intersection.back());
        intersection.pop_back();
        j--; // restart check
      }
    }
  }

  if (intersection.empty())
  {
    return cv::INTERSECT_NONE;
  }

  // If this check fails then it means we're getting dupes
  // CV_Assert(intersection.size() <= 8);

  // At this point, there might still be some edge cases failing the check above
  // However, it doesn't affect the result of polygon area,
  // even if the number of intersections is greater than 8.
  // Therefore, we just print out these cases for now instead of assertion.
  // TODO: These cases should provide good reference for improving the accuracy
  // for intersection computation above (for example, we should use
  // cross-product/dot-product of vectors instead of line equation to
  // judge the relationships between the points and line segments)

  if (intersection.size() > 8)
  {
    std::cout << "Intersection size = " << intersection.size() << std::endl;
    std::cout << "Rect 1:\n";
    for (int i = 0; i < 4; i++)
    {
      std::cout << " (" << pts1[i].x << " ," << pts1[i].y << "),\n";
    }
    std::cout << "Rect 2:\n";
    for (int i = 0; i < 4; i++)
    {
      std::cout << " (" << pts2[i].x << " ," << pts2[i].y << "),\n";
    }
    std::cout << "Intersections:\n";
    for (auto& p : intersection)
    {
      std::cout << " (" << p.x << " ," << p.y << "),\n";
    }
  }

  cv::Mat(intersection).copyTo(intersectingRegion);

  return ret;
}

ImageRegistration::ImageRegistration(const int rows, const int cols)
    : rows_(rows), cols_(cols), log_polar_size_(std::max(rows_, cols_)),
      image_transforms_(rows_, cols_, log_polar_size_, log_polar_size_)
{
  high_pass_filter_ = getHighPassFilter();
  // initialize(im);
}

ImageRegistration::~ImageRegistration() {}

Eigen::MatrixXd ImageRegistration::getHighPassFilter()
{
  Eigen::VectorXd yy =
      Eigen::VectorXd::LinSpaced(rows_, -M_PI / 2.0, M_PI / 2.0);
  Eigen::VectorXd yy_vec = Eigen::VectorXd::Ones(cols_);
  Eigen::MatrixXd yy_matrix =
      yy * yy_vec.transpose(); // identical cols, each row is linspace

  Eigen::VectorXd xx =
      Eigen::VectorXd::LinSpaced(cols_, -M_PI / 2.0, M_PI / 2.0);
  Eigen::VectorXd xx_vec = Eigen::VectorXd::Ones(rows_);
  Eigen::MatrixXd xx_matrix = xx_vec * xx.transpose();

  Eigen::MatrixXd filter =
      (yy_matrix.cwiseProduct(yy_matrix) + xx_matrix.cwiseProduct(xx_matrix))
          .cwiseSqrt()
          .array()
          .cos();
  filter = filter.cwiseProduct(filter);
  filter = -filter;
  filter = filter.array() + 1.0;
  return filter;
}

void ImageRegistration::initialize(const cv::Mat& im)
{
  CV_Assert(!im.empty());
  CV_Assert(im.type() == CV_8UC1);
  processImage(im, im0_gray_, im0_logpolar_);
}

double ImageRegistration::registerImage(const cv::Mat& im, cv::Mat& cur2ref_R,
                                        cv::Mat& cur2ref_T, double& overlap,
                                        double& double_check, bool fix_scale,
                                        bool display_images)
{
  double rs_row, rs_col;
  double t_row, t_col;
  double scale, rotation, last_rotation;

  CV_Assert(!im.empty());
  CV_Assert(!cur2ref_R.empty());
  CV_Assert(!cur2ref_T.empty());
  CV_Assert(im.type() == CV_8UC1);
  CV_Assert(cur2ref_R.type() == CV_64FC1);
  CV_Assert(cur2ref_T.type() == CV_64FC1);

#ifdef TIME_TEST
  AwesomeClock t;
  t.start();
#endif
  processImage(im, im1_gray_, im1_logpolar_);
#ifdef TIME_TEST
  std::cout << "Part1 time used: " << t.stop() * 1e3 << " ms.\n";
  t.start();
#endif
  // rs_col);
  cv::Point2d rotation_and_scale =
      cv::phaseCorrelate(im1_logpolar_, im0_logpolar_);
  rs_row = -rotation_and_scale.y;
  rs_col = -rotation_and_scale.x;

  image_transforms_.getScaleRotation(rs_row, rs_col, scale, rotation);
  if (fix_scale)
    scale = 1.0;

  last_rotation =
      atan2(cur2ref_R.at<double>(1, 0), cur2ref_R.at<double>(0, 0)) * 180.0 /
      M_PI;
  //   std::cout << "last_rotation: " << last_rotation << std::endl;
  double diff_rotation = fabs(rotation - last_rotation);
  if (diff_rotation > 90.)
    rotation = rotation > 0. ? (rotation - 180.) : (rotation + 180.);
//   std::cout << "rotation: " << rotation << std::endl;
// last_rotation_ = rotation;
#ifdef TIME_TEST
  std::cout << "Part2 time used: " << t.stop() * 1e3 << " ms.\n";
  t.start();
#endif
  image_transforms_.rotateAndScale(im0_gray_, im0_rotated_, scale, rotation);
#ifdef TIME_TEST
  std::cout << "Part3 time used: " << t.stop() * 1e3 << " ms.\n";
  t.start();
#endif
  double response;

  //  Mat im1_gray = Mat::zeros(im1_gray_.rows*2, im1_gray_.cols*2,
  //  im1_gray_.type());
  //  im1_gray_.copyTo(im1_gray(Rect(im1_gray_.cols/2,im1_gray_.rows/2,im1_gray_.cols,im1_gray_.rows)));
  //  Mat im0_rotated = Mat::zeros(im0_rotated_.rows*2, im0_rotated_.cols*2,
  //  im0_rotated_.type());
  //  im0_rotated_.copyTo(im0_rotated(Rect(im0_rotated_.cols/2,im0_rotated_.rows/2,im0_rotated_.cols,im0_rotated_.rows)));

  cv::Point2d tr =
      cv::phaseCorrelate(im1_gray_, im0_rotated_, noArray(), &response);
  
  t_row = -tr.y;
  t_col = -tr.x;
  if(0&&(std::fabs(t_row) > 150 || std::fabs(t_col) > 150 ))
  {
//      LOG(WARNING) << t_row << ", " << t_col << ", " <<response ;
//      LOG(WARNING) <<"cur2ref_R: \n" <<  cur2ref_R <<"\n cur2ref_T: \n" << cur2ref_T 
//       << "\noverlap... :\n" << overlap <<", " << double_check << ", " << fix_scale << ", " << display_images;
    cv::Mat im0_show, im1_show;
    im1_gray_.convertTo(im1_show, CV_8UC1, 255.);
    im0_rotated_.convertTo(im0_show, CV_8UC1, 255.);
    cv::imwrite("/home/jz/im0_show.png", im0_show);
    cv::imwrite("/home/jz/im1_show.png", im1_show);
    
    cv::imwrite("/home/jz/im0_rotated.png", im0_rotated_);
    cv::imwrite("/home/jz/im1_gray.png", im1_gray_);
  }
  

  //  std::cout << "x: " << t_col << ", y: " << t_row
  //            << ", rotation: " << rotation << ", scale: " << scale
  //            << std::endl;

  if (double_check > 0. && (std::fabs(t_row) < 150 && std::fabs(t_col) < 150))
  {
    const int rows = im0_rotated_.rows;
    const int cols = im0_rotated_.cols;
    const int area_thresh = rows * cols / 6;
    const double row_offset = t_row < 0 ? t_row + im0_rotated_.rows : t_row;
    const double col_offset = t_col < 0 ? t_col + im0_rotated_.cols : t_col;
    const int row_offset_int = (int)row_offset;
    const int col_offset_int = (int)col_offset;
    int cur_area;
    double max_response = 0.;
    double best_t_row;
    double best_t_col;

    cur_area = (cols - col_offset_int) * (rows - row_offset_int);
    if (cur_area > area_thresh)
    {
      Mat cur_gray =
          im1_gray_(Rect(col_offset_int, row_offset_int, cols - col_offset_int,
                         rows - row_offset_int));
      Mat rotated_ref_gray = im0_rotated_(
          Rect(0, 0, cols - col_offset_int, rows - row_offset_int));

      double response_check;
      cv::Point2d tr_check = cv::phaseCorrelate(cur_gray, rotated_ref_gray,
                                                noArray(), &response_check);

      // std::cout << "area1: " << response_check << ", (" << col_offset << ", "
      // << row_offset << ").\n";
      if (response_check > max_response && fabs(tr_check.x) < 3. &&
          fabs(tr_check.y) < 3.)
      {
        max_response = response_check;
        best_t_row = row_offset;
        best_t_col = col_offset;
      }
    }

    cur_area = col_offset_int * (rows - row_offset_int);
    if (cur_area > area_thresh)
    {
      Mat cur_gray = im1_gray_(
          Rect(0, row_offset_int, col_offset_int, rows - row_offset_int));
      Mat rotated_ref_gray = im0_rotated_(Rect(
          cols - col_offset_int, 0, col_offset_int, rows - row_offset_int));

      double response_check;
      cv::Point2d tr_check = cv::phaseCorrelate(cur_gray, rotated_ref_gray,
                                                noArray(), &response_check);

      // std::cout << "area2: " << response_check << ", (" << col_offset-cols <<
      // ", " << row_offset << ").\n";
      if (response_check > max_response && fabs(tr_check.x) < 3. &&
          fabs(tr_check.y) < 3.)
      {
        max_response = response_check;
        best_t_row = row_offset;
        best_t_col = col_offset - cols;
      }
    }

    cur_area = (cols - col_offset_int) * row_offset_int;
    if (cur_area > area_thresh)
    {
      Mat cur_gray = im1_gray_(
          Rect(col_offset_int, 0, cols - col_offset_int, row_offset_int));
      Mat rotated_ref_gray = im0_rotated_(Rect(
          0, rows - row_offset_int, cols - col_offset_int, row_offset_int));

      double response_check;
      cv::Point2d tr_check = cv::phaseCorrelate(cur_gray, rotated_ref_gray,
                                                noArray(), &response_check);

      // std::cout << "area3: " << response_check << ", (" << col_offset << ", "
      // << row_offset-rows << ").\n";
      if (response_check > max_response && fabs(tr_check.x) < 3. &&
          fabs(tr_check.y) < 3.)
      {
        max_response = response_check;
        best_t_row = row_offset - rows;
        best_t_col = col_offset;
      }
    }

    cur_area = col_offset_int * row_offset_int;
    if (cur_area > area_thresh)
    {
      Mat cur_gray = im1_gray_(Rect(0, 0, col_offset_int, row_offset_int));
      Mat rotated_ref_gray =
          im0_rotated_(Rect(cols - col_offset_int, rows - row_offset_int,
                            col_offset_int, row_offset_int));

      double response_check;
      cv::Point2d tr_check = cv::phaseCorrelate(cur_gray, rotated_ref_gray,
                                                noArray(), &response_check);

      // std::cout << "area4: " << response_check << ", (" << col_offset-cols <<
      // ", " << row_offset-rows << ").\n";
      if (response_check > max_response && fabs(tr_check.x) < 3. &&
          fabs(tr_check.y) < 3.)
      {
        max_response = response_check;
        best_t_row = row_offset - rows;
        best_t_col = col_offset - cols;
      }
    }
    if (max_response > double_check)
    {
      double_check = max_response;
      t_row = best_t_row;
      t_col = best_t_col;
    }
  }

//  cv::Mat registered_image;
//  image_transforms_.translate(im0_rotated_, registered_image, t_col,
//                              t_row); // x, y
//  imshow("registered_image", registered_image);
//  waitKey(0);

#ifdef TIME_TEST
  std::cout << "Part4 time used: " << t.stop() * 1e3 << " ms.\n";
  t.start();
#endif
  cv::Mat rotationMatrix = cv::getRotationMatrix2D(
      cv::Point(im0_gray_.cols / 2, im0_gray_.rows / 2), rotation, scale);
  double r11, r12, r13, r21, r22, r23;
  r11 = rotationMatrix.at<double>(0, 0);
  r12 = rotationMatrix.at<double>(0, 1);
  r13 = rotationMatrix.at<double>(0, 2) + t_col;
  r21 = rotationMatrix.at<double>(1, 0);
  r22 = rotationMatrix.at<double>(1, 1);
  r23 = rotationMatrix.at<double>(1, 2) + t_row;

  cv::Mat ref2cur_R, ref2cur_T;
  ref2cur_R = cv::Mat::eye(3, 3, CV_64FC1);
  ref2cur_T = cv::Mat::zeros(3, 1, CV_64FC1);

  ref2cur_R.at<double>(0, 0) = r11;
  ref2cur_R.at<double>(0, 1) = r12;
  ref2cur_R.at<double>(1, 0) = r21;
  ref2cur_R.at<double>(1, 1) = r22;

  ref2cur_T.at<double>(0) = r13;
  ref2cur_T.at<double>(1) = r23;

  //  Point2f p1_in_ref(0.f, 0.f), p2_in_ref(cols_, 0.f), p3_in_ref(cols_,
  //  rows_);
  //  const RotatedRect ref_rect(p1_in_ref, p2_in_ref, p3_in_ref);

  const Point2f center_in_ref(cols_ / 2.f, rows_ / 2.f);
  const Size2f size(cols_, rows_);
  const RotatedRect ref_rect(center_in_ref, size, 0.f);

  //  std::cout << "center_in_ref: (" << center_in_ref.x << ", " <<
  //  center_in_ref.y << ") - (" << ref_rect.center.x << ", " <<
  //  ref_rect.center.y << ")\n";
  //  std::cout << "size: (" << size.width << ", " << size.height << ") - (" <<
  //  ref_rect.size.width << ", " << ref_rect.size.height << ")\n";
  //  std::cout << "angle: " << 0 << " - " << ref_rect.angle << std::endl;

  //  Point2f p1_in_cur, p2_in_cur, p3_in_cur;
  //  p1_in_cur.x = r11*p1_in_ref.x+r12*p1_in_ref.y+r13;
  //  p1_in_cur.y = r21*p1_in_ref.x+r22*p1_in_ref.y+r23;
  //  p2_in_cur.x = r11*p2_in_ref.x+r12*p2_in_ref.y+r13;
  //  p2_in_cur.y = r21*p2_in_ref.x+r22*p2_in_ref.y+r23;
  //  p3_in_cur.x = r11*p3_in_ref.x+r12*p3_in_ref.y+r13;
  //  p3_in_cur.y = r21*p3_in_ref.x+r22*p3_in_ref.y+r23;
  // const RotatedRect cur_rect(p1_in_cur, p2_in_cur, p3_in_cur);

  Point2f center_in_cur;
  center_in_cur.x = r11 * center_in_ref.x + r12 * center_in_ref.y + r13;
  center_in_cur.y = r21 * center_in_ref.x + r22 * center_in_ref.y + r23;
  const RotatedRect cur_rect(center_in_cur, size, -rotation);

  //  std::cout << "center_in_cur: (" << center_in_cur.x << ", " <<
  //  center_in_cur.y << ") - (" << cur_rect.center.x << ", " <<
  //  cur_rect.center.y << ")\n";
  //  std::cout << "size: (" << size.width << ", " << size.height << ") - (" <<
  //  cur_rect.size.width << ", " << cur_rect.size.height << ")\n";
  //  std::cout << "angle: " << rotation << " - " << cur_rect.angle <<
  //  std::endl;

  std::vector<cv::Point2f> intersectingRegion;
  // int intersection_type = cv::rotatedRectangleIntersection(ref_rect,
  // cur_rect, intersectingRegion);
  int intersection_type = cvfix_rotatedRectangleIntersection(
      ref_rect, cur_rect, intersectingRegion);
  if (intersection_type == cv::RectanglesIntersectTypes::INTERSECT_NONE)
  {
    response = 0.;
  }
  else
  {
    std::vector<cv::Point2f> intersectingContour;
    cv::convexHull(intersectingRegion, intersectingContour);
    overlap = cv::contourArea(intersectingContour) / (rows_ * cols_);

//    std::cout << "overlap: " << overlap << std::endl;
//    std::cout << "response: " << response << std::endl;

    response /= overlap;

    //    std::cout << "overlap_ratio1: " << overlap << std::endl;
    //    std::cout << "overlap_ratio2: " << rows_*cols_ << std::endl;
    // std::cout << "overlap_ratio: " << overlap/(rows_*cols_) << std::endl;
  }

  //  for(const cv::Point2f& pt : intersectingRegion)
  //    std::cout << pt.x << ", " << pt.y << std::endl;

  cur2ref_R = ref2cur_R.inv();
  cur2ref_T = -cur2ref_R * ref2cur_T;

  if (display_images)
  {
    // cv::imshow("im0_rotated", im0_rotated_);

    std::cout << "x: " << t_col << ", y: " << t_row
              << ", rotation: " << rotation << ", scale: " << scale
              << std::endl;

    cv::Mat registered_image2;
    cv::Mat registered_mat_2d = cv::Mat::zeros(2, 3, CV_64FC1);
    registered_mat_2d.at<double>(0, 0) = cur2ref_R.at<double>(0, 0);
    registered_mat_2d.at<double>(0, 1) = cur2ref_R.at<double>(0, 1);
    registered_mat_2d.at<double>(1, 0) = cur2ref_R.at<double>(1, 0);
    registered_mat_2d.at<double>(1, 1) = cur2ref_R.at<double>(1, 1);
    registered_mat_2d.at<double>(0, 2) = cur2ref_T.at<double>(0);
    registered_mat_2d.at<double>(1, 2) = cur2ref_T.at<double>(1);

    cv::Mat im0_show, im1_show;
    im0_gray_.convertTo(im0_show, CV_8UC1, 255.);
    im1_gray_.convertTo(im1_show, CV_8UC1, 255.);

    cv::warpAffine(im1_show, registered_image2, registered_mat_2d,
                   im1_show.size());

    cv::Mat overlay_image2;
    cv::addWeighted(im0_show, 0.5, registered_image2, 0.5, 0.0, overlay_image2);

    cv::imwrite(std::string(getenv("HOME")) + "/test/im1.bmp", im1_show);
    cv::imwrite(std::string(getenv("HOME")) + "/test/overlay_image2.bmp",
                overlay_image2);
    // cv::imwrite("/home/miow/test/result.bmp", im0_gray_);

    //    cv::imshow("im1_gray_", im1_gray_);
    //    cv::imshow("im0_registered", overlay_image2);

    //    cv::Mat overlay_image;
    //    cv::addWeighted(getCurrentImage(), 0.5, registered_image, 0.5, 0.0,
    //                    overlay_image);
    //    overlay_image.convertTo(overlay_image, CV_8UC1, 255.);
    //    cv::imwrite("/home/miow/test/overlay_image.bmp", overlay_image);
    // cv::imshow("overlay_image", overlay_image);

    //    cv::waitKey(0);
  }
#ifdef TIME_TEST
  std::cout << "Part5 time used: " << t.stop() * 1e3 << " ms.\n";
#endif

  return response;
}

void ImageRegistration::next()
{
  im1_logpolar_.copyTo(im0_logpolar_);
  im1_gray_.copyTo(im0_gray_);
}

cv::Mat ImageRegistration::getBorderMask()
{
  return image_transforms_.getBorderMask();
}

//----------------------------------------------------------
// Recombinate image quaters
//----------------------------------------------------------
void Recomb(Mat& src, Mat& dst)
{
  int cx = src.cols >> 1;
  int cy = src.rows >> 1;
  Mat tmp;
  tmp.create(src.size(), src.type());
  src(Rect(0, 0, cx, cy)).copyTo(tmp(Rect(cx, cy, cx, cy)));
  src(Rect(cx, cy, cx, cy)).copyTo(tmp(Rect(0, 0, cx, cy)));
  src(Rect(cx, 0, cx, cy)).copyTo(tmp(Rect(0, cy, cx, cy)));
  src(Rect(0, cy, cx, cy)).copyTo(tmp(Rect(cx, 0, cx, cy)));
  dst = tmp;
}
//----------------------------------------------------------
// 2D Forward FFT
//----------------------------------------------------------
void ForwardFFT(Mat& Src, Mat* FImg, bool do_recomb = true)
{
  //  int M = getOptimalDFTSize(Src.rows);
  //  int N = getOptimalDFTSize(Src.cols);
  //  Mat padded;
  //  if (M != Src.rows || N != Src.cols)
  //    copyMakeBorder(Src, padded, 0, M - Src.rows, 0, N - Src.cols,
  //                   BORDER_CONSTANT, Scalar::all(0));
  //  else
  //    padded = Src;

  int M = Src.rows;
  int N = Src.cols;
  Mat padded = Src;

  Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
  Mat complexImg;

  merge(planes, 2, complexImg);
  dft(complexImg, complexImg);
  split(complexImg, planes);

  //  Mat planes_copy[2];
  //  planes[0].convertTo(planes_copy[0], CV_8U, 1.0);
  //  cv::imshow("planes[0]", planes_copy[0]);
  //  planes[1].convertTo(planes_copy[1], CV_8U, 1.0);
  //  cv::imshow("planes[1]", planes_copy[1]);

  //  std::ofstream fout("/home/miow/test/b.txt");
  //  for(int i = 0;i < planes[0].rows;i++)
  //  {
  //    for(int j = 0;j < planes[0].cols;j++)
  //    {
  //      fout << planes[0].at<float>(i, j) << '\t';
  //    }
  //    fout << '\n';
  //  }
  //  fout.close();
  //  exit(0);

  planes[0] = planes[0](Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
  planes[1] = planes[1](Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));
  if (do_recomb)
  {
    Recomb(planes[0], planes[0]);
    Recomb(planes[1], planes[1]);
  }
  planes[0] /= float(M * N);
  planes[1] /= float(M * N);
  FImg[0] = planes[0].clone();
  FImg[1] = planes[1].clone();
}

//-----------------------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------------------
void highpass(Size sz, Mat& dst)
{
  Mat a = Mat(sz.height, 1, CV_32FC1);
  Mat b = Mat(1, sz.width, CV_32FC1);

  float step_y = CV_PI / sz.height;
  float val = -CV_PI * 0.5;

  for (int i = 0; i < sz.height; ++i)
  {
    a.at<float>(i) = cos(val);
    val += step_y;
  }

  val = -CV_PI * 0.5;
  float step_x = CV_PI / sz.width;
  for (int i = 0; i < sz.width; ++i)
  {
    b.at<float>(i) = cos(val);
    val += step_x;
  }

  Mat tmp = a * b;
  dst = (1.0 - tmp).mul(2.0 - tmp);
}

void ImageRegistration::processImage(const cv::Mat& im, cv::Mat& gray,
                                     cv::Mat& log_polar)
{
  //  im.convertTo(gray, CV_32F, 1.0);

  im.convertTo(gray, CV_32F, 1.0 / 255.0);
  //  cv::cvtColor(gray, gray, CV_BGR2GRAY);

  cv::Mat apodized;
  // cv::Mat im_dft_cv;

  image_transforms_.apodize(gray, apodized);

  apodized.convertTo(apodized, CV_8U, 255.0);
  //  cv::imwrite("/home/miow/test/a.bmp", apodized);

  Mat F0[2], f0;
  ForwardFFT(apodized, F0);
  magnitude(F0[0], F0[1], f0);

  Mat h_;
  // Create filter
  highpass(f0.size(), h_);

  // Apply it in freq domain
  f0 = f0.mul(h_);

  //  std::ofstream fout("/home/miow/test/3_1.txt");
  //  for(int i = 0;i < F0[0].rows;i++)
  //  {
  //    for(int j = 0;j < F0[0].cols;j++)
  //    {
  //      fout << F0[0].at<float>(i, j) << '\t';
  //    }
  //    fout << '\n';
  //  }
  //  fout.close();
  //  exit(0);

  image_transforms_.remapLogPolar(f0, log_polar);

  //  Mat h;
  //  cv::eigen2cv(high_pass_filter_, h);
  //  cv::imshow("h", h);
  //  cv::waitKey(0);
  //  // Apply it in freq domain
  //  f0 = f0.mul(h);
  //  f0.convertTo(f0, CV_8U, 255.0);
  //  cv::imshow("f0", f0);
  //  cv::waitKey(0);

  //  std::ofstream fout("/home/miow/test/1.txt");

  //  ComplexMatrix im_dft2 = imdft_.fft(apodized);
  //  Mat real_part = Mat::zeros(im_dft2.rows(), im_dft2.cols(), CV_32FC1);
  //  for(int i = 0;i < im_dft2.rows();i++)
  //  {
  //    for(int j = 0;j < im_dft2.cols();j++)
  //    {
  //      std::complex<double> num = im_dft2(i, j);
  //      fout << "(" << num.real() << "," << num.imag() << ")\t";

  //      real_part.at<float>(i,j) = num.real();
  //    }
  //    fout << '\n';
  //  }
  //  fout.close();
  ////  cv::eigen2cv(im_dft2.real(), im_dft_cv);
  //  real_part.convertTo(real_part, CV_8U, 1.0);
  //  cv::imshow("real_part", real_part);
  //  cv::waitKey(0);

  //  Eigen::MatrixXf im_dft = (imdft_.fftShift(imdft_.fft(apodized))
  //                                .cwiseProduct(high_pass_filter_)
  //                                .cwiseAbs()).cast<float>();
  //  cv::eigen2cv(im_dft, im_dft_cv);
  //  image_transforms_.remapLogPolar(im_dft_cv, log_polar);
}

cv::Mat ImageRegistration::getPreviousImage() { return im0_gray_; }

cv::Mat ImageRegistration::getCurrentImage() { return im1_gray_; }
