/*
 * Copyright 2020 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "laser_matcher.h"
#include <boost/assign.hpp>

using namespace std;
LaserMatcher::LaserMatcher()
{
  initParams();
  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;

  // Initialize output_ vectors as Null for error-checking
  output_.cov_x_m = 0;
  output_.dx_dy1_m = 0;
  output_.dx_dy2_m = 0;
  pr_ch_x_ = 0;
  pr_ch_y_ = 0;
  pr_ch_a_ = 0;
  
  scan1_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan1",1);
  scan2_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan2",1);
}

LaserMatcher::~LaserMatcher()
{

}
void LaserMatcher::initParams()
{
  // Maximum angular displacement between scans
  input_.max_angular_correction_deg = 45.0;

  // Maximum translation between scans (m)
  input_.max_linear_correction = 0.50;

  // Maximum ICP cycle iterations
  input_.max_iterations = 100;

  // A threshold for stopping (m)
  input_.epsilon_xy = 0.000001;

  // A threshold for stopping (rad)
  input_.epsilon_theta = 0.000001;

  // Maximum distance for a correspondence to be valid
  input_.max_correspondence_dist = 0.3;

  // Noise in the scan (m)
  input_.sigma = 0.010;

  // Use smart tricks for finding correspondences.
  input_.use_corr_tricks = 1;

  // Restart: Restart if error is over threshold
  input_.restart = 0;

  // Restart: Threshold for restarting
  input_.restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting. (m)
  input_.restart_dt = 1.0;

  // Restart: displacement for restarting. (rad)
  input_.restart_dtheta = 0.1;

  // Max distance for staying in the same clustering
  input_.clustering_threshold = 0.25;

  // Number of neighbour rays used to estimate the orientation
  input_.orientation_neighbourhood = 20;

  // If 0, it's vanilla ICP
  input_.use_point_to_line_distance = 1;

  // Discard correspondences based on the angles
  input_.do_alpha_test = 0;

  // Discard correspondences based on the angles - threshold angle, in degrees
  input_.do_alpha_test_thresholdDeg = 20.0;

  // Percentage of correspondences to consider: if 0.9,
  // always discard the top 10% of correspondences with more error
  input_.outliers_maxPerc = 0.90;

  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  input_.outliers_adaptive_order = 0.7;

  input_.outliers_adaptive_mult = 2.0;

  // If you already have a guess of the solution, you can compute the polar angle
  // of the points of one scan in the new position. If the polar angle is not a monotone
  // function of the readings index, it means that the surface is not visible in the
  // next position. If it is not visible, then we don't use it for matching.
  input_.do_visibility_test = 0;

  // no two points in laser_sens can have the same corr.
  input_.outliers_remove_doubles = 1;

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  input_.do_compute_covariance = 0;

  // Checks that find_correspondences_tricks gives the right answer
  input_.debug_verify_tricks = 0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  input_.use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the
  // correspondence by 1/sigma^2
  input_.use_sigma_weights = 0;
  
  input_.min_reading = 0;
  input_.max_reading = 10;
}

void LaserMatcher::createTfFromXYTheta(
  double x, double y, double theta, tf::Transform& t)
{
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}

bool LaserMatcher::computePose(const LDP &ldp1,const LDP &ldp2,tf::Transform &f2l)
{
  ldp1->odometry[0] = 0.0;
  ldp1->odometry[1] = 0.0;
  ldp1->odometry[2] = 0.0;

  ldp1->estimate[0] = 0.0;
  ldp1->estimate[1] = 0.0;
  ldp1->estimate[2] = 0.0;

  ldp1->true_pose[0] = 0.0;
  ldp1->true_pose[1] = 0.0;
  ldp1->true_pose[2] = 0.0;

  input_.laser_ref  = ldp1;
  input_.laser_sens = ldp2;

  // **** estimated change since last scan
  pr_ch_x_ = 0;pr_ch_y_ = 0;pr_ch_a_ = 0;
  tf::Transform pr_ch;
  createTfFromXYTheta(pr_ch_x_, pr_ch_y_, pr_ch_a_, pr_ch);

  input_.first_guess[0] = pr_ch.getOrigin().getX();
  input_.first_guess[1] = pr_ch.getOrigin().getY();
  input_.first_guess[2] = tf::getYaw(pr_ch.getRotation());
  // If they are non-Null, free covariance gsl matrices to avoid leaking memory
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = 0;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = 0;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = 0;
  }

  // *** scan match - using point to line icp from CSM
  sm_icp(&input_, &output_);
  tf::Transform corr_ch;
  std::cout << "6"<<std::endl;
  if (output_.valid)
  {
    createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch);
    //pr_ch_x_ = output_.x[0];pr_ch_y_ = output_.x[1];pr_ch_a_ = output_.x[2];
    
    if(output_.error > 7.80){
      std::cout << "***************************too big error !!!***************************"<<std::endl;
      return false;
    }
    f2l = corr_ch;
//    std::cout <<"output_.error and result: " <<output_.cov_x_m<<",   "<<std::endl<< f2l.getOrigin().x()<<","<<f2l.getOrigin().y()<<","<<f2l.getOrigin().z()<<std::endl;
    return true;
  }
  else{
    std::cout << "***************************not valid !!!***************************"<<std::endl;
    return false;
  }
}


void LaserMatcher::laserScanToLDP(const sensor_msgs::LaserScan& scan_msg,
                                            LDP& ldp)
{
  unsigned int n = scan_msg.ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame

    double r = scan_msg.ranges[i];

    if (r > scan_msg.range_min && r < scan_msg.range_max)
    {
      // fill in laser scan data

      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }

    ldp->theta[i]    = scan_msg.angle_min + i * scan_msg.angle_increment;
//    show2_.inputScanPoints("2",r,ldp->theta[i],50);
    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void LaserMatcher::match(sensor_msgs::LaserScan scan1_msg, 
                         sensor_msgs::LaserScan scan2_msg, 
                         tf::Transform& f2l)
{
  LDP ldp1, ldp2;

  laserScanToLDP(scan1_msg,ldp1);
  laserScanToLDP(scan2_msg,ldp2);
  computePose(ldp1,ldp2,f2l);
 
  scan1_msg.header.frame_id = "laser_link1";
  scan2_msg.header.frame_id = "laser_link2";
  scan1_pub_.publish(scan1_msg);
  scan2_pub_.publish(scan2_msg);
  
  broadcaster_.sendTransform(
      tf::StampedTransform(
        f2l, ros::Time::now(), "laser_link1", "laser_link2"));
}

std::unique_ptr<LaserMatcher> matcher;
static int k = 0;
sensor_msgs::LaserScan msg1;
void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if(k == 0)
    msg1 = *msg;
  else if(k % 5 == 0)
  {
    tf::Transform tf;
    tf.setIdentity();
    matcher->match(msg1,*msg,tf);
    msg1 = *msg;
    cout << tf.getOrigin() <<endl;
  }
  k++;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_matcher");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan_emma_nav_front",5,scanCallBack);
  matcher.reset(new LaserMatcher);
  
  ros::spin();
  
  return 1;
}