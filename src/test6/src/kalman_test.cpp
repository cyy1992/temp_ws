/*
 * Copyright 2021 <copyright holder> <email>
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

#include "kalman_test.h"
using namespace cv;
using namespace std;
static inline Point calcPoint(Point2f center, double R, double angle)
{
  return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
}
KalmanTest::KalmanTest()
{
  fstream distances_file("/home/cyy/distances.txt");
  vector<double> distances;
  if(distances_file.is_open())
  {
    while(!distances_file.eof())
    {
      double dis;
      distances_file >> dis;
      distances.push_back(dis);
    }
  }
  
  fstream det_times_file("/home/cyy/det_times.txt");
  vector<double> det_times;
  if(det_times_file.is_open())
  {
    while(!det_times_file.eof())
    {
      double time;
      det_times_file >> time;
      det_times.push_back(time);
    }
  }
  
  char code = (char)-1;
  cout << code<<endl;
  for(;;){
    KalmanFilter KF(3, 1, 0);
    Mat state(3, 1, CV_32F); /* (phi, delta_phi) */
//     Mat processNoise(2, 1, CV_32F);
    Mat measurement = Mat::zeros(1, 1, CV_32F);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(10));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(1));
    state.at<float>(0) = distances[0];
    state.at<float>(1) = 0;
    state.at<float>(2) = 0;
    KF.statePost = state;

    Mat img(500, 2000, CV_8UC3);
    for(int i =1; i < det_times.size(); i++)
    {
      cout << det_times[i] << ", " << distances[i] <<endl;
      KF.transitionMatrix = (Mat_<float>(3, 3) << 1, det_times[i], 0.5 * det_times[i]*det_times[i],0, 1,det_times[i],0,0,1);
      measurement = distances[i];
//       Point2f center(img.cols*0.5f, img.rows*0.5f);
//       float R = img.cols/3.f;
//       double stateAngle = state.at<float>(0);
//       Point statePt = calcPoint(center, R, stateAngle);
      Mat prediction = KF.predict();
//       double predictAngle = prediction.at<float>(0);
//       Point predictPt = calcPoint(center, R, predictAngle);
//       measurement += KF.measurementMatrix*state;
      if(theRNG().uniform(0,4) != 0)
        KF.correct(measurement);
//       state = KF.transitionMatrix*state;
//       img = Scalar::all(0);
//       double measAngle = measurement.at<float>(0);
//       Point measPt = calcPoint(center, R, measAngle);
      #define drawCross( center, color, d )                                        \
      line( img, Point( center.x - d, center.y - d ),                          \
      Point( center.x + d, center.y + d ), color, 1, LINE_AA, 0); \
      line( img, Point( center.x + d, center.y - d ),                          \
      Point( center.x - d, center.y + d ), color, 1, LINE_AA, 0 )
      //     drawCross( statePt, Scalar(255,255,255), 3 );
      //     drawCross( measPt, Scalar(0,0,255), 3 );
      //     drawCross( predictPt, Scalar(0,255,0), 3 );
      
      //     line( img, statePt, measPt, Scalar(0,0,255), 3, LINE_AA, 0 );
      //     line( img, statePt, predictPt, Scalar(0,255,255), 3, LINE_AA, 0 );
      circle(img, Point(i, KF.statePost.at<float>(0)), 1,Scalar(0,0,255));
      circle(img, Point(i, KF.statePost.at<float>(0)), 1,Scalar(0,0,255));
      cout << KF.statePost.at<float>(0) <<endl;
      imshow( "Kalman", img );
      code = (char)waitKey(50);
      if( code > 0 )
        break;
    }
    if( code == 27 || code == 'q' || code == 'Q' )
      break;
  }
}

KalmanTest::~KalmanTest()
{
  
}


int main(int, char**)
{
  KalmanTest k;
  return 1;
}