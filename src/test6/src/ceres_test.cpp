// ceres_test.cpp : 定义控制台应用程序的入口点。
//

#include "distance_option.h"

using namespace std;
using namespace ceres;
int main()
{
  Eigen::Vector2d mark_tranlation(10, 5);
  Problem ploblem;
  Eigen::Vector2d mark(1,2);
  for(int k =0; k < 100; k++){
    for (int i = 10; i < 100; i++)
    {
      Eigen::Vector2d sensor2global_t(0.1* i + k *10, 0.01 * i+ k*0.1 + 3);
      DistaceObservation observation;
      float temp_err = rand() % 10 * 0.01*2;
  //     cout << temp << endl;
      observation.mark_id = 1;
      observation.weight = 10;
      observation.distance = (mark_tranlation - sensor2global_t).norm() + temp_err;
  //     cout << observation.distance << endl;
      ploblem.AddResidualBlock(
        DistaceMarkCostFunction::CreateAutoDiffCostFuction(observation, sensor2global_t),
        nullptr, mark.data());
    }
    ceres::Solver::Options options;
    options.max_num_iterations = 10000;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-8;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &ploblem, &summary);
    cout << mark[0] << ", " << mark[1] << endl;
  }
//   cout <<summary.BriefReport()<<endl;
  return 0;
}

