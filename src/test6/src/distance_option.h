#ifndef DISTANCE_OPTION_H_
#define DISTANCE_OPTION_H_

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/autodiff_cost_function.h>
#include "unity.hpp"
struct DistaceObservation
{
  int mark_id;
  double distance;
  double weight;
};
class DistaceMarkCostFunction {
public:
  static ceres::CostFunction* CreateAutoDiffCostFuction(
    const DistaceObservation& observation, const Eigen::Vector2d cur_translation) {
    return new ceres::AutoDiffCostFunction<DistaceMarkCostFunction, 
      1, 
      2>(new DistaceMarkCostFunction(observation, cur_translation));
  }

  template <typename T>
  bool operator()(const T* const landmark_translation, T* const e) const {
    const Eigen::Matrix<T, 2, 1> sensor2global_t(
      T(cur_translation_(0)),
      T(cur_translation_(1)));
    const T error = (sensor2global_t[0] - landmark_translation[0]) * (sensor2global_t[0] - landmark_translation[0])
      + (sensor2global_t[1] - landmark_translation[1]) * (sensor2global_t[1] - landmark_translation[1]) - T(distance_);
    e[0] = error * weight_;
    return true;
  }
private:
  DistaceMarkCostFunction(const DistaceObservation& observation, 
    const Eigen::Vector2d cur_translation)
    : distance_(observation.distance * observation.distance),
    weight_(observation.weight),
    cur_translation_(cur_translation){}
  const double distance_;
  const double weight_;
  const Eigen::Vector2d cur_translation_;
};
#endif