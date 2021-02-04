#ifndef UNITY_H
#define UNITY_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include <Eigen/Eigen>
#include <ros/ros.h>

template <typename FloatType>
class Rigid3 {
 public:
  using Vector = Eigen::Matrix<FloatType, 3, 1>;
  using Quaternion = Eigen::Quaternion<FloatType>;
  using AngleAxis = Eigen::AngleAxis<FloatType>;

  Rigid3() : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}
  Rigid3(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid3(const Vector& translation, const AngleAxis& rotation)
      : translation_(translation), rotation_(rotation) {}

  static Rigid3 Rotation(const AngleAxis& angle_axis) {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }

  static Rigid3 Rotation(const Quaternion& rotation) {
    return Rigid3(Vector::Zero(), rotation);
  }

  static Rigid3 Translation(const Vector& vector) {
    return Rigid3(vector, Quaternion::Identity());
  }

  static Rigid3 FromArrays(const std::array<FloatType, 4>& rotation,
                           const std::array<FloatType, 3>& translation) {
    return Rigid3(Eigen::Map<const Vector>(translation.data()),
                  Eigen::Quaternion<FloatType>(rotation[0], rotation[1],
                                               rotation[2], rotation[3]));
  }

  static Rigid3<FloatType> Identity() { return Rigid3<FloatType>(); }

  template <typename OtherType>
  Rigid3<OtherType> cast() const {
    return Rigid3<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }
  const Quaternion& rotation() const { return rotation_; }

  Rigid3 inverse() const {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);
  }

  std::string DebugString() const {
    std::string str = "{ t: [" + std::to_string(translation().x()) + ", " 
      + std::to_string(translation().y()) + ", " 
      + std::to_string(translation().z()) + "], q: [" 
      + std::to_string(rotation().w()) + ", " 
      + std::to_string(rotation().x()) + ", "
      + std::to_string(rotation().y()) + ", "
      + std::to_string(rotation().z()) + "] }";
    return str;
  }

  bool IsValid() const {
    return !std::isnan(translation_.x()) && !std::isnan(translation_.y()) &&
           !std::isnan(translation_.z()) &&
           std::abs(FloatType(1) - rotation_.norm()) < FloatType(1e-3);
  }

 private:
  Vector translation_;
  Quaternion rotation_;
};
template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
                            const Rigid3<FloatType>& rhs) {
  return Rigid3<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      (lhs.rotation() * rhs.rotation()).normalized());
}

template <typename FloatType>
typename Rigid3<FloatType>::Vector operator*(
    const Rigid3<FloatType>& rigid,
    const typename Rigid3<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const Rigid3<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

using Rigid3d = Rigid3<double>;
using Rigid3f = Rigid3<float>;
class Unity{
private:
  ros::NodeHandle nh_;
  std::map<std::string, ros::Publisher> publishers_;
public:
  Unity(){}
  ~Unity(){}
protected:
  template <typename T>
  void PubMsg(const std::string& topic, const T& msg)
  {
    if(publishers_.find(topic) == publishers_.end())
    {
      publishers_[topic] = nh_.advertise<T>(topic, 3);
    }
    publishers_[topic].publish(msg);
  }

};
#endif //UNITY_H