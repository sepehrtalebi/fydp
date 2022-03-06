#pragma once

#include <bmb_math/Quaternion.h>
#include <bmb_math/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

template <typename T>
ignition::math::Vector3<T> bmbToIgnitionVector3(const Vector3<T>& vec) {
  ignition::math::Vector3<T> result;
  result.X(vec.x);
  result.Y(vec.y);
  result.Z(vec.z);
  return result;
}

template <typename T>
geometry_msgs::Vector3 ignitionToGeometryVector3(const T& vec) {
  geometry_msgs::Vector3 result;
  result.x = vec.X();
  result.y = vec.Y();
  result.z = vec.Z();
  return result;
}

template <typename T>
geometry_msgs::Vector3 ignitionToGeometryPoint(const T& vec) {
  geometry_msgs::Vector3 result;
  result.x = vec.X();
  result.y = vec.Y();
  result.z = vec.Z();
  return result;
}

template <typename T>
geometry_msgs::Quaternion ignitionToGeometryQuaternion(const T& quat) {
  geometry_msgs::Quaternion result;
  result.w = quat.W();
  result.x = quat.X();
  result.y = quat.Y();
  result.z = quat.Z();
  return result;
}
