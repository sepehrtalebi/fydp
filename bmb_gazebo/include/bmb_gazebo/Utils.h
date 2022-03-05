#pragma once

#include <bmb_math/Quaternion.h>
#include <bmb_math/Vector3.h>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

template <typename T>
Vector3<T> asBMBVector3(const ignition::math::Vector3<T>& vec) {
  return {vec.X(), vec.Y(), vec.Z()};
}

template <typename T>
ignition::math::Vector3<T> asIgnitionVector3(const Vector3<T>& vec) const {
  ignition::math::Vector3<T> result;
  result.X(vec.x);
  result.Y(vec.y);
  result.Z(vec.z);
  return result;
}

// allow implicit conversions
template <typename T>
Quaternion<T> asBMBQuaternion(const ignition::math::Quaternion<T>& quat) {
  return {quat.W(), quat.X(), quat.Y(), quat.Z()};
}

template<typename T>
ignition::math::Quaternion<T> copy_to(const Quaternion<T>& quat) {
  ignition::math::Quaternion result;
  result.W(quat.q0);
  result.X(quat.q1);
  result.Y(quat.q2);
  result.Z(quat.q3);
  return result;
}
