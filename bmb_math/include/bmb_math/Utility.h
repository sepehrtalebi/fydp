#pragma once

#include <cstddef>

namespace bmb_math {

template<typename T>
Matrix<T, 2, 2> getRotationMatrix(const T& theta) {
  T sin = std::sin(theta);
  T cos = std::cos(theta);
  return {cos, -sin,
          sin, cos};
}

/**
 * @brief Computes a rotation matrix that would rotate Vector<T, 2>{1, 0} to point in the same direction as the given vec
 */
template<typename T>
Matrix<T, 2, 2> getRotationMatrix(const Vector<T, 2>& vec) {
  T hypot = vec.magnitude();
  T sin = vec[1] / hypot;
  T cos = vec[0] / hypot;
  return {cos, -sin,
          sin, cos};
}

}  // namespace bmb_math
