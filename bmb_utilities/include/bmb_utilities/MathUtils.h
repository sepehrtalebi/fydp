#pragma once

#include <bmb_math/Vector.h>
#include <bmb_math/Matrix.h>
#include <cmath>
#include <type_traits>

namespace bmb_utilities {

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

template <typename T1, typename T2>
std::common_type_t<T1, T2> saturation(const T1& value, const T2& limit) {
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

template <typename T1, typename T2, typename T3>
std::common_type_t<T1, T2, T3> saturation(const T1& value, const T2& min, const T3& max) {
  if (value > max) return max;
  if (value < min) return min;
  return value;
}

template <typename T>
T squared(const T& val) {
    return val * val;
}

}
