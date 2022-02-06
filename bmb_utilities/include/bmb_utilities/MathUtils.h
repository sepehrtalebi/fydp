#pragma once

#include <cmath>
#include <type_traits>

namespace bmb_utilities {

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
constexpr auto squared(const T& val) {
    return val * val;
}

static constexpr size_t slice_count(const size_t& start, const size_t& stop,
                                    const size_t& step = 1) {
  return (stop - start + step - 1) / step;
}

template<typename T>
constexpr T abs_difference(const T& a, const T&b) {
  return a > b ? a - b : b - a;
}

template<typename T>
constexpr T heaviside_difference(const T& a, const T&b) {
  return a > b ? a - b : 0;
}

}
