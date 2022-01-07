#pragma once

#include <cmath>
#include <type_traits>

namespace compiled {

template <typename T>
struct logarithm {
  using type = logarithm<T>;
  using operand = T;

  static inline double apply(const double* variables) {
    return std::log(T::apply(variables));
  }
};

template <typename T>
using logarithm_t = typename logarithm<T>::type;

template<typename T>
struct is_logarithm : std::false_type {};

template<typename T>
struct is_logarithm<logarithm<T>> : std::true_type {};

template<typename T>
inline constexpr bool is_logarithm_v = is_logarithm<T>::value;

}  // namespace compiled
