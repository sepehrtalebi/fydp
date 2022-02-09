#pragma once

#include <array>
#include <cmath>
#include <type_traits>

namespace compiled {

template <typename T>
struct logarithm {
  using type = logarithm<T>;
  using operand = T;

  template <size_t n>
  static inline double apply(const std::array<double, n>& variables) {
    return std::log(T::apply(variables));
  }
};

template <typename T>
using logarithm_t = typename logarithm<T>::type;

/**
 * Returns true if T is an instance of logarithm and false otherwise.
 */
template<typename T>
struct is_logarithm : std::false_type {};

template<typename T>
struct is_logarithm<logarithm<T>> : std::true_type {};

template<typename T>
inline constexpr bool is_logarithm_v = is_logarithm<T>::value;

}  // namespace compiled
