#pragma once

#include <array>
#include <cmath>
#include <type_traits>

namespace compiled {

template <typename T>
struct exponential {
  using type = exponential<T>;
  using operand = T;

  template <size_t n>
  static inline double apply(const std::array<double, n>& variables) {
    return std::exp(T::apply(variables));
  }
};

template <typename T>
using exponential_t = typename exponential<T>::type;

/**
 * Returns true if T is an instance of exponential and false otherwise.
 */
template <typename T>
struct is_exponential : std::false_type {};

template <typename T>
struct is_exponential<exponential<T>> : std::true_type {};

template <typename T>
inline constexpr bool is_exponential_v = is_exponential<T>::value;

}  // namespace compiled
