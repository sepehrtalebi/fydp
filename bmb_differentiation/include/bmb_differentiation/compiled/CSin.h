#pragma once

#include <array>
#include <cmath>
#include <type_traits>

namespace compiled {

template <typename T>
struct sine {
  using type = sine<T>;
  using operand = T;

  template <size_t n>
  static inline double apply(const std::array<double, n>& variables) {
    return std::sin(T::apply(variables));
  }
};

template <typename T>
using sine_t = typename sine<T>::type;

/**
 * Returns true if T is an instance of sine and false otherwise.
 */
template <typename T>
struct is_sine : std::false_type {};

template <typename T>
struct is_sine<sine<T>> : std::true_type {};

template <typename T>
inline constexpr bool is_sine_v = is_sine<T>::value;

}  // namespace compiled
