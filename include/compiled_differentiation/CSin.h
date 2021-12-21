#pragma once

#include <cmath>

namespace compiled {

template <typename T>
struct sine {
  using type = sine<T>;

  static inline double apply(const double* variables) {
    return std::sin(T::apply(variables));
  }
};

template <typename T>
using sine_t = typename sine<T>::type;

template<typename T>
struct is_sine : std::false_type {};

template<typename T>
struct is_sine<sine<T>> : std::true_type {};

template<typename T>
inline constexpr bool is_sine_v = is_sine<T>::value;

}  // namespace compiled
