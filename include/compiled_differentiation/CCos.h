#pragma once

#include <cmath>
#include <type_traits>

namespace compiled {

template <typename T>
struct cosine {
  using type = cosine<T>;
  using operand = T;

  static inline double apply(const double* variables) {
    return std::cos(T::apply(variables));
  }
};

template <typename T>
using cosine_t = typename cosine<T>::type;

template<typename T>
struct is_cosine : std::false_type {};

template<typename T>
struct is_cosine<cosine<T>> : std::true_type {};

template<typename T>
inline constexpr bool is_cosine_v = is_cosine<T>::value;

}  // namespace compiled
