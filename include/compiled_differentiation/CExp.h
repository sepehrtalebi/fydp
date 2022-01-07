#pragma once

#include <cmath>
#include <type_traits>

namespace compiled {

template <typename T>
struct exponential {
  using type = exponential<T>;
  using operand = T;

  static inline double apply(const double* variables) {
    return std::exp(T::apply(variables));
  }
};

template <typename T>
using exponential_t = typename exponential<T>::type;


template<typename T>
struct is_exponential : std::false_type {};

template<typename T>
struct is_exponential<exponential<T>> : std::true_type {};

template<typename T>
inline constexpr bool is_exponential_v = is_exponential<T>::value;

}  // namespace compiled
