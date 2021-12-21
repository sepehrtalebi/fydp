#pragma once

#include <cmath>

namespace compiled {

template <typename B, typename E>
struct power {
  using type = power<B, E>;

  static inline double apply(const double* variables) {
    return std::pow(B::apply(variables), E::apply(variables));
  }
};

template <typename B, typename E>
using power_t = typename power<B, E>::type;

template<typename T>
struct is_power : std::false_type {};

template<typename B, typename E>
struct is_power<power<B, E>> : std::true_type {};

template<typename T>
inline constexpr bool is_power_v = is_power<T>::value;

}  // namespace compiled
