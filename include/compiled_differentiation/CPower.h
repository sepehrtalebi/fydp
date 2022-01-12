#pragma once

#include <array>
#include <cmath>
#include <type_traits>

namespace compiled {

template <typename B, typename E>
struct power {
  using type = power<B, E>;
  using base = B;
  using exponent = E;

  template <size_t n>
  static inline double apply(const std::array<double, n>& variables) {
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

template<typename T>
struct get_base {
  using type = void;
};

template<typename B, typename E>
struct get_base<power<B, E>> {
  using type = B;
};

template<typename T>
using get_base_t = typename get_base<T>::type;

template<typename T>
struct get_exponent {
  using type = void;
};

template<typename B, typename E>
struct get_exponent<power<B, E>> {
  using type = E;
};

template<typename T>
using get_exponent_t = typename get_exponent<T>::type;

}  // namespace compiled
