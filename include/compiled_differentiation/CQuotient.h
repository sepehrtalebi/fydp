#pragma once

#include <type_traits>

namespace compiled {

template <typename N, typename D>
struct quotient {
  using type = quotient<N, D>;
  using num_type = N;
  using den_type = D;

  static inline double apply(const double* variables) {
    return N::apply(variables) / D::apply(variables);
  }
};

template <typename N, typename D>
using quotient_t = typename quotient<N, D>::type;

template<typename T>
struct is_quotient : std::false_type {};

template<typename L, typename R>
struct is_quotient<quotient<L, R>> : std::true_type {};

template<typename T>
inline constexpr bool is_quotient_v = is_quotient<T>::value;

}  // namespace compiled
