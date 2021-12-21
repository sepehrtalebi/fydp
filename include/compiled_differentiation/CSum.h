#pragma once

namespace compiled {

template <typename L, typename R>
struct sum {
  using type = sum<L, R>;

  static inline double apply(const double* variables) {
    return L::apply(variables) + R::apply(variables);
  }
};

template <typename L, typename R>
using sum_t = typename sum<L, R>::type;

template<typename T>
struct is_sum : std::false_type {};

template<typename L, typename R>
struct is_sum<sum<L, R>> : std::true_type {};

template<typename T>
inline constexpr bool is_sum_v = is_sum<T>::value;

}  // namespace compiled
