#pragma once

#include <array>
#include <type_traits>

namespace compiled {

template <typename L, typename R>
struct sum {
  using type = sum<L, R>;
  using left_operand_type = L;
  using right_operand_type = R;

  template <size_t n>
  static inline double apply(const std::array<double, n>& variables) {
    return L::apply(variables) + R::apply(variables);
  }
};

template <typename L, typename R>
using sum_t = typename sum<L, R>::type;

/**
 * Returns true if T is an instance of sum and false otherwise.
 */
template <typename T>
struct is_sum : std::false_type {};

template <typename L, typename R>
struct is_sum<sum<L, R>> : std::true_type {};

template <typename T>
inline constexpr bool is_sum_v = is_sum<T>::value;

}  // namespace compiled
