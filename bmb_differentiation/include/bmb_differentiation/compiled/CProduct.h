#pragma once

#include <array>
#include <type_traits>

namespace compiled {

template <typename L, typename R>
struct product {
  using type = product<L, R>;
  using left_operand_type = L;
  using right_operand_type = R;

  template <size_t n>
  static inline double apply(const std::array<double, n>& variables) {
    return L::apply(variables) * R::apply(variables);
  }
};

template <typename L, typename R>
using product_t = typename product<L, R>::type;

/**
 * Returns true if T is an instance of product and false otherwise.
 */
template <typename T>
struct is_product : std::false_type {};

template <typename L, typename R>
struct is_product<product<L, R>> : std::true_type {};

template <typename T>
inline constexpr bool is_product_v = is_product<T>::value;

}  // namespace compiled
