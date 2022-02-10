#pragma once

#include <bmb_differentiation/compiled/CDifference.h>
#include <bmb_differentiation/compiled/CProduct.h>
#include <bmb_differentiation/compiled/CSum.h>
#include <type_traits>

namespace compiled {

/**
 * Returns the L in sum<L, R>, difference<L, R>, or product<L, R>.
 * If the given type is not an instance of any of these, then void is returned.
 */
template <typename T>
struct get_left_operand {
  using type = void;
};

template <typename T>
using get_left_operand_t = typename get_left_operand<T>::type;

template <typename L, typename R>
struct get_left_operand<sum<L, R>> {
  using type = L;
};

template <typename L, typename R>
struct get_left_operand<difference<L, R>> {
  using type = L;
};

template <typename L, typename R>
struct get_left_operand<product<L, R>> {
  using type = L;
};

/**
 * Returns the R in sum<L, R>, difference<L, R>, or product<L, R>.
 * If the given type is not an instance of any of these, then void is returned.
 */
template <typename T>
struct get_right_operand {
  using type = void;
};

template <typename T>
using get_right_operand_t = typename get_right_operand<T>::type;

template <typename L, typename R>
struct get_right_operand<sum<L, R>> {
  using type = R;
};

template <typename L, typename R>
struct get_right_operand<difference<L, R>> {
  using type = R;
};

template <typename L, typename R>
struct get_right_operand<product<L, R>> {
  using type = R;
};

}  // namespace compiled
