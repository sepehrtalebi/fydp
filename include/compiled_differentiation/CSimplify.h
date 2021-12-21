#pragma once

#include "CConstant.h"
#include "CCos.h"
#include "CDifference.h"
#include "CExp.h"
#include "CLog.h"
#include "CNodeCount.h"
#include "CPower.h"
#include "CProduct.h"
#include "CQuotient.h"
#include "CSin.h"
#include "CSum.h"
#include "CVariable.h"

#include <type_traits>

namespace compiled {

template <typename T, typename /** enable **/ = void>
struct simplify {
  using type = T;
};

template <typename T>
using simplify_t = typename simplify<T>::type;

template <typename T>
inline constexpr simplify_t<T> getSimplified(T) {
  return simplify_t<T>{};
}

template <typename T>
struct is_fully_simplified {
  static constexpr bool value = std::is_same_v<T, simplify_t<T>>;
};

template <typename T>
inline constexpr bool is_fully_simplified_v = is_fully_simplified<T>::value;

// base simplifications:

// C1 + C2 -> C3
template <typename R1, typename R2>
struct simplify<sum<Constant<R1>, Constant<R2>>> {
  using type = Constant<std::ratio_add<R1, R2>>;
};

// 0 + T -> T
template <typename T>
struct simplify<sum<Zero, T>, std::enable_if_t<!is_constant_v<T>>> {
  using type = simplify_t<T>;
};

// T + 0 -> T
template <typename T>
struct simplify<sum<T, Zero>, std::enable_if_t<!is_constant_v<T>>> {
  using type = simplify_t<T>;
};

// T + T -> 2 * T
template <typename T>
struct simplify<sum<T, T>, std::enable_if_t<!is_constant_v<T>>> {
  using type = product_t<Constant<std::ratio<2>>, simplify_t<T>>;
};

template <typename L, typename R>
struct simplify<
    sum<L, R>, std::enable_if_t<
                   !(is_constant_v<L> && is_constant_v<R>) &&
                   !std::is_same_v<L, Zero> &&
                   !std::is_same_v<R, Zero> &&
                   !std::is_same_v<L, R>>> {
  using type = sum_t<simplify_t<L>, simplify_t<R>>;
};

// C1 - C2 -> C3
template <typename R1, typename R2>
struct simplify<difference<Constant<R1>, Constant<R2>>> {
  using type = Constant<std::ratio_subtract<R1, R2>>;
};

// 0 - T -> -1 * T
template <typename T>
struct simplify<difference<Zero, T>, std::enable_if_t<!is_constant_v<T>>> {
  using type = simplify_t<product_t<MinusOne, simplify_t<T>>>;
};

// T - 0 -> T
template <typename T>
struct simplify<difference<T, Zero>, std::enable_if_t<!is_constant_v<T>>> {
  using type = simplify_t<T>;
};

// T - T -> 0
template <typename T>
struct simplify<difference<T, T>, std::enable_if_t<!is_constant_v<T>>> {
  using type = Zero;
};

template <typename L, typename R>
struct simplify<difference<L, R>, std::enable_if_t<
                                      !(is_constant_v<L> && is_constant_v<R>) &&
                                      !std::is_same_v<L, Zero> &&
                                      !std::is_same_v<R, Zero> &&
                                      !std::is_same_v<L, R>>> {
  using type = difference_t<simplify_t<L>, simplify_t<R>>;
};

// C1 * C2 -> C3
template <typename R1, typename R2>
struct simplify<product<Constant<R1>, Constant<R2>>> {
  using type = Constant<std::ratio_multiply<R1, R2>>;
};

// 0 * R -> 0
template <typename R>
struct simplify<product<Zero, R>, std::enable_if_t<!is_constant_v<R>>> {
  using type = Zero;
};

// L * 0 -> 0
template <typename L>
struct simplify<product<L, Zero>, std::enable_if_t<!is_constant_v<L>>> {
  using type = Zero;
};

// 1 * R -> R
template <typename R>
struct simplify<product<One, R>, std::enable_if_t<!is_constant_v<R>>> {
  using type = simplify_t<R>;
};

// L * 1 -> L
template <typename L>
struct simplify<product<L, One>, std::enable_if_t<!is_constant_v<L>>> {
  using type = simplify_t<L>;
};

// L * L -> L ^ 2
template <typename T>
struct simplify<product<T, T>, std::enable_if_t<!is_constant_v<T>>> {
  using type = power_t<simplify_t<T>, Constant<std::ratio<2>>>;
};

template <typename L, typename R>
struct simplify<product<L, R>, std::enable_if_t<
                                   !(is_constant_v<L> && is_constant_v<R>) &&
                                   !is_zero_or_one_v<L> &&
                                   !is_zero_or_one_v<R> &&
                                   !std::is_same_v<L, R>>> {
  using type = product_t<simplify_t<L>, simplify_t<R>>;
};

// C1 / C2 -> C3
template <typename R1, typename R2>
struct simplify<quotient<Constant<R1>, Constant<R2>>> {
  using type = Constant<std::ratio_divide<R1, R2>>;
};

// T / 1 -> T
template <typename T>
struct simplify<quotient<T, One>, std::enable_if_t<!is_constant_v<T>>> {
  using type = simplify_t<T>;
};

// 0 / T -> 0
template <typename T>
struct simplify<quotient<Zero, T>, std::enable_if_t<!is_constant_v<T>>> {
  using type = Zero;
};

// T / T -> 1
template <typename T>
struct simplify<quotient<T, T>, std::enable_if_t<!is_constant_v<T>>> {
  using type = One;
};

template <typename N, typename D>
struct simplify<quotient<N, D>, std::enable_if_t<
                                    !(is_constant_v<N> && is_constant_v<D>) &&
                                    !std::is_same_v<D, One> &&
                                    !std::is_same_v<N, Zero> &&
                                    !std::is_same_v<N, D>>> {
  using type = quotient_t<simplify_t<N>, simplify_t<D>>;
};

// sin(0) -> 0
template <>
struct simplify<sine<Zero>> {
  using type = Zero;
};

template <typename T>
struct simplify<sine<T>, std::enable_if_t<!std::is_same_v<T, Zero>>> {
  using type = sine_t<simplify_t<T>>;
};

// cos(0) -> 1
template <>
struct simplify<cosine<Zero>> {
  using type = One;
};

template <typename T>
struct simplify<cosine<T>, std::enable_if_t<std::is_same_v<T, Zero>>> {
  using type = cosine_t<simplify_t<T>>;
};

// exp(0) -> 1
template <>
struct simplify<exponential<Zero>> {
  using type = One;
};

template <typename T>
struct simplify<exponential<T>, std::enable_if_t<std::is_same_v<T, Zero>>> {
  using type = exponential_t<simplify_t<T>>;
};

// log(1) -> 0
template <>
struct simplify<logarithm<One>> {
  using type = Zero;
};

template <typename T>
struct simplify<logarithm<T>, std::enable_if_t<std::is_same_v<T, One>>> {
  using type = logarithm_t<simplify_t<T>>;
};

// 0 ^ 0 -> 1
template <>
struct simplify<power<Zero, Zero>> {
  using type = One;
};

// 0 ^ 1 -> 0
template <>
struct simplify<power<Zero, One>> {
  using type = Zero;
};

// 1 ^ 0 -> 1
template <>
struct simplify<power<One, Zero>> {
  using type = One;
};

// 1 ^ 1 -> 1
template <>
struct simplify<power<One, One>> {
  using type = One;
};

// B ^ 0 -> 1
template <typename B>
struct simplify<power<B, Zero>, std::enable_if_t<!is_zero_or_one_v<B>>> {
  using type = One;
};

// B ^ 1 -> B
template <typename B>
struct simplify<power<B, One>, std::enable_if_t<!is_zero_or_one_v<B>>> {
  using type = simplify_t<B>;
};

// 0 ^ E -> 0
template <typename E>
struct simplify<power<Zero, E>, std::enable_if_t<!is_zero_or_one_v<E>>> {
  using type = Zero;
};

// 1 ^ E -> 1
template <typename E>
struct simplify<power<One, E>, std::enable_if_t<!is_zero_or_one_v<E>>> {
  using type = One;
};

template <typename B, typename E>
struct simplify<power<B, E>, std::enable_if_t<
                                 !is_zero_or_one_v<B> &&
                                 !is_zero_or_one_v<E>>> {
  using type = power_t<simplify_t<B>, simplify_t<E>>;
};

// compound simplifications:

// B ^ E1 * B ^ E2 -> B ^ (E1 + E2)
template <typename B, typename E1, typename E2>
struct simplify<product<power<B, E1>, power<B, E2>>> {
  using type = power_t<simplify_t<B>, sum_t<simplify_t<E1>, simplify_t<E2>>>;
};

// B1 ^ E * B2 ^ E -> (B1 * B2) ^ E
template <typename B1, typename B2, typename E>
struct simplify<product<power<B1, E>, power<B2, E>>> {
  using type =
      power_t<product_t<simplify_t<B1>, simplify_t<B2>>, simplify_t<E>>;
};

// B ^ E * B -> B ^ (E + 1)
template <typename B, typename E>
struct simplify<product<power<B, E>, B>> {
  using type = power_t<B, simplify_t<sum_t<simplify_t<E>, One>>>;
};

// B * B ^ E -> B ^ (E + 1)
template <typename B, typename E>
struct simplify<product<B, power<B, E>>> {
  using type = power_t<B, simplify_t<sum_t<simplify_t<E>, One>>>;
};

// B ^ E / B -> B ^ (E - 1)
template <typename B, typename E>
struct simplify<quotient<power<B, E>, B>> {
  using type =
      power_t<simplify_t<B>, simplify_t<difference_t<simplify_t<E>, One>>>;
};

// B / B ^ E = 1 / B ^ (E - 1)
template <typename B, typename E>
struct simplify<quotient<B, power<B, E>>> {
  using type =
      quotient_t<One, power_t<simplify_t<B>,
                              simplify_t<difference_t<simplify_t<E>, One>>>>;
};

// L + (-R) -> L - R
template <typename L, typename R>
struct simplify<sum<L, product<MinusOne, R>>> {
  using type = difference_t<simplify_t<L>, simplify_t<R>>;
};

// (-L) + R -> R - L
template <typename L, typename R>
struct simplify<sum<product<MinusOne, L>, R>> {
  using type = difference_t<simplify_t<R>, simplify_t<L>>;
};

}  // namespace compiled
