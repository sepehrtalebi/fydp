#pragma once

#include "CCos.h"
#include "CDerivative.h"
#include "CDifference.h"
#include "CExp.h"
#include "CLog.h"
#include "CPower.h"
#include "CProduct.h"
#include "CQuotient.h"
#include "CSimplify.h"
#include "CSin.h"
#include "CSum.h"

#include <type_traits>

namespace compiled {

template <typename T, typename /** enable **/ = void>
struct deep_simplify : T {};

template <typename T>
using deep_simplify_t = typename deep_simplify<T>::type;

template <typename T>
struct deep_simplify<T, std::enable_if_t<!is_fully_simplified_v<T>>> : deep_simplify<simplify_t<T>> {};

template <typename T>
inline constexpr deep_simplify_t<T> getDeepSimplified(T) {
  return deep_simplify_t<T>{};
}

// operator overloading

template <typename L, typename R>
inline constexpr deep_simplify_t<sum_t<L, R>> operator+(L, R) {
  return deep_simplify_t<sum_t<L, R>>{};
}

template <typename L, typename R>
inline constexpr deep_simplify_t<difference_t<L, R>> operator-(L, R) {
  return deep_simplify_t<difference_t<L, R>>{};
}

template <typename L, typename R>
inline constexpr deep_simplify_t<product_t<L, R>> operator*(L, R) {
  return deep_simplify_t<product_t<L, R>>{};
}

template <typename N, typename D>
inline constexpr deep_simplify_t<quotient_t<N, D>> operator/(N, D) {
  return deep_simplify_t<quotient_t<N, D>>{};
}

template<typename T>
inline constexpr deep_simplify_t<T> operator+(T) {
  return deep_simplify_t<T>{};
}

template<typename T>
inline constexpr deep_simplify_t<product_t<MinusOne, T>> operator-(T) {
  return deep_simplify_t<product_t<MinusOne, T>>{};
}

template <typename T>
inline constexpr deep_simplify_t<sine_t<T>> sin(T) {
  return deep_simplify_t<sine_t<T>>{};
}

template <typename T>
inline constexpr deep_simplify_t<cosine_t<T>> cos(T) {
  return deep_simplify_t<cosine_t<T>>{};
}

template <typename T>
inline constexpr deep_simplify_t<exponential_t<T>> exp(T) {
  return deep_simplify_t<exponential_t<T>>{};
}

template <typename T>
inline constexpr deep_simplify_t<logarithm_t<T>> log(T) {
  return deep_simplify_t<logarithm_t<T>>{};
}

template <typename B, typename E>
inline constexpr deep_simplify_t<power_t<B, E>> pow(B, E) {
  return deep_simplify_t<power_t<B, E>>{};
}

template <size_t id, typename T>
inline constexpr deep_simplify_t<derivative_t<id, T>> getDerivative(
    Variable<id>, T) {
  return deep_simplify_t<derivative_t<id, T>>{};
}

/**
 * Note that order needs to be the first template parameter so that this can be
 * called via getHigherDerivative<2>(x, x + x)
 *
 * @tparam order The number of derivatives to take
 */
template <size_t order, size_t id, typename T>
inline constexpr deep_simplify_t<higher_derivative_t<id, order, T>>
getHigherDerivative(Variable<id>, T) {
  return deep_simplify_t<higher_derivative_t<id, order, T>>{};
}

}  // namespace compiled
