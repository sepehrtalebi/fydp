#pragma once

#include <bmb_differentiation/compiled/CCos.h>
#include <bmb_differentiation/compiled/CDerivative.h>
#include <bmb_differentiation/compiled/CHigherDerivative.h>
#include <bmb_differentiation/compiled/CDifference.h>
#include <bmb_differentiation/compiled/CExp.h>
#include <bmb_differentiation/compiled/CLog.h>
#include <bmb_differentiation/compiled/CPower.h>
#include <bmb_differentiation/compiled/CProduct.h>
#include <bmb_differentiation/compiled/CQuotient.h>
#include <bmb_differentiation/compiled/CSimplify.h>
#include <bmb_differentiation/compiled/CSin.h>
#include <bmb_differentiation/compiled/CSum.h>

#include <type_traits>

namespace compiled {

template <typename T, typename /** enable **/ = void>
struct deep_simplify : T {};

template <typename T>
using deep_simplify_t = typename deep_simplify<T>::type;

template <typename T>
struct deep_simplify<T, std::enable_if_t<!is_fully_simplified_v<T>>> :
    deep_simplify<simplify_t<T>> {};

// operator overloading

template <typename L, typename R>
inline constexpr deep_simplify_t<sum_t<L, R>> operator+(L, R) {
  return {};
}

template<typename R>
inline constexpr double operator+(Constant<R>, const double& d) {
  return Constant<R>::value + d;
}

template<typename R>
inline constexpr double operator+(const double& d, Constant<R>) {
  return d + Constant<R>::value;
}

template <typename L, typename R>
inline constexpr deep_simplify_t<difference_t<L, R>> operator-(L, R) {
  return {};
}

template<typename R>
inline constexpr double operator-(Constant<R>, const double& d) {
  return Constant<R>::value - d;
}

template<typename R>
inline constexpr double operator-(const double& d, Constant<R>) {
  return d - Constant<R>::value;
}

template <typename L, typename R>
inline constexpr deep_simplify_t<product_t<L, R>> operator*(L, R) {
  return {};
}

template<typename R>
inline constexpr double operator*(Constant<R>, const double& d) {
  return Constant<R>::value * d;
}

template<typename R>
inline constexpr double operator*(const double& d, Constant<R>) {
  return d * Constant<R>::value;
}

template <typename N, typename D>
inline constexpr deep_simplify_t<quotient_t<N, D>> operator/(N, D) {
  return {};
}

template<typename R>
inline constexpr double operator/(Constant<R>, const double& d) {
  return Constant<R>::value / d;
}

template<typename R>
inline constexpr double operator/(const double& d, Constant<R>) {
  return d / Constant<R>::value;
}

template<typename T>
inline constexpr deep_simplify_t<T> operator+(T) {
  return {};
}

template<typename T>
inline constexpr deep_simplify_t<product_t<MinusOne, T>> operator-(T) {
  return {};
}

template <typename T>
inline constexpr deep_simplify_t<sine_t<T>> sin(T) {
  return {};
}

template <typename T>
inline constexpr deep_simplify_t<cosine_t<T>> cos(T) {
  return {};
}

template <typename T>
inline constexpr deep_simplify_t<exponential_t<T>> exp(T) {
  return {};
}

template <typename T>
inline constexpr deep_simplify_t<logarithm_t<T>> log(T) {
  return {};
}

template <typename B, typename E>
inline constexpr deep_simplify_t<power_t<B, E>> pow(B, E) {
  return {};
}

template<typename R>
inline constexpr double pow(Constant<R>, const double& d) {
  return std::pow(Constant<R>::value, d);
}

template<typename R>
inline constexpr double pow(const double& d, Constant<R>) {
  return std::pow(d, Constant<R>::value);
}

template <size_t id, typename T>
inline constexpr deep_simplify_t<derivative_t<id, T>> getDerivative(
    Variable<id> /** var **/, T /** expr **/) {
  return {};
}

/**
 * Note that order needs to be the first template parameter so that this can be
 * called via getHigherDerivative<2>(x, x + x)
 *
 * @tparam order The number of derivatives to take
 */
template <size_t order, size_t id, typename T>
inline constexpr deep_simplify_t<higher_derivative_t<id, order, T>>
getHigherDerivative(Variable<id> /** var **/, T /** expr **/) {
  return {};
}

}  // namespace compiled
