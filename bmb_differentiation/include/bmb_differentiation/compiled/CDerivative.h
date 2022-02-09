#pragma once

#include <bmb_differentiation/compiled/CConstant.h>
#include <bmb_differentiation/compiled/CCos.h>
#include <bmb_differentiation/compiled/CDifference.h>
#include <bmb_differentiation/compiled/CExp.h>
#include <bmb_differentiation/compiled/CLog.h>
#include <bmb_differentiation/compiled/CPower.h>
#include <bmb_differentiation/compiled/CProduct.h>
#include <bmb_differentiation/compiled/CQuotient.h>
#include <bmb_differentiation/compiled/CSin.h>
#include <bmb_differentiation/compiled/CSum.h>
#include <bmb_differentiation/compiled/CMultiSum.h>
#include <bmb_differentiation/compiled/CVariable.h>

#include <cstddef>

namespace compiled {

template <size_t id, typename T>
struct derivative;

template <size_t id, typename T>
using derivative_t = typename derivative<id, T>::type;

// Constant

template <size_t id, typename R>
struct derivative<id, Constant<R>> {
  using type = Zero;
};

// Variable

template <size_t deriv_id, size_t var_id>
struct derivative<deriv_id, Variable<var_id>> {
  using type = Zero;
};

template <size_t id>
struct derivative<id, Variable<id>> {
  using type = One;
};

// sum

template <size_t id, typename L, typename R>
struct derivative<id, sum<L, R>> {
  using type = sum_t<derivative_t<id, L>, derivative_t<id, R>>;
};

template<size_t id, typename T, typename ...Ts>
struct derivative<id, multi_sum<T, Ts...>> {
  using type = multi_sum_t<derivative_t<id, T>, derivative_t<id, multi_sum_t<Ts...>>>;
};

template<size_t id, typename T>
struct derivative<id, multi_sum<T>> : derivative<id, T> {};

template<size_t id>
struct derivative<id, multi_sum<>> : Zero {};

// difference

template <size_t id, typename L, typename R>
struct derivative<id, difference<L, R>> {
  using type = difference_t<derivative_t<id, L>, derivative_t<id, R>>;
};

// product

template <size_t id, typename L, typename R>
struct derivative<id, product<L, R>> {
  using type = sum_t<product_t<R, derivative_t<id, L>>,
                     product_t<L, derivative_t<id, R>>>;
};

// quotient

template <size_t id, typename N, typename D>
struct derivative<id, quotient<N, D>> {
 private:
  using num = difference_t<product_t<D, derivative_t<id, N>>,
                           product_t<N, derivative_t<id, D>>>;
  using den = product_t<D, D>;

 public:
  using type = quotient_t<num, den>;
};

// sine

template <size_t id, typename T>
struct derivative<id, sine<T>> {
  using type = product_t<derivative_t<id, T>, cosine_t<T>>;
};

// cosine

template <size_t id, typename T>
struct derivative<id, cosine<T>> {
  using type = product_t<MinusOne, product_t<derivative_t<id, T>, sine_t<T>>>;
};

// exponential

template <size_t id, typename T>
struct derivative<id, exponential<T>> {
  using type = product_t<derivative_t<id, T>, exponential_t<T>>;
};

// logarithm

template <size_t id, typename T>
struct derivative<id, logarithm<T>> {
  using type = quotient_t<derivative_t<id, T>, logarithm_t<T>>;
};

// Power

template <size_t id, typename B, typename E>
struct derivative<id, power<B, E>> {
  using type =
      product_t<power_t<B, E>,
                sum_t<product_t<derivative_t<id, E>, logarithm_t<B>>,
                      product_t<derivative_t<id, B>, quotient_t<E, B>>>>;
};

}  // namespace compiled
