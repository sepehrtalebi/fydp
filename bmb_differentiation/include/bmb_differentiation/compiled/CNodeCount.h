#pragma once

#include <bmb_differentiation/compiled/CConstant.h>
#include <bmb_differentiation/compiled/CCos.h>
#include <bmb_differentiation/compiled/CDifference.h>
#include <bmb_differentiation/compiled/CExp.h>
#include <bmb_differentiation/compiled/CLog.h>
#include <bmb_differentiation/compiled/CMultiSum.h>
#include <bmb_differentiation/compiled/CPower.h>
#include <bmb_differentiation/compiled/CProduct.h>
#include <bmb_differentiation/compiled/CQuotient.h>
#include <bmb_differentiation/compiled/CSin.h>
#include <bmb_differentiation/compiled/CSum.h>
#include <bmb_differentiation/compiled/CVariable.h>

namespace compiled {

template <typename T>
struct node_count {};

template <typename T>
constexpr inline unsigned int node_count_v = node_count<T>::value;

template <typename T>
inline constexpr unsigned int getNodeCount(T) {
  return node_count_v<T>;
}

/**
 * Computes the type with the smallest node_count of all input types.
 * In the case of a tie, the first such candidate is selected
 */
template<typename ...Ts>
struct get_smallest_node_count;

template<typename ...Ts>
using get_smallest_node_count_t = typename get_smallest_node_count<Ts...>::type;

template<typename T>
struct get_smallest_node_count<T> {
    using type = T;
};

template<typename T, typename ...Ts>
struct get_smallest_node_count<T, Ts...> {
private:
    using Ts_smallest_type = get_smallest_node_count_t<Ts...>;
    static constexpr unsigned int Ts_node_count = node_count_v<Ts_smallest_type>;
    static constexpr unsigned int T_node_count = node_count_v<T>;
public:
    using type = std::conditional_t<T_node_count <= Ts_node_count, T, Ts_smallest_type>;
};

// node_count implementations for different types

template <typename R>
struct node_count<Constant<R>> {
  static constexpr unsigned int value = 1;
};

template <size_t id>
struct node_count<Variable<id>> {
  static constexpr unsigned int value = 1;
};

template <typename L, typename R>
struct node_count<sum<L, R>> {
  static constexpr unsigned int value = node_count_v<L> + node_count_v<R> + 1;
};

template<typename T, typename ...Ts>
struct node_count<multi_sum<T, Ts...>> {
  static constexpr unsigned int value = node_count_v<T> + node_count_v<multi_sum_t<Ts...>> + 1;
};

template<typename T>
struct node_count<multi_sum<T>> : node_count<T> {};

template<>
struct node_count<multi_sum<>> {
  static constexpr unsigned int value = 1;
};

template <typename L, typename R>
struct node_count<difference<L, R>> {
  static constexpr unsigned int value = node_count_v<L> + node_count_v<R> + 1;
};

template <typename L, typename R>
struct node_count<product<L, R>> {
  static constexpr unsigned int value = node_count_v<L> + node_count_v<R> + 1;
};

template <typename N, typename D>
struct node_count<quotient<N, D>> {
  static constexpr unsigned int value = node_count_v<N> + node_count_v<D> + 1;
};

template <typename T>
struct node_count<sine<T>> {
  static constexpr unsigned int value = node_count_v<T> + 1;
};

template <typename T>
struct node_count<cosine<T>> {
  static constexpr unsigned int value = node_count_v<T> + 1;
};

template <typename T>
struct node_count<exponential<T>> {
  static constexpr unsigned int value = node_count_v<T> + 1;
};

template <typename T>
struct node_count<logarithm<T>> {
  static constexpr unsigned int value = node_count_v<T> + 1;
};

template <typename B, typename E>
struct node_count<power<B, E>> {
  static constexpr unsigned int value = node_count_v<B> + node_count_v<E> + 1;
};

}  // namespace compiled
