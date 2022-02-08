#pragma once

#include <bmb_differentiation/compiled/CConstant.h>
#include <bmb_differentiation/compiled/CVariable.h>

#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>
#include <stdexcept>

namespace compiled {

/**
 * Represents an index into a TypeArray
 */
template<size_t i>
struct TypeArrayIndex {
  using type = TypeArrayIndex<i>;
  static constexpr size_t value = i;
};

/**
 * Compile time array of types.
 */
template<typename ...Ts>
struct TypeArray;

template<typename T, typename ...Ts>
struct TypeArray<T, Ts...> {
  using type = TypeArray<T, Ts...>;

  /**
   * Returns the ith element of this array.
   */
  template <size_t i>
  constexpr auto operator[](TypeArrayIndex<i>) {
    if constexpr (i == 0) return T{};
    else return TypeArray<Ts...>{}[TypeArrayIndex<i - 1>{}];
  }

  /**
   * Returns a copy of this array with the given type added at index 0.
   */
  template <typename V>
  constexpr TypeArray<V, T, Ts...> push_head(V) {
    return {};
  }

  /**
   * Returns a copy of this array with the given type added at the end.
   */
  template <typename V>
  constexpr TypeArray<T, Ts..., V> push_tail(V) {
    return {};
  }

  /**
   * Returns a copy of this array with the given type added at the specified index.
   */
  template <size_t i, typename V>
  constexpr auto set(TypeArrayIndex<i>, V) {
    if constexpr (i == 0) return TypeArray<V, Ts...>{};
    else {
      using rec = decltype(TypeArray<Ts...>{}.set(TypeArrayIndex<i - 1>{}, std::declval<V>()));
      return decltype(rec{}.push_head(std::declval<T>())){};
    }
  }
};

template<>
struct TypeArray<> {
  using type = TypeArray<>;

  template <size_t i>
  constexpr auto operator[](TypeArrayIndex<i>) {
    throw std::logic_error("Array index out of bounds!");
  }

  template <typename V>
  constexpr TypeArray<V> push_head(V) {
    return {};
  }

  template <typename V>
  constexpr TypeArray<V> push_tail(V) {
    return {};
  }

  template <size_t i, typename V>
  constexpr auto set(TypeArrayIndex<i>, V) {
    throw std::logic_error("Array index out of bounds!");
  }
};

/**
 * Constructs a TypeArray consisting of n Zeros
 */
template<size_t n>
struct get_zeros_type_array {
  using type = decltype(get_zeros_type_array<n - 1>{}.push_head(Zero{}));
};

template<>
struct get_zeros_type_array<0> : TypeArray<> {};

template<size_t n>
using get_zeros_type_array_t = typename get_zeros_type_array<n>::type;

/**
 * Constructs a TypeArray of size n consisting of
 * Variable<0>, Variable<1>, ... Variable<n - 1>.
 */
template<size_t n>
struct get_variables_type_array;

template<size_t n>
using get_variables_type_array_t = typename get_variables_type_array<n>::type;

template<size_t n>
struct get_variables_type_array {
  using type = decltype(
      get_variables_type_array_t<n - 1>{}.push_tail(Variable<n - 1>{}));
};

template<>
struct get_variables_type_array<0> : TypeArray<> {};

template<char... cs>
constexpr TypeArrayIndex<parse_size_t_v<cs...>> operator "" _i() {
  return {};
}

}
