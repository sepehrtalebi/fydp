#pragma once

#include <cstddef>
#include <tuple>
#include <type_traits>

template<typename ...Ts>
struct TypeArray;

template<typename T, typename ...Ts>
struct TypeArray<T, Ts...> {
  template <size_t i>
  using get_type = std::conditional<i == 0, T, typename TypeArray<Ts...>::template get_type<i - 1>::type>;

  template <size_t i>
  using get_type_t = typename get_type<i>::type;

  template <size_t i>
  auto get() {
    if constexpr (i == 0) return T{};
    else return TypeArray<Ts...>{}.template get<i - 1>();
  }
};
