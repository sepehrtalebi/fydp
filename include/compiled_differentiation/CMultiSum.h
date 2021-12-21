#pragma once

namespace compiled {

template <typename ...Ts>
struct multi_sum;

template <typename ...Ts>
using multi_sum_t = typename multi_sum<Ts...>::type;

template<typename T, typename ...Ts>
struct multi_sum<T, Ts...> {
  using type = multi_sum<T, Ts...>;

  static inline double apply(const double* variables) {
    return T::apply(variables) + multi_sum_t<Ts...>::apply(variables);
  }
};

template<typename T>
struct multi_sum<T> : T {};

template<>
struct multi_sum<> : Zero {};

template<typename T>
struct is_multi_sum : std::false_type {};

template<typename ...Ts>
struct is_multi_sum<multi_sum<Ts...>> : std::true_type {};

template<typename ...Ts>
inline constexpr bool is_multi_sum_v = is_multi_sum<Ts...>::value;

}  // namespace compiled
