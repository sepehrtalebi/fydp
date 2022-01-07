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

namespace internal {

// used for recursive calls for convenience
#define REC(T) decltype(simplify(std::declval<T>()))

template <typename T>
constexpr auto simplify(T /** value **/) {
  // Sum
  if constexpr (is_sum_v<T>) {
    using left = typename T::left_operand_type;
    using right = typename T::right_operand_type;

    if constexpr (is_constant_v<left> && is_constant_v<right>)
      return Constant<
          std::ratio_add<typename left::ratio, typename right::ratio>>{};
    // 0 + T -> T
    else if constexpr (std::is_same_v<left, Zero>)
      return simplify(right{});
    // T + 0 -> T
    else if constexpr (std::is_same_v<right, Zero>)
      return simplify(left{});
    // T + T -> 2 * T
    else if constexpr (std::is_same_v<left, right>)
      return simplify(product_t<Constant<std::ratio<2>>, REC(left)>{});
    // (T1 + T2) + T3, where T1 + T2 and T3 are both fully simplified
    else if constexpr (is_sum_v<left> && std::is_same_v<REC(left), left> &&
                       std::is_same_v<REC(right), right>) {
      using T1 = typename left::left_operand_type;
      using T2 = typename left::right_operand_type;
      using sum_13 = sum_t<T1, right>;
      using s_sum_13 = REC(sum_13);
      using sum_23 = sum_t<T2, right>;
      using s_sum_23 = REC(sum_23);
      if constexpr (!std::is_same_v<sum_13, s_sum_13>)
        return simplify(sum_t<s_sum_13, T2>{});
      else if constexpr (!std::is_same_v<sum_23, s_sum_23>)
        return simplify(sum_t<s_sum_23, T1>{});
      else
        return T{};
    }
    // default case
    else
      return sum_t<REC(left), REC(right)>{};
    // C1 + C2 -> C3
  }

  // Difference
  else if constexpr (is_difference_v<T>) {
    using left = typename T::left_operand_type;
    using right = typename T::right_operand_type;

    using minus_right = product_t<MinusOne, REC(right)>;
    return simplify(sum_t<REC(left), REC(minus_right)>{});
  }

  // Product
  else if constexpr (is_product_v<T>) {
    using left = typename T::left_operand_type;
    using right = typename T::right_operand_type;

    // C1 * C2 -> C3
    if constexpr (is_constant_v<left> && is_constant_v<right>)
      return Constant<
          std::ratio_multiply<typename left::ratio, typename right::ratio>>{};
    // 0 * R -> 0, L * 0 -> 0
    else if constexpr (std::is_same_v<left, Zero> ||
                       std::is_same_v<right, Zero>)
      return Zero{};
    // 1 * R -> R
    else if constexpr (std::is_same_v<left, One>)
      return simplify(right{});
    // L * 1 -> L
    else if constexpr (std::is_same_v<right, One>)
      return simplify(left{});
    // L * L -> L ^ 2
    else if constexpr (std::is_same_v<left, right>)
      return simplify(power_t<REC(left), Constant<std::ratio<2>>>{});
    // B ^ E1 * B ^ E2 -> B ^ (E1 + E2)
    else if constexpr (is_power_v<left> && is_power_v<right>) {
      if constexpr (std::is_same_v<typename left::base, typename right::base>) {
        using s_e =
            sum_t<REC(typename left::exponent), REC(typename right::exponent)>;
        return simplify(power_t<REC(typename left::base), REC(s_e)>{});
      } else
        return product_t<REC(left), REC(right)>{};
    }
    // B1 ^ E * B2 ^ E -> (B1 * B2) ^ E
    else if constexpr (is_power_v<left> && is_power_v<right>) {
      if constexpr (std::is_same_v<typename left::exponent,
                                   typename right::exponent>) {
        using s_b =
            product_t<REC(typename left::base), REC(typename right::base)>;
        return simplify(power_t<REC(s_b), REC(typename left::exponent)>{});
      } else
        return product_t<REC(left), REC(right)>{};
    }
    // B ^ E * B -> B ^ (E + 1)
    else if constexpr (is_power_v<left>) {
      if constexpr (std::is_same_v<typename left::base, right>) {
        using s_e = sum_t<REC(typename left::exponent), One>;
        return simplify(power_t<REC(right), REC(s_e)>{});
      } else
        return product_t<REC(left), REC(right)>{};
    }
    // B * B ^ E -> B ^ (E + 1)
    else if constexpr (is_power_v<right>) {
      if constexpr (std::is_same_v<typename right::base, left>) {
        using s_e = sum_t<REC(typename right::exponent), One>;
        return simplify(power_t<REC(left), REC(s_e)>{});
      } else
        return product_t<REC(left), REC(right)>{};
    }
    // (T1 * T2) * T3, where T1 * T2 and T3 are both fully simplified
    else if constexpr (is_product_v<left> && std::is_same_v<REC(left), left> &&
                       std::is_same_v<REC(right), right>) {
      using T1 = typename left::left_operand_type;
      using T2 = typename left::right_operand_type;
      using product_13 = product_t<T1, right>;
      using s_product_13 = REC(product_13 );
      using product_23 = product_t<T2, right>;
      using s_product_23 = REC(product_23);
      if constexpr (!std::is_same_v<product_13, s_product_13>)
        return simplify(product_t<s_product_13, T2>{});
      else if constexpr (!std::is_same_v<product_23, s_product_23>)
        return simplify(product_t<s_product_23, T1>{});
      else
        return T{};
    }
    // T1 * (T2 * T3), where T1 and T2 * T3 are both fully simplified
    else if constexpr (is_product_v<right> && std::is_same_v<REC(left), left> &&
                       std::is_same_v<REC(right), right>) {
      using T2 = typename right::left_operand_type;
      using T3 = typename right::right_operand_type;
      using product_12 = product_t<left, T2>;
      using s_product_12 = REC(product_12);
      using product_13 = product_t<left, T3>;
      using s_product_13 = REC(product_13);
      if constexpr (!std::is_same_v<product_12, s_product_12>)
        return simplify(product_t<s_product_12, T3>{});
      else if constexpr (!std::is_same_v<product_13, s_product_13>)
        return simplify(product_t<s_product_13, T2>{});
      else
        return T{};
    }
    // default case
    else
      return product_t<REC(left), REC(right)>{};
  }

  // Quotient
  else if constexpr (is_quotient_v<T>) {
    using num = typename T::num_type;
    using den = typename T::den_type;

    using den_inv = power_t<REC(den), MinusOne>;
    return simplify(product_t<REC(num), REC(den_inv)>{});
  }

  // Sine
  else if constexpr (is_sine_v<T>) {
    using operand = typename T::operand;

    // sin(0) -> 0
    if constexpr (std::is_same_v<operand, Zero>) return Zero{};
    // default case
    else
      return sine_t<REC(operand)>{};
  }

  // Cosine
  else if constexpr (is_cosine_v<T>) {
    using operand = typename T::operand;

    // cos(0) -> 1
    if constexpr (std::is_same_v<operand, Zero>) return One{};
    // default case
    else
      return cosine_t<REC(operand)>{};
  }

  // Exponential
  else if constexpr (is_exponential_v<T>) {
    using operand = typename T::operand;

    // exp(0) -> 1
    if constexpr (std::is_same_v<operand, Zero>) return One{};
    // default case
    else
      return exponential_t<REC(operand)>{};
  }

  // Logarithm
  else if constexpr (is_logarithm_v<T>) {
    using operand = typename T::operand;

    // log(1) -> 0
    if constexpr (std::is_same_v<operand, One>) return Zero{};
    // default case
    else
      return logarithm_t<REC(operand)>{};
  }

  // Power
  else if constexpr (is_power_v<T>) {
    using base = typename T::base;
    using exponent = typename T::exponent;

    // 0 ^ 0 -> 1
    if constexpr (std::is_same_v<base, Zero> && std::is_same_v<exponent, Zero>)
      return One{};
    // 0 ^ 1 -> 0
    else if constexpr (std::is_same_v<base, Zero> && std::is_same_v<exponent, One>)
      return Zero{};
    // 1 ^ 0 -> 1
    else if constexpr (std::is_same_v<base, One> && std::is_same_v<exponent, Zero>)
      return One{};
    // 1 ^ 1 -> 1
    else if constexpr (std::is_same_v<base, One> && std::is_same_v<exponent, One>)
      return One{};
    // B ^ 0 -> 1
    else if constexpr (std::is_same_v<exponent, Zero>) return One{};
    // B ^ 1 -> B
    else if constexpr (std::is_same_v<exponent, One>) return simplify(base{});
    // 0 ^ E -> 0
    else if constexpr (std::is_same_v<base, Zero>) return Zero{};
    // 1 ^ E -> 1
    else if constexpr (std::is_same_v<base, One>) return One{};
    // default case
    else
      return power_t<REC(base), REC(exponent)>{};
  }

  // default case
  else
    return T{};
}

}  // namespace internal

template <typename T>
struct simplify {
  using type = decltype(internal::simplify(std::declval<T>()));
};

template <typename T>
using simplify_t = typename simplify<T>::type;

template <typename T>
struct is_fully_simplified {
  static constexpr bool value = std::is_same_v<T, simplify_t<T>>;
};

template <typename T>
inline constexpr bool is_fully_simplified_v = is_fully_simplified<T>::value;

// reordering operations

}  // namespace compiled
