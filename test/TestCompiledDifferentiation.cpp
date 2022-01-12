#include "TestCompiledDifferentiation.h"

#include "CConstant.h"
#include "CVariable.h"
#include "CDerivative.h"
#include "CHigherDerivative.h"
#include "CDeepSimplify.h"

#include <type_traits>

#define ASSERT_EQUAL(T1, T2) static_assert(std::is_same_v<decltype(T1), decltype(T2)>)

template<typename T>
auto func(T x) {
  using namespace compiled;

  auto y = sin(3_c * x + 4_c);
  return 3_c * y * y;
}

void testCompiledDifferentiation() {
    using namespace compiled;

    auto x = 0_v;
    auto y = 1_v;

    static_assert(is_constant_v<decltype(3_c)>);
    static_assert(is_constant_v<Zero>);
    static_assert(is_constant_v<One>);
    static_assert(!is_constant_v<decltype(x)>);
    static_assert(is_constant_v<decltype(3_c + 4_c)>);

    ASSERT_EQUAL(0_c, 0_c);
    ASSERT_EQUAL(7_c, 3_c + 4_c);
    ASSERT_EQUAL(4.7_c, 2.1_c + 2.6_c);
    ASSERT_EQUAL(3_c, 3_c + 0_c);
    static_assert(getNodeCount(3_c + 0_c) == 1);
    static_assert(getNodeCount(3_c + 2_c) == 1);
    static_assert(getNodeCount(x + y) == 3);
    ASSERT_EQUAL(one, getDerivative(x, x + y));
    ASSERT_EQUAL(x, getHigherDerivative<0>(x, x));
    ASSERT_EQUAL(one, getDerivative(x, x));
    ASSERT_EQUAL(one, getHigherDerivative<1>(x, x));
    static_assert(getNodeCount(getDerivative(x, x + y)) == 1);
    ASSERT_EQUAL(x * 2_c, getDerivative(x, pow(x, 2_c)));
    ASSERT_EQUAL(2_c * x - 4_c, x - 4_c + x);

//    static_assert(std::is_same_v<product_t<2_c, x>, deep_simplify_t<derivative_t<0, power_t<x, 2_c>>>>);
//    static_assert(std::is_same_v<product_t<6_c, x>, higher_derivative_t<0, 2, power_t<x, 3_c>>>);

    // test func
    auto result = getDerivative(0_v, func(0_v));
    double result2 = func(3.0);
}
