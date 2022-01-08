#include "TestCompiledDifferentiation.h"

#include "CConstant.h"
#include "CVariable.h"
#include "CDerivative.h"
#include "CHigherDerivative.h"
#include "CDeepSimplify.h"

#include <type_traits>

#define ASSERT_EQUAL(T1, T2) static_assert(std::is_same_v<decltype(T1), decltype(T2)>)

void testOperators() {
    using namespace compiled;

    auto x = Variable<0>{};
    auto y = Variable<1>{};

    auto t = getHigherDerivative<1>(x, 3_c * x + x);

    ASSERT_EQUAL(4_c, getDerivative(x, 4_c * x + y));
}

void testCompiledDifferentiation() {
    using namespace compiled;

    auto x = Variable<0>{};
    auto y = Variable<1>{};

    static_assert(is_constant_v<decltype(3_c)>);
    static_assert(is_constant_v<Zero>);
    static_assert(is_constant_v<One>);
    static_assert(!is_constant_v<decltype(x)>);
    static_assert(is_constant_v<decltype(3_c + 4_c)>);

    ASSERT_EQUAL(0_c, 0_c);
    ASSERT_EQUAL(7_c, 3_c + 4_c);
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
}
