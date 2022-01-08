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
    auto two = Constant<std::ratio<2>>{};
    auto three = Constant<std::ratio<3>>{};
    auto four = Constant<std::ratio<4>>{};
    auto six = Constant<std::ratio<6>>{};
    auto seven = Constant<std::ratio<7>>{};

    auto t = getHigherDerivative<1>(x, three * x + x);

    ASSERT_EQUAL(four, getDerivative(x, four * x + y));
}

void testCompiledDifferentiation() {
    using namespace compiled;

    auto x = Variable<0>{};
    auto y = Variable<1>{};
    auto two = Constant<std::ratio<2>>{};
    auto three = Constant<std::ratio<3>>{};
    auto four = Constant<std::ratio<4>>{};
    auto five = Constant<std::ratio<45>>{};
    auto six = Constant<std::ratio<6>>{};
    auto seven = Constant<std::ratio<7>>{};

    static_assert(is_constant_v<decltype(three)>);
    static_assert(is_constant_v<Zero>);
    static_assert(is_constant_v<One>);
    static_assert(!is_constant_v<decltype(x)>);
    static_assert(is_constant_v<decltype(three + four)>);

    ASSERT_EQUAL(zero, zero);
    ASSERT_EQUAL(seven, three + four);
    ASSERT_EQUAL(three, three + zero);
    static_assert(getNodeCount(three + zero) == 1);
    static_assert(getNodeCount(three + two) == 1);
    static_assert(getNodeCount(x + y) == 3);
    ASSERT_EQUAL(one, getDerivative(x, x + y));
    ASSERT_EQUAL(x, getHigherDerivative<0>(x, x));
    ASSERT_EQUAL(one, getDerivative(x, x));
    ASSERT_EQUAL(one, getHigherDerivative<1>(x, x));
    static_assert(getNodeCount(getDerivative(x, x + y)) == 1);
    ASSERT_EQUAL(x * two, getDerivative(x, pow(x, two)));
    ASSERT_EQUAL(two * x - four, x - four + x);

//    static_assert(std::is_same_v<product_t<two, x>, deep_simplify_t<derivative_t<0, power_t<x, two>>>>);
//    static_assert(std::is_same_v<product_t<six, x>, higher_derivative_t<0, 2, power_t<x, three>>>);
}
