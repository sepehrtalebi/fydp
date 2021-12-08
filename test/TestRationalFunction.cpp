#include "TestRationalFunction.h"
#include "RationalFunction.h"
#include <cassert>

void testRationalFunction() {
    RationalFunction<double, 1, 2> f{2, 3, 1};
    RationalFunction<double, 1, 3> g{4, 5, 0, 1};
    auto multiply = f * g;
    multiply.print();
    RationalFunction<double, 1, 4> truth{8, 15, 5, 3, 1};
    assert(truth.numerator_data(0) == multiply.numerator_data(0));
    for (int i = 0; i < 4; i++)
        assert(multiply.denominator_data(i) == truth.denominator_data(i));
    auto divide = f / g;
    divide.print();
    RationalFunction<double, 3, 2> truth1 = {10, 0, 2, 12, 4};
    for (int i = 0; i < 3; i++)
        assert(divide.numerator_data(i) == truth1.numerator_data(i));
    for (int i = 0; i < 2; i++)
        assert(divide.denominator_data(i) == truth1.denominator_data(i));
    auto add = f + g;
    add.print();
    RationalFunction<double, 3, 4> truth2 {22, 4, 2, 15, 5, 3, 1};
    for (int i = 0; i < 3; i++)
        assert(add.numerator_data(i) == truth2.numerator_data(i));
    for (int i = 0; i < 4; i++)
        assert(add.denominator_data(i) == truth2.denominator_data(i));
    auto subtract = f - g;
    subtract.print();
    RationalFunction<double, 3, 4> truth3 = {-2, -4, 2, 15, 5, 3, 1};
    for (int i = 0; i < 3; i++)
        assert(subtract.numerator_data(i) == truth3.numerator_data(i));
    for (int i = 0; i < 4; i++)
        assert(subtract.denominator_data(i) == truth3.denominator_data(i));
    auto g_of_f = g._of_(f);
    g_of_f.print();
    RationalFunction<double, 3, 3> truth4 = {36, 24, 4, 49, 30, 5};
    for (int i = 0; i < 3; i++)
        assert(g_of_f.numerator_data(i) == truth4.numerator_data(i));
    for (int i = 0; i < 3; i++)
        assert(g_of_f.denominator_data(i) == truth4.denominator_data(i));
    }
