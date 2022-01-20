#include "TestPolynomial.h"
#include "../include/bmb_math/Polynomial.h"
#include <cassert>

//test constructors and operators *= += *
bool test1() {
    Polynomial<double, 3> poly{4, 3, 2};
    for (int i = 0; i < 3; i++)
        assert(poly[i] == double(4 - i));
    auto test = poly * poly;
    std::array<double, 5> truth{16, 24, 25, 12, 4};
    for (int i = 0; i < 5; i++)
        assert(test[i] == truth[i]);
    test *= 2;
    for (int i = 0; i < 5; i++)
        assert(test[i] == 2 * truth[i]);
    test += Polynomial<double, 5> {truth};
    test.print();
    for (int i = 0; i < 5; i++)
        assert(test[i] == 3 * truth[i]);
    return true;
}

//test * edge cases
bool test2() {
    Polynomial<double, 3> poly{4, 0, 3};
    Polynomial<double, 3> other{2, 0, 1};
    Polynomial<double, 5> test = poly * other;
    std::array<double, 5> truth{8, 0, 10, 0, 3};
    for (int i = 0; i < 5; i++)
        assert(test[i] == truth[i]);
    test += other;
    truth = {10, 0, 11, 0, 3};
    for (int i = 0; i < 5; i++)
        assert(test[i] == truth[i]);
    test.print();
    test *= 0;
    for (int i = 0; i < 5; i++)
        assert(test[i] == 0);
    return true;
}

//test composite method _of_
bool test3() {
    Polynomial<double, 4> f = {1, 3, 0, 5};
    Polynomial<double, 3> g{5, 0, 6};
    auto f_of_g = f._of_(g);
    Polynomial<double, 7> truth{641, 0, 2268, 0, 2700, 0, 1080};
    for (int i = 0; i < 7; i++)
        assert(f_of_g[i] == truth[i]);
    f_of_g.print();
    return true;
}

void testPolynomial() {
    test1();
    test2();
    test3();
}
