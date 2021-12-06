#include "TestPolynomial.h"
#include "Polynomial.h"
#include <cassert>

//test constructors and overloaded operators
bool test1() {
    Polynomial<double, 3> poly{2, 3, 4};
    for (int i = 0; i < 3; i++)
        assert(poly[2 - i] == double(i + 2));
    auto squared = poly * poly;
    std::array<double, 5> test{4, 12, 25, 24, 16};
    for (int i = 0; i < 5; i++)
        assert(poly[i] == test[i]);
    poly *= double(2);
//    for (int i = 0; i < 5; i++)
//        assert(poly[i] == 2 * test[i]);
//    poly += Polynomial<double> {test};
//    for (int i = 0; i < 5; i++)
//        assert(poly[i] == 3 * test[i]);
    return true;
}

bool test3() {
//    Polynomial<double> poly{std::vector<double>{3, 0, 4}};
//    Polynomial<double> other{std::vector<double>{1, 0, 2}};
//    poly *= other;
//    assert(poly.get_size() == 5);
//    std::vector<double> truth{3, 0, 10, 0, 8};
//    for (int i = 0; i < 5; i++) {
//        assert(poly[i] == truth[i])
//    }
//    poly += other;
//    assert(poly.get_size() == 5);
//    truth = {3, 0, 11, 0, 10};
//    for (int i = 0; i < 5; i++) {
//        assert(poly[i] == truth[i])
//    }
    return true;
}

void testPolynomial() {
    test1();
    test2();
    test3();
}
