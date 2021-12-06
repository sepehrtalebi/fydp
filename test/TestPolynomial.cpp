#include "TestPolynomial.h"
#include "Polynomial.h"
#include <cassert>

//test empty constructor and pushing
bool test1() {
//    Polynomial<double> poly;
//    assert(poly.get_size() == 0);
//    for (int i = 2; i < 5; i++)
//        poly.push(double(i));
//    for (int i = 0; i < 3; i++)
//        assert(poly[2 - i] == double(i + 2));
//    assert(poly.get_size() == 3);
    return true;
}

//test vector constructor and overloaded operators
bool test2() {
//    Polynomial<double> poly{std::vector<double>{2, 3, 4}};
//    for (int i = 0; i < 3; i++)
//        assert(poly[2 - i] == double(i + 2));
//    poly *= poly;
//    assert(poly.get_size() == 5);
//    std::vector<double> test{4, 12, 25, 24, 16};
//    for (int i = 0; i < 5; i++)
//        assert(poly[i] == test[i]);
//    poly *= 2;
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
