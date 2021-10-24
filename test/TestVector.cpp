#include "TestVector.h"
#include "Vector.h"
#include "Expression.h"
#include "Variable.h"
#include <cassert>
#include <iostream>

static int roundDown(const double &num) {
    return (int) num;
}

void testVector() {
    Vector<double, 5> x;
    for (int i = 0; i < 5; i++) assert(x[i] == 0);
    Vector<double, 5> y;
    x[0] = 2;
    y[0] = 4;
    x[1] = -4;
    y[1] = 7;

    Vector<double, 5> z = x + y;
    assert(z[0] == 6);
    assert(z[1] == 3);
    y[2] = 8;
    assert(z[2] == 0);
    z += y;
    assert(z[2] == 8);
    assert(z[0] == 10);

    Vector<double, 5> test{1.1, -4.5, 3, 4, 0};
    Vector<int, 5> test_floor = test.applyFunc<int>(&roundDown);
    for (int i = 0; i < 5; i++) assert(test_floor[i] == (int) test[i]);

    Vector<int, 5> test_sq_int = test.applyFunc<int>([](const double &d) { return (int) (d * d); });
    for (int i = 0; i < 5; i++) assert(test_sq_int[i] == (int) (test[i] * test[i]));

    Vector<ExprPtr, 3> expr{std::make_shared<Variable>("x"), std::make_shared<Variable>("y"),
                            std::make_shared<Variable>("z")};
    const std::map<std::string, double> subs{{"x", 0},
                                             {"y", 1},
                                             {"z", 2}};
    Vector<double, 3> expr_sub = expr.applyFunc<double>([&subs](const ExprPtr &e) { return e->evaluate(subs); });
    for (int i = 0; i < 3; i++) assert(expr_sub[i] == i);

    std::cout << "Passed All Tests for Vector!" << std::endl;
}
