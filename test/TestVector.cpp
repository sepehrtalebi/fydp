#include "TestVector.h"
#include "Vector.h"
#include <cassert>
#include <iostream>

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

    std::cout << "Passed All Tests for Vector!" << std::endl;
}
