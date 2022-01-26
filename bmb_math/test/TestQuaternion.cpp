#include "TestQuaternion.h"
#include <bmb_math/Quaternion.h>

#include <cassert>
#include <iostream>

void testQuaternion() {
    Quaternion<double> first = Quaternion<double>::identity();
    Quaternion<double> second{};
    second = first;
    second.q1 = 1;
    assert(first.q1 == 0);
    assert(second.q0 == 1);
    assert(second.q1 == 1);

    std::cout << "Passed All Tests for Quaternion!\n";
}
