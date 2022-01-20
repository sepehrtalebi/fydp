#include "TestVector3.h"
#include "../include/bmb_math/Vector3.h"
#include <cassert>
#include <iostream>

void testVector3() {
    Vector3<int> first{1, 2, 3};
    Vector3<int> second;
    second = first;
    second.x = 4;
    assert(first.x == 1);
    assert(second.x == 4);

    std::cout << "Passed All Tests for Vector3!\n";
}
