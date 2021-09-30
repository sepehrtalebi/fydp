#include "test/TestVector.h"
#include "test/TestMatrix.h"
#include "test/TestDifferentiation.h"
#include <iostream>

int main() {
    testVector();
    testMatrix();
    testDifferentiation();
    std::cout << "Passed All Tests!" << std::endl;
    return 0;
}
