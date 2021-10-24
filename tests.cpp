#include "test/TestVector.h"
#include "test/TestMatrix.h"
#include "test/TestDifferentiation.h"
#include "test/TestPrimes.h"
#include "test/TestFourier.h"
#include <iostream>

int main() {
    testVector();
    testMatrix();
    testDifferentiation();
    testPrimes();
    testFourier();
    std::cout << "Passed All Tests!" << std::endl;
    return 0;
}
