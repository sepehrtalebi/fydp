#include "test/TestVector.h"
#include "test/TestVector3.h"
#include "test/TestQuaternion.h"
#include "test/TestMatrix.h"
#include "test/TestDifferentiation.h"
#include "test/TestPrimes.h"
#include "test/TestFourier.h"
#include "test/TestPolynomial.h"
#include "test/TestRationalFunction.h"
#include "test/TestTransferFunction.h"
#include "test/TestDubinsPath.h"
#include "test/TestPurePursuit.h"
#include "test/TestCompiledDifferentiation.h"
#include "test/TestRatioPower.h"
#include "test/TestTypeArray.h"
#include <iostream>


int main() {
    testVector();
    testVector3();
    testQuaternion();
    testMatrix();
    testDifferentiation();
    testPrimes();
    testFourier();
    testDubinsPath();
    testPurePursuit();
    testPolynomial();
    testRationalFunction();
    testTransferFunction();
    testCompiledDifferentiation();
    testRatioPower();
    testTypeArray();
    std::cout << "Passed All Tests!" << std::endl;
    return 0;
}
