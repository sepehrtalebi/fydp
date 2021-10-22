#include "TestPrimes.h"
#include "Primes.h"
#include <cassert>
#include <iostream>

void testPrimes() {
    std::vector<int> factors = primeFactorization(69); // 3 * 23
    assert(factors.size() == 2);
    assert(factors[0] == 3);
    assert(factors[1] == 23);

    std::vector<int> factors2 = primeFactorization(140); // 2 * 2 * 5 * 7
    assert(factors2.size() == 4);
    assert(factors2[0] == 2);
    assert(factors2[1] == 2);
    assert(factors2[2] == 5);
    assert(factors2[3] == 7);

    std::cout << "Passed All Tests for primes!" << std::endl;
}
