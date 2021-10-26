#include "TestPrimes.h"
#include "Primes.h"
#include <cassert>
#include <iostream>

void testPrimes() {
    std::vector<int> factors69 = primeFactorization(69); // 3 * 23
    assert(factors69.size() == 2);
    assert(factors69[0] == 3);
    assert(factors69[1] == 23);

    std::vector<int> factors140 = primeFactorization(140); // 2 * 2 * 5 * 7
    assert(factors140.size() == 4);
    assert(factors140[0] == 2);
    assert(factors140[1] == 2);
    assert(factors140[2] == 5);
    assert(factors140[3] == 7);

    std::vector<int> factors2 = primeFactorization(2); // 2
    assert(factors2.size() == 1);
    assert(factors2[0] == 2);

    std::cout << "Passed All Tests for primes!" << std::endl;
}
