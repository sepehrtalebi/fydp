#include "TestPrimes.h"
#include "Primes.h"
#include <cassert>
#include <iostream>

void testPrimes() {
    std::vector<size_t> factors69 = primeFactorization(69); // 3 * 23
    assert(factors69.size() == 2);
    assert(factors69[0] == 3);
    assert(factors69[1] == 23);

    std::vector<size_t> factors140 = primeFactorization(140); // 2 * 2 * 5 * 7
    assert(factors140.size() == 4);
    assert(factors140[0] == 2);
    assert(factors140[1] == 2);
    assert(factors140[2] == 5);
    assert(factors140[3] == 7);

    std::vector<size_t> factors2 = primeFactorization(2); // 2
    assert(factors2.size() == 1);
    assert(factors2[0] == 2);

    std::vector<size_t> test_cases{2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31};
    std::vector<size_t> outputs{1, 2, 2, 3, 2, 2, 3, 2, 5, 2, 3};
    for (size_t i = 0; i < test_cases.size(); i++) {
        assert(lowestPrimitiveRootOfPrime(test_cases[i]) == outputs[i]);
    }

    std::cout << "Passed All Tests for primes!" << std::endl;
}
