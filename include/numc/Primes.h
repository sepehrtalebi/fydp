#pragma once

#include <vector>

std::vector<size_t> primesUpTo(const size_t &n);

std::vector<size_t> primeFactorization(size_t n);

std::vector<size_t> uniquePrimeFactors(size_t n);

size_t lowestPrimitiveRootOfPrime(const size_t &p);

size_t powMod(size_t base, size_t exponent, const size_t &m);
