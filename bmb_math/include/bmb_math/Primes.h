#pragma once

#include <vector>

/**
 * @brief Computes all prime numbers less than or equal to n
 *
 * @param n The limit up to which to calculate primes
 * @return A std::vector<size_t> of prime numbers less than or equal to n
 */
std::vector<size_t> primesUpTo(const size_t& n);

/**
 * @brief Computes the prime factors of n in sorted order
 * Repeated factors are repeated in the resulting vector
 *
 * @param n The number to calculate the prime factors of
 * @return A std::vector<size_t> of the prime factors of n
 */
std::vector<size_t> primeFactorization(size_t n);

std::vector<size_t> uniquePrimeFactors(size_t n);

size_t lowestPrimitiveRootOfPrime(const size_t& p);

size_t powMod(size_t base, size_t exponent, const size_t& m);
