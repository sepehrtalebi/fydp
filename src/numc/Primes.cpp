#include "Primes.h"
#include <cmath>
#include <stdexcept>

std::vector<int> primesUpTo(const int &n) {
    // returns all primes <= n using the sieve of Eratosthenes
    if (n < 2) {
        return {};
    }

    std::vector<bool> is_prime(n - 1, true); // index i corresponds to the number i + 2
    std::vector<int> primes;

    // use the prime number theorem to guess the number of primes
    // this will reduce the number of times the vector needs to be resized
    primes.reserve((int) (n / log(n)));

    // TODO: use the segmented sieve of Eratosthenes to reduce memory usage for large n
    for (int i = 2; i <= n; i++) {
        if (is_prime[i - 2]) {
            primes.push_back(i);
            for (int j = 2 * i; j <= n; j += i) is_prime[j - 2] = false;
        }
    }
    primes.shrink_to_fit();
    return primes;
}

std::vector<int> primeFactorization(int n) {
    std::vector<int> factors;
    for (int candidate : primesUpTo((int) std::sqrt(n))) {
        while (n % candidate == 0) {
            factors.push_back(candidate);
            n /= candidate;
        }
    }
    if (n != 1) {
        factors.push_back(n);
    }
    return factors;
}

std::vector<int> uniquePrimeFactors(int n) {
    std::vector<int> factors;
    for (int candidate : primesUpTo((int) std::sqrt(n))) {
        if (n % candidate == 0) {
            factors.push_back(candidate);
            n /= candidate;
        }
        while (n % candidate == 0) {
            n /= candidate;
        }
    }
    if (n != 1) {
        factors.push_back(n);
    }
    return factors;
}

int lowestPrimitiveRootOfPrime(const int &p) {
    // p must be prime
    // Based on: https://math.stackexchange.com/questions/124408/finding-a-primitive-root-of-a-prime-number
    std::vector<int> test_cases = uniquePrimeFactors(p - 1);
    for (int &test_case: test_cases) test_case = p / test_case;
    for (int n = 2; n < p; n++) {
        int is_primitive_root = true;
        for (const int &test_case : test_cases) {
            if (powMod(n, test_case, p) == 1) {
                is_primitive_root = false;
                break;
            }
        }
        if (is_primitive_root) return n;
    }
    throw std::runtime_error("No primitive root found. This function only supports prime arguments!");
}

int powMod(int base, int exponent, const int &m) {
    // computes (base ** exponent) mod m
    int result = 1;
    while (exponent > 0) {
        if (exponent & 1) result = (result * base) % m;
        base = (base * base) % m;
        exponent /= 2;
    }
    return result;
}