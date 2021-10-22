#include "Primes.h"
#include <cmath>

std::vector<int> primesUpTo(const int &n) {
    // returns all primes <= n
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
