#include "prime.h"
#include <cmath>

std::vector<int> primesUpTo(const int &n) {
    // returns all primes <= n
    std::vector<bool> is_prime(n - 1, true); // index i corresponds to the number i + 2

    // use the prime number theorem to guess the number of primes
    // this will reduce the number of times the vector needs to be resized
    int prime_count_guess = std::floor(n / log(n));
    std::vector<int> primes(prime_count_guess);
    int prime_count = 0;

    // TODO: use the segmented sieve of Eratosthenes to reduce memory usage
    for (int i = 2; i <= n; i++) {
        if (is_prime[i - 2]) {
            prime_count++;
            if (prime_count < prime_count_guess) primes[prime_count - 1] = i;
            else primes.push_back(i);
            for (int j = 2 * i; j <= n; j += i) is_prime[j - 2] = false;
        }
    }
    if (prime_count < prime_count_guess) {
        primes.resize(prime_count);
    }
    return primes;
}

std::vector<int> primeFactorization(const int &n) {
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
