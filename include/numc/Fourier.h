#pragma once

#include "Primes.h"
#include <complex>
#include <iostream>

template<typename T>
class Fourier {
    typedef typename std::vector<std::complex<T>>::iterator iterator;
    typedef std::vector<int>::iterator factors_iterator;
public:
    static void fft(iterator begin, iterator end) {
        int N = end - begin;
        if (N <= 1) return;
        std::vector<int> factors = primeFactorization(N);
        fft_impl(begin, end, 1, factors.begin(), factors.end());
    }

    static void inverse_fft(iterator begin, iterator end) {
        for (iterator it = begin; it < end; it++) {
            (*it) = std::conj(*it);
        }

        fft(begin, end);

        int N = (end - begin);
        for (iterator it = begin; it < end; it++) {
            (*it) = std::conj(*it) / N;
        }
    }

private:
    static void fft_impl(iterator begin, iterator end, const int &incr,
                         factors_iterator factors_begin, factors_iterator factors_end) {
        int factors_count = (int) (factors_end - factors_begin);
        if (factors_count == 1) {
            // prime number of elements
            dft(begin, end, incr);
            return;
        }
        int N = (end - begin) / incr;
        int P = *factors_begin;
        int Q = N / P;
        std::cout << "fft with " << P << ", " << Q << std::endl;

        for (int i = 0; i < Q; i++) {
            fft_impl(begin + i * incr, end, incr * Q, factors_begin, factors_begin + 1);
        }

        std::cout << "twiddling fft with " << P << ", " << Q << std::endl;
        // twiddle factors
        std::vector<std::complex<T>> roots_of_unity = getRootsOfUnity(N);
        for (int p = 0; p < P; p++) for (int q = 0; q < Q; q++) {
            begin[(Q * p + q) * incr] *= roots_of_unity[(N - p * q) % N];
        }

        std::cout << "second fft with " << P << ", " << Q << std::endl;
        for (int i = 0; i < P; i++) {
            fft_impl(begin + i * incr, end, incr * P, factors_begin + 1, factors_end);
        }

        std::cout << "transposing fft with " << P << ", " << Q << std::endl;
        transpose(begin, incr, P, Q);
        std::cout << "done fft with " << P << ", " << Q << std::endl;
    }

    static void dft(iterator begin, iterator end, const int &incr) {
        // should only be used for prime sized data
        int N = (end - begin) / incr;
        std::vector<std::complex<T>> roots_of_unity = getRootsOfUnity(N);

        std::vector<std::complex<T>> result(N);
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                result[i * incr] += begin[j * incr] * roots_of_unity[(N * N - i * j) % N];
            }
        }

        for (int i = 0; i < N; i++) {
            begin[i * incr] = result[i * incr];
        }
    }

    static std::vector<std::complex<T>> getRootsOfUnity(const int &N) {
        // the kth element of the output is e^[i*(2*pi*k/N)]
        // compute roots of unity via repeated multiplication
        // this avoids repetitive use of std::polar which is slower since it requires sin and cos
        std::vector<std::complex<T>> roots_of_unity(N);
        roots_of_unity[0] = std::complex<T>(1, 0);
        roots_of_unity[1] = std::polar(1.0, (2 * M_PI / N));
        for (int i = 2; i < N; i++) {
            roots_of_unity[i] = roots_of_unity[1] * roots_of_unity[i - 1];
        }
        return roots_of_unity;
    }

public:
    static void transpose(iterator begin, const int &incr, const int &P, const int &Q) {
        int N = P * Q;
        std::vector<bool> moved(N - 1);
        moved[0] = true;
        // first and last points of the matrix never need to move
        for (int i = 1; i < N - 1; i++) {
            if (!moved[i]) {
                std::complex<T> last_value = begin[i * incr];
                moved[i] = true;
                int next_index = (Q * i) % (N - 1);
                while (next_index != i) {
                    moved[next_index] = true;
                    std::swap(last_value, begin[next_index * incr]);
                    next_index = (Q * next_index) % (N - 1);
                }
                begin[i * incr] = last_value;
            }
        }
    }
};
