#pragma once

#include "Primes.h"
#include <complex>
#include <iostream>

template<typename T>
class Fourier {
    typedef typename std::vector<std::complex<T>>::iterator iterator;
public:
    static void fft(iterator begin, iterator end) {
        // Based on: https://numericalrecipes.wordpress.com/2009/05/29/the-cooley-tukey-fft-algorithm-for-general-factorizations/
        size_t N = end - begin;
        if (N <= 1) return;
        FactorTree *factor_tree = planFFT(N);
        std::vector<std::complex<T>> roots_of_unity = getRootsOfUnity(N);
        cooleyTukeyFFT(begin, end, 1, factor_tree, roots_of_unity);
        delete factor_tree;
    }

    static void inverseFFT(iterator begin, iterator end) {
        for (iterator it = begin; it < end; it++) {
            (*it) = std::conj(*it);
        }

        fft(begin, end);

        size_t N = (end - begin);
        for (iterator it = begin; it < end; it++) {
            (*it) = std::conj(*it) / N;
        }
    }

public:  // private:
    struct FactorTree {
        size_t value;
        FactorTree *left = nullptr;
        FactorTree *right = nullptr;

        [[nodiscard]] bool isLeaf() const {
            return left == nullptr;
        }

        ~FactorTree() {
            delete left;
            delete right;
        }

        [[nodiscard]] unsigned int multiplicationCount() const {
            if (isLeaf()) return value * value;
            return right->value * left->multiplicationCount() + left->value * right->multiplicationCount() + value;
        }
    };

    static FactorTree *planFFT(const size_t &N) {
        std::vector<size_t> factors = primeFactorization(N);

        FactorTree *tree = new FactorTree();
        tree->value = factors[factors.size() - 1];

        for (size_t i = factors.size() - 2; i >= 0; i--) {
            FactorTree *left = new FactorTree();
            left->value = factors[i];
            FactorTree *right = tree;

            tree = new FactorTree();
            tree->value = left->value * right->value;
            tree->left = left;
            tree->right = right;
        }
        return tree;
    }

    static void cooleyTukeyFFT(iterator begin, iterator end, const size_t &incr,
                               FactorTree *factor_tree,
                               const std::vector<std::complex<T>> &roots_of_unity) {
        const size_t &N = factor_tree->value;

        if (factor_tree->isLeaf()) {
            // prime number of elements
            dft(begin, incr, roots_of_unity, N);
            return;
        }
        const size_t &P = factor_tree->left->value;
        const size_t &Q = factor_tree->right->value;

        for (size_t i = 0; i < Q; i++) {
            cooleyTukeyFFT(begin + i * incr, end, incr * Q, factor_tree->left, roots_of_unity);
        }

        // twiddle factors
        for (size_t p = 0; p < P; p++) for (size_t q = 0; q < Q; q++) {
            begin[(Q * p + q) * incr] *= roots_of_unity[p * q * incr];
        }

        for (size_t i = 0; i < P; i++) {
            cooleyTukeyFFT(begin + i * Q * incr, end, incr, factor_tree->right, roots_of_unity);
        }

        transpose(begin, incr, P, Q);
    }

    static void dft(iterator begin, const size_t &incr, const std::vector<std::complex<T>> &roots_of_unity,
                    const size_t &N) {
        // should only be used for prime sized data
        size_t ratio = roots_of_unity.size() / N;
        std::vector<std::complex<T>> result(N);
        for (size_t i = 0; i < N; i++) {
            for (size_t j = 0; j < N; j++) {
                result[i] += begin[j * incr] * roots_of_unity[((i * j) % N) * ratio];
            }
        }

        for (size_t i = 0; i < N; i++) {
            begin[i * incr] = result[i];
        }
    }

    static std::vector<std::complex<T>> getRootsOfUnity(const size_t &N) {
        // the kth element of the output is e^[i*(-2*pi*k/N)]
        // compute roots of unity via repeated multiplication
        // this avoids repetitive use of std::polar which is slower since it requires sin and cos
        std::vector<std::complex<T>> roots_of_unity(N);
        roots_of_unity[0] = std::complex<T>(1, 0);
        roots_of_unity[1] = std::polar(1.0, (-2 * M_PI / N));
        for (size_t i = 2; i < N; i++) {
            roots_of_unity[i] = roots_of_unity[1] * roots_of_unity[i - 1];
        }
        return roots_of_unity;
    }

    static void transpose(iterator begin, const size_t &incr, const size_t &P, const size_t &Q) {
        // Based on: https://en.wikipedia.org/wiki/In-place_matrix_transposition#Properties_of_the_permutation
        size_t N = P * Q;
        // first and last points of the matrix never need to move, so we only need N - 2 bits of auxiliary storage
        // thus, whether the ith element of the matrix has been moved is stored in the (i - 1)th index of moved
        std::vector<bool> moved(N - 2);
        for (size_t i = 1; i < N - 1; i++) {
            if (!moved[i - 1]) {
                std::complex<T> last_value = begin[i * incr];
                moved[i - 1] = true;
                size_t next_index = (Q * i) % (N - 1);
                while (next_index != i) {
                    moved[next_index - 1] = true;
                    std::swap(last_value, begin[next_index * incr]);
                    next_index = (Q * next_index) % (N - 1);
                }
                begin[i * incr] = last_value;
            }
        }
    }
};
