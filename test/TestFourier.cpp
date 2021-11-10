#include "TestFourier.h"
#include "Fourier.h"
#include <cmath>
#include <chrono>
#include <random>
#include <vector>
#include <complex>
#include <cassert>
#include <iostream>

static unsigned seed = std::chrono::system_clock::now().time_since_epoch().count(); // NOLINT(cert-err58-cpp)
static auto random_engine = std::default_random_engine(seed); // NOLINT(cert-err58-cpp)
static std::uniform_real_distribution<double> distribution(-100, 100); // NOLINT(cert-err58-cpp)

#define BENCHMARK 0

void testFFT(const size_t &N) {
    std::vector<std::complex<double>> test1;
    std::vector<std::complex<double>> test2;
    test1.reserve(N);
    test2.reserve(N);
    for (size_t i = 0; i < N; i++) {
        double real = distribution(random_engine);
        double imag = distribution(random_engine);
        test1.emplace_back(real, imag);
        test2.emplace_back(real, imag);
    }
#if BENCHMARK == 1
    auto t1 = std::chrono::high_resolution_clock::now();
    Fourier<double>::fft(test1.begin(), test1.end());
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fft_ms = t2 - t1;

    t1 = std::chrono::high_resolution_clock::now();
    Fourier<double>::dft(test2.begin(), 1, Fourier<double>::getRootsOfUnity(N), N);
    t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> dft_ms = t2 - t1;

    std::cout << "FFT is " << dft_ms.count() / fft_ms.count() << " times faster for N = " << N << std::endl;
#else
    Fourier<double>::fft(test1.begin(), test1.end());
    Fourier<double>::dft(test2.begin(), 1, Fourier<double>::getRootsOfUnity(N), N);
#endif

    for (size_t i = 0; i < N; i++) {
        std::complex<double> error = test1[i] - test2[i];
        assert(std::abs(error.real()) / abs(test2[i]) < 0.0001);
        assert(std::abs(error.imag()) / abs(test2[i]) < 0.0001);
    }
}

void testFourier() {
    for (size_t i = 2; i < 1000; i++) {
        testFFT(i);
    }
    testFFT(10000);

    std::cout << "Passed All Tests for Fourier!" << std::endl;
}
