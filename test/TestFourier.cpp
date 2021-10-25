#include "TestFourier.h"
#include "Fourier.h"
#include <cmath>
#include <vector>
#include <complex>
#include <cassert>
#include <iostream>

#define TEST_SIZE 100

void testFourier() {
    std::vector<std::complex<double>> test1;
    std::vector<std::complex<double>> test2;
    test1.reserve(TEST_SIZE);
    test2.reserve(TEST_SIZE);
    for (int i = 0; i < TEST_SIZE; i++) {
        test1.emplace_back(1, 1);
        test2.emplace_back(1, 1);
    }

    Fourier<double>::fft(test1.begin(), test1.end());
    Fourier<double>::dft(test2.begin(), test2.end(), 1, Fourier<double>::getRootsOfUnity(TEST_SIZE));

    for (int i = 0; i < TEST_SIZE; i++) {
        std::complex<double> error = test1[i] - test2[i];
        assert(std::abs(error.real()) < 0.01);
        assert(std::abs(error.imag()) < 0.01);
    }

    std::cout << "Passed All Tests for Fourier!" << std::endl;
}
