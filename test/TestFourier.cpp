#include "TestFourier.h"
#include "Fourier.h"
#include <vector>
#include <complex>
#include <iostream>

#define TEST_SIZE 100

void testFourier() {
    std::vector<std::complex<double>> test;
    test.reserve(TEST_SIZE);
    for (int i = 0; i < TEST_SIZE; i++) {
        test.emplace_back(1, 1);
    }
    Fourier<double>::fft(test.begin(), test.end());

    std::cout << "Passed All Tests for Fourier!" << std::endl;
}
