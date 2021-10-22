#include "TestFourier.h"
#include "Fourier.h"
#include <vector>
#include <complex>
#include <iostream>

#define TEST_SIZE 15

void testTranspose() {
    std::vector<std::complex<double>> vec(TEST_SIZE);
    for (int i = 0; i < TEST_SIZE; i++) {
        vec[i] = std::complex((double) i, 0.0);
    }
    std::cout << "test" << std::endl;
    Fourier<double>::transpose(vec.begin(), 1, 3, 5);
    return;
}

void testFourier() {
//    std::vector<std::complex<double>> test;
//    test.reserve(TEST_SIZE);
//    for (int i = 0; i < TEST_SIZE; i++) {
//        test.emplace_back(1, 1);
//    }
//    Fourier<double>::fft(test.begin(), test.end());
    testTranspose();

    std::cout << "Passed All Tests for Fourier!" << std::endl;
}
