#include "TestMatrix.h"
#include "Matrix.h"
#include <cassert>
#include <iostream>

void testMatrix() {
    Matrix<uint8_t, 2, 3> mat{};
    for (int i = 0; i < 2; i++) for (int j = 0; j < 3; j++) assert(mat[i][j] == 0);

    Matrix<uint8_t, 3, 4> mat2 = Matrix<uint8_t, 3, 4>::identity();
    Matrix<uint8_t, 2, 4> product = mat * mat2;
    Matrix<uint8_t, 2, 4> z = Matrix<uint8_t, 2, 4>::zeros();
    for (int i = 0; i < 2; i++) for (int j = 0; j < 4; j++) assert(product[i][j] == z[i][j]);

    std::cout << "Passed All Tests for Matrix!" << std::endl;
}