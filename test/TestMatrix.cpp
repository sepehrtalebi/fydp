#include "TestMatrix.h"
#include "Matrix.h"
#include <string>
#include <cassert>
#include <iostream>

void testMatrix() {
    Matrix<uint8_t, 2, 3> mat{};
    for (size_t i = 0; i < 2; i++) for (size_t j = 0; j < 3; j++) assert(mat[i][j] == 0);

    Matrix<uint8_t, 3, 4> mat2 = Matrix<uint8_t, 3, 4>::identity();
    Matrix<uint8_t, 2, 4> product = mat * mat2;
    Matrix<uint8_t, 2, 4> z = Matrix<uint8_t, 2, 4>::zeros();
    for (size_t i = 0; i < 2; i++) for (size_t j = 0; j < 4; j++) assert(product[i][j] == z[i][j]);

    Matrix<int, 1, 2> d_mat = Matrix<int, 1, 2>::identity();
    Matrix<int, 1, 2> s_mat = d_mat.applyFunc<int>([] (const int &num) { return num * num; });
    assert(s_mat[0][0] == 1);
    assert(s_mat[0][1] == 0);

    std::cout << "Passed All Tests for Matrix!\n";
}
