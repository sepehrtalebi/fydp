#pragma once

#include <array>
#include "Vector.h"
#include "Matrix.h"

template<typename T, int n, int m, int p>
class Matrix3D {
protected:
    std::array<Matrix<T, m, p>, n> data{};
public:
    Matrix3D() = default;

    static Matrix3D<T, n, m, p> zeros() {
        return {};
    }

    Vector<T, n * m * p> flatten() const {
        Vector<T, n * m * p> flat_mat;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) for (int k = 0; k < p; k++)
            flat_mat[i * m * p + j * p + k] = data[i][j][k];
        return flat_mat;
    }

    Matrix3D<T, n, m, p> operator+(const Matrix3D<T, n, m, p> &other) const {
        Matrix3D<T, n, m, p> sum;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) for (int k = 0; k < p; k++)
            sum[i][j][k] = data[i][j][k] + other[i][j][k];
        return sum;
    }

    void operator+=(const Matrix3D<T, n, m, p> &other) {
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) for (int k = 0; k < p; k++)
            data[i][j][k] += other[i][j][k];
    }

    Matrix3D<T, n, m, p> operator-(const Matrix3D<T, n, m, p> &other) const {
        Matrix3D<T, n, m, p> difference;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) for (int k = 0; k < p; k++)
            difference[i][j][k] = data[i][j][k] - other[i][j][k];
        return difference;
    }

    void operator-=(const Matrix3D<T, n, m, p> &other) {
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) for (int k = 0; k < p; k++)
            data[i][j][k] -= other[i][j][k];
    }

    Matrix3D<T, n, m, p> operator*(const T &scalar) const {
        Matrix3D<T, n, m, p> product;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) for (int k = 0; k < p; k++)
            product[i][j][k] = data[i][j][k] * scalar;
        return product;
    }

    void operator*=(const T &scalar) {
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) for (int k = 0; k < p; k++) data[i][j][k] *= scalar;
    }

    Matrix3D<T, n, m, p> operator/(const T &scalar) const {
        Matrix3D<T, n, m, p> quotient;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) for (int k = 0; k < p; k++)
            quotient[i][j][k] = data[i][j][k] / scalar;
        return quotient;
    }

    void operator/=(const T &scalar) {
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) for (int k = 0; k < p; k++)
            data[i][j][k] /= scalar;
    }

    template<int q>
    Matrix3D<T, n, m, q> operator*(const Matrix3D<T, p, q> &other) const {
        Matrix3D<T, n, m, q> product{};
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) for (int k = 0; k < q; k++)
                for (int f = 0; f < p; f++) product[i][j][k] += data[i][j][f] * other[f][k];
        return product;
    }

    Matrix<T, n, m> operator*(const Vector<T, p> &vec) const {
        Matrix<T, n, m> product{};
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) for (int k = 0; k < p; k++)
                product[i][j] += data[i][j][k] * vec[k];
        return product;
    }

    Matrix<T, m, p> &operator[](int index) {
        return this->data[index];
    }

    Matrix<T, m, p> operator[](int index) const {
        return this->data[index];
    }
};
