#pragma once

#include <array>
#include "Vector.h"
#include "Matrix.h"
#include <functional>

template<typename T, size_t n, size_t m, size_t p>
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
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            flat_mat[i * m * p + j * p + k] = data[i][j][k];
        return flat_mat;
    }

    template<typename R>
    Matrix3D<R, n, m, p> applyFunc(const std::function<R(const T &)> &func) const {
        Matrix3D<R, n, m, p> result;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            result[i][j][k] = func(data[i][j][k]);
        return result;
    }

    Matrix3D<T, n, m, p> operator+(const Matrix3D<T, n, m, p> &other) const {
        Matrix3D<T, n, m, p> sum;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            sum[i][j][k] = data[i][j][k] + other[i][j][k];
        return sum;
    }

    void operator+=(const Matrix3D<T, n, m, p> &other) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            data[i][j][k] += other[i][j][k];
    }

    Matrix3D<T, n, m, p> operator-(const Matrix3D<T, n, m, p> &other) const {
        Matrix3D<T, n, m, p> difference;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            difference[i][j][k] = data[i][j][k] - other[i][j][k];
        return difference;
    }

    void operator-=(const Matrix3D<T, n, m, p> &other) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            data[i][j][k] -= other[i][j][k];
    }

    Matrix3D<T, n, m, p> operator*(const T &scalar) const {
        Matrix3D<T, n, m, p> product;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            product[i][j][k] = data[i][j][k] * scalar;
        return product;
    }

    void operator*=(const T &scalar) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++) data[i][j][k] *= scalar;
    }

    Matrix3D<T, n, m, p> operator/(const T &scalar) const {
        Matrix3D<T, n, m, p> quotient;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            quotient[i][j][k] = data[i][j][k] / scalar;
        return quotient;
    }

    void operator/=(const T &scalar) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            data[i][j][k] /= scalar;
    }

    Matrix<T, n, m> operator*(const Vector<T, p> &vec) const {
        Matrix<T, n, m> product{};
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
                product[i][j] += data[i][j][k] * vec[k];
        return product;
    }

    Matrix<T, m, p> &operator[](const size_t &index) {
        return this->data[index];
    }

    Matrix<T, m, p> operator[](const size_t &index) const {
        return this->data[index];
    }
};
