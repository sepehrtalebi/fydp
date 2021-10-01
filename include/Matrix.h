#pragma once

#include <array>
#include "Vector.h"

template<typename T, int n, int m>
class Matrix {
protected:
    std::array<Vector<T, m>, n> data{};
public:
    Matrix() = default;

    static Matrix<T, n, m> identity() {
        Matrix<T, n, m> output{};
        for (int i = 0; i < n && i < m; i++) output[i][i] = 1;
        return output;
    }

    static Matrix<T, n, m> zeros() {
        return {};
    }

    Matrix<T, m, n> transpose() const {
        Matrix<T, m, n> transpose;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) transpose[j][i] = data[i][j];
        return transpose;
    }

    Matrix<T, m, n> inv() const {
        // TODO
        return transpose();
    }

    Vector<T, n * m> flatten() const {
        Vector<T, n * m> flat_mat;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) flat_mat[i * m + j] = data[i][j];
        return flat_mat;
    }

    Matrix<T, n, m> operator+(const Matrix<T, n, m> &other) const {
        Matrix<T, n, m> sum;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) sum[i][j] = data[i][j] + other[i][j];
        return sum;
    }

    void operator+=(const Matrix<T, n, m> &other) {
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) data[i][j] += other[i][j];
    }

    Matrix<T, n, m> operator-(const Matrix<T, n, m> &other) const {
        Matrix<T, n, m> difference;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) difference[i][j] = data[i][j] - other[i][j];
        return difference;
    }

    void operator-=(const Matrix<T, n, m> &other) {
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) data[i][j] -= other[i][j];
    }

    Matrix<T, n, m> operator*(const T &scalar) const {
        Matrix<T, n, m> product;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) product[i][j] = data[i][j] * scalar;
        return product;
    }

    void operator*=(const T &scalar) {
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) data[i][j] *= scalar;
    }

    Matrix<T, n, m> operator/(const T &scalar) const {
        Matrix<T, n, m> quotient;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) quotient[i][j] = data[i][j] / scalar;
        return quotient;
    }

    void operator/=(const T &scalar) {
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) data[i][j] /= scalar;
    }

    template<int p>
    Matrix<T, n, p> operator*(const Matrix<T, m, p> &other) const {
        Matrix<T, n, p> product = Matrix<T, n, p>::zeros();
        for (int i = 0; i < n; i++) for (int j = 0; j < p; j++)
            for (int k = 0; k < m; k++) product[i][j] = product[i][j] + (data[i][k] * other[k][j]);
        return product;
    }

    Vector<T, n> operator*(const Vector<T, m> &vec) const {
        Vector<T, n> product{};
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++)
            product[i] = product[i] + (data[i][j] * vec[j]);
        return product;
    }

    Vector<T, m> &operator[](int index) {
        return this->data[index];
    }

    Vector<T, m> operator[](int index) const {
        return this->data[index];
    }
};
