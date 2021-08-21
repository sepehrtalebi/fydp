#pragma once

#include <array>
#include "Vector.h"

template<int n, int m>
class Matrix {
protected:
    std::array<Vector<m>, n> data{};
public:
    Matrix() = default;

    static Matrix<n, m> identity() {
        Matrix<n, m> output{};
        for (int i = 0; i < n && i < m; i++) output[i][i] = 1;
        return output;
    }

    static Matrix<n, m> zeros() {
        return {};
    }

    Matrix<m, n> transpose() const {
        Matrix<m, n> transpose;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) transpose[j][i] = this->data[i][j];
        return transpose;
    }

    Matrix<m, n> inv() const {
        // TODO
        return transpose();
    }

    Vector<n * m> flatten() const {
        Vector<n * m> flat_mat;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) flat_mat[i * m + j] = this->data[i][j];
        return flat_mat;
    }

    Matrix<n, m> operator+(const Matrix<n, m> &other) const {
        Matrix<n, m> sum;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) sum[i][j] = this->data[i][j] + other[i][j];
        return sum;
    }

    void operator+=(const Matrix<n, m> &other) {
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) this->data[i][j] += other[i][j];
    }

    Matrix<n, m> operator-(const Matrix<n, m> &other) const {
        Matrix<n, m> sum;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) sum[i][j] = this->data[i][j] - other[i][j];
        return sum;
    }

    void operator-=(const Matrix<n, m> &other) {
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) this->data[i][j] -= other[i][j];
    }

    Matrix<n, m> operator*(const double &scalar) const {
        Matrix<n, m> product;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) product[i][j] *= scalar;
        return product;
    }

    void operator*=(const double &scalar) {
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) this->data[i][j] *= scalar;
    }

    Matrix<n, m> operator/(const double &scalar) const {
        Matrix<n, m> product;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) product[i][j] /= scalar;
        return product;
    }

    void operator/=(const double &scalar) {
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) this->data[i][j] /= scalar;
    }

    template<int p>
    Matrix<n, p> operator*(const Matrix<m, p> &other) const {
        Matrix<n, p> product;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < p; j++) {
                product[i][j] = 0;
                for (int k = 0; k < m; k++) product[i][j] += this->data[i][k] * other[k][j];
            }
        return product;
    }

    Vector<n> operator*(const Vector<m> &vec) const {
        Vector<n> product;
        for (int i = 0; i < n; i++) {
            product[i] = 0;
            for (int j = 0; j < m; j++) product[i] += this->data[i][j] * vec[j];
        }
        return product;
    }

    Vector<m> &operator[](int index) {
        return this->data[index];
    }

    Vector<m> operator[](int index) const {
        return this->data[index];
    }
};
