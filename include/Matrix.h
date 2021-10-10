#pragma once

#include <array>
#include<stdexcept>
#include <iostream>
#include "Vector.h"

template<typename T, int n, int m>
class Matrix {
protected:
    std::array<Vector<T, m>, n> data{};
public:
    Matrix() = default;

    Matrix(std::initializer_list<T> elements) {
        // Function caller must ensure the number of arguments matches the template arguments
        // Excess arguments will be ignored
        int i = 0;
        int j = 0;
        for (auto it = elements.begin(); i < n && it != elements.end(); it++) {
            data[i][j] = *it;
            j++;
            if (j == m) {
                j = 0;
                i++;
            }
        }
    }

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


    Vector<T, n * m> flatten() const {
        Vector<T, n * m> flat_mat;
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) flat_mat[i * m + j] = data[i][j];
        return flat_mat;
    }

    // conceptually this function should be an instance method of the Vector class,
    // but that is not possible due to the resulting circular dependence between Vector and Matrix
    static Matrix<T, n, m> outerProduct(const Vector<T, n> &first, const Vector<T, m> &second) {
        Matrix<T, n, m> outerProduct;
        for (int i = 0; i < n; i++) for (int j = 0; j < n; j++)
            outerProduct[i][j] = first[i] * second[j];
        return outerProduct;
    }

    template<typename R>
    Matrix<R, n, m> applyFunc(const std::function<R(T)> &func) {
        Matrix<R, n, m> result;
        for (int i = 0; i < n; i++) result[i] = data[i].applyFunc(func);
        return result;
    }



    Matrix<T, m, n> cholesky() const {
        if (m != n) throw std::invalid_argument("Cannot find the Cholesky Decomposition of a non-square matrix");
        Matrix<T, m, n> L; //Lower-triangular matrix defined by A=LL'
        for (int i = 0; i < m; i++) {
            for (int j = 0; j <= i; j++) {
                float sum = 0;
                for (int k = 0; k < j; k++)
                    sum += L[i][k] * L[j][k];

                if (i == j)
                    L[i][j] = std::sqrt(this->data[i][i] - sum);
                else
                    L[i][j] = (1.0 / L[j][j] * (this->data[i][j] - sum));
            }
        }
        return L;
    }

    Matrix<T, m, n> inv() const {
        if (m != n) throw std::invalid_argument("Cannot find the inverse of a non-square matrix");
        std::cout << "Are you sure you want to use the inverse? This is very inefficient." << std::endl;
        Matrix<T, m, n> L = cholesky();
        Matrix<T, m, n> u;
        //TODO: forward substitution then back substitution
        return transpose();
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
        Matrix<T, n, p> product{};
        for (int i = 0; i < n; i++) for (int j = 0; j < p; j++)
            for (int k = 0; k < m; k++) product[i][j] += data[i][k] * other[k][j];
        return product;
    }

    Vector<T, n> operator*(const Vector<T, m> &vec) const {
        Vector<T, n> product{};
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++)
            product[i] += data[i][j] * vec[j];
        return product;
    }

    Vector<T, m> &operator[](int index) {
        return this->data[index];
    }

    Vector<T, m> operator[](int index) const {
        return this->data[index];
    }
};
