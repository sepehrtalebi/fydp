#pragma once

#include <array>
#include <initializer_list>
#include <cmath>

template<int n>
class Vector {
protected:
    std::array<double, n> data{};
public:
    Vector() = default;

    Vector(std::initializer_list<double> elements) {
        // Function caller must ensure the number of arguments matches the template argument
        // Excess arguments will be ignored
        int i = 0;
        for (std::initializer_list<double>::iterator it = elements.begin(); i < n && it != elements.end(); it++)
            data[i++] = *it;
    }

    Vector(const Vector<n> &other) {
        for (int i = 0; i < n; i++) this->data[i] = other.data[i];
    }

    double magnitude() const {
        double sum = 0;
        for (int i = 0; i < n; i++) sum += data[i] * data[i];
        return sqrt(sum);
    }

    void normalize() {
        double size = this->magnitude();
        for (int i = 0; i < n; i++) this->data[i] /= size;
    }

    double dot(const Vector<n> &other) const {
        double sum = 0;
        for (int i = 0; i < n; i++) sum += this->data[i] * other.data[i];
        return sum;
    }

    template<int m>
    Vector<n + m> concatenate(const Vector<m> &other) const {
        Vector<n + m> concat;
        for (int i = 0; i < n; i++) concat[i] = this->data[i];
        for (int i = n; i < n + m; i++) concat[i] = other[i];
        return concat;
    }

    Vector<n> operator+(const Vector<n> &other) const {
        Vector<n> sum;
        for (int i = 0; i < n; i++) sum[i] = this->data[i] + other.data[i];
        return sum;
    }

    void operator+=(const Vector<n> &other) {
        for (int i = 0; i < n; i++) this->data[i] += other.data[i];
    }

    Vector<n> operator-(const Vector<n> &other) const {
        Vector<n> sum;
        for (int i = 0; i < n; i++) sum[i] = this->data[i] - other.data[i];
        return sum;
    }

    void operator-=(const Vector<n> &other) {
        for (int i = 0; i < n; i++) this->data[i] -= other.data[i];
    }

    Vector<n> operator*(const double &scalar) const {
        Vector<n> product;
        for (int i = 0; i < n; i++) product[i] = scalar * this->data[i];
        return product;
    }

    void operator*=(const double &scalar) {
        for (int i = 0; i < n; i++) this->data[i] *= scalar;
    }

    Vector<n> operator/(const double &scalar) const {
        Vector<n> product;
        for (int i = 0; i < n; i++) product[i] = this->data[i] / scalar;
        return product;
    }

    void operator/=(const double &scalar) {
        for (int i = 0; i < n; i++) this->data[i] /= scalar;
    }

    double &operator[](int index) {
        return this->data[index];
    }

    double operator[](int index) const {
        return this->data[index];
    }
};
