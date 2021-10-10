#pragma once

#include <array>
#include <initializer_list>
#include <cmath>
#include <functional>

template<typename T, int n>
class Vector {
public:
    std::array<T, n> data{};
public:
    Vector() = default;

    Vector(std::initializer_list<T> elements) {
        // Function caller must ensure the number of arguments matches the template argument
        // Excess arguments will be ignored
        int i = 0;
        for (auto it = elements.begin(); i < n && it != elements.end(); it++)
            data[i++] = *it;
    }

    Vector(const Vector<T, n> &other) {
        for (int i = 0; i < n; i++) data[i] = other[i];
    }

    T magnitude() const {
        T sum{};
        for (int i = 0; i < n; i++) sum += data[i] * data[i];
        return sqrt(sum);
    }

    void normalize() {
        T size = this->magnitude();
        for (int i = 0; i < n; i++) data[i] /= size;
    }

    T dot(const Vector<T, n> &other) const {
        T sum{};
        for (int i = 0; i < n; i++) sum += data[i] * other[i];
        return sum;
    }

    template<int m>
    Vector<T, n + m> concatenate(const Vector<T, m> &other) const {
        Vector<T, n + m> concat;
        for (int i = 0; i < n; i++) concat[i] = data[i];
        for (int i = n; i < n + m; i++) concat[i] = other[i];
        return concat;
    }

    template<typename R>
    Vector<R, n> applyFunc(const std::function<R(T)> &func) {
        Vector<R, n> result;
        for (int i = 0; i < n; i++) result[i] = func(data[i]);
        return result;
    }

    Vector<T, n> operator+(const Vector<T, n> &other) const {
        Vector<T, n> sum;
        for (int i = 0; i < n; i++) sum[i] = data[i] + other[i];
        return sum;
    }

    void operator+=(const Vector<T, n> &other) {
        for (int i = 0; i < n; i++) data[i] += other[i];
    }

    Vector<T, n> operator+(const double &scalar) {
        Vector<T, n> sum;
        for (int i = 0; i < n; i++) sum[i] = data[i] + scalar;
        return sum;
    }

    void operator+=(const double &scalar) {
        for (int i = 0; i < n; i++) data[i] += scalar;
    }

    Vector<T, n> operator-(const Vector<T, n> &other) const {
        Vector<T, n> sum;
        for (int i = 0; i < n; i++) sum[i] = data[i] - other[i];
        return sum;
    }

    void operator-=(const Vector<T, n> &other) {
        for (int i = 0; i < n; i++) data[i] -= other[i];
    }

    Vector<T, n> operator-(const double &scalar) {
        Vector<T, n> sum;
        for (int i = 0; i < n; i++) sum[i] = data[i] - scalar;
        return sum;
    }

    void operator-=(const double &scalar) {
        for (int i = 0; i < n; i++) data[i] -= scalar;
    }

    Vector<T, n> operator*(const T &scalar) const {
        Vector<T, n> product;
        for (int i = 0; i < n; i++)
            product[i] = data[i] * scalar; // respect operator order in case the underlying type is non-commutative
        return product;
    }

    void operator*=(const T &scalar) {
        for (int i = 0; i < n; i++) data[i] *= scalar;
    }

    Vector<T, n> operator/(const T &scalar) const {
        Vector<T, n> product;
        for (int i = 0; i < n; i++) product[i] = data[i] / scalar;
        return product;
    }

    void operator/=(const T &scalar) {
        for (int i = 0; i < n; i++) data[i] /= scalar;
    }

    T &operator[](int index) {
        return this->data[index];
    }

    T operator[](int index) const {
        return this->data[index];
    }
};

template<typename T, int n>
Vector<T, n> operator+(const double &scalar, const Vector<T, n> vec) {
    Vector<T, n> sum;
    for (int i = 0; i < n; i++)
        sum[i] = scalar + vec[i]; // respect operator order in case underlying type is non-commutative
    return sum;
}

template<typename T, int n>
Vector<T, n> operator-(const double &scalar, const Vector<T, n> vec) {
    Vector<T, n> sum;
    for (int i = 0; i < n; i++)
        sum[i] = scalar - vec[i]; // respect operator order in case underlying type is non-commutative
    return sum;
}

template<typename T, int n>
Vector<T, n> operator*(const double &scalar, const Vector<T, n> vec) {
    Vector<T, n> sum;
    for (int i = 0; i < n; i++)
        sum[i] = scalar * vec[i]; // respect operator order in case underlying type is non-commutative
    return sum;
}

template<typename T, int n>
Vector<T, n> operator/(const double &scalar, const Vector<T, n> vec) {
    Vector<T, n> sum;
    for (int i = 0; i < n; i++)
        sum[i] = scalar / vec[i]; // respect operator order in case underlying type is non-commutative
    return sum;
}
