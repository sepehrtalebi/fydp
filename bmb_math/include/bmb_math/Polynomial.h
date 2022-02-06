#pragma once

#include <bmb_math/Vector.h>

#include <algorithm>
#include <cmath>
#include <iostream>

template <typename T, size_t n>
class Polynomial: public Vector<T, n> {
    static_assert(n>0);

    template<int m>
    void operator*=(const Polynomial<T, m> &other) {
        Polynomial<T, n> foil;
        for (int i = 0; i < n; i++) {
            for (int j = n - 1; j >= 0; j--) {
                if (other[j] != 0)
                    foil[i + j] += other[j] * this->data[i];
            }
        }
        (*this) = foil;
    }

    void operator*=(const Polynomial<T, n> &other) {
        Polynomial<T, n> foil;
        for (int i = n - 1; i >= 0; i--) {
            if (this->data[i] != 0) { //ignore the higher order terms of max size polynomial, very hacky
                for (int j = n - 1; j >= 0; j--) {
                    if (other[j] != 0)
                        foil[i + j] += other[j] * this->data[i];
                }
            }
        }
        (*this) = foil;
    }

    void pow(const size_t& p) {
        Polynomial<T, n> factor = *this;
        *this = Polynomial::identity();
        size_t pos = p;
        while (pos > 0) {
            if (pos % 2 == 1) {
                (*this) *= factor;
            }
            pos /= 2;
            factor *= factor;
        }
    }

    template<size_t m>
    static Polynomial<T, std::max(n, m)> hacky_product(Polynomial<T, n> first, Polynomial<T, m> other) {
        Polynomial<T, std::max(n, m)> foil;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                if ((i + j) < std::max(n, m)) foil[i + j] += other[j] * first[i];
        return foil;
    }

    template<typename, size_t>
    friend class Polynomial;
    template<typename, size_t, size_t>
    friend class RationalFunction;
    template<typename, size_t, size_t>
    friend class TransferFunction;
public:
    using Vector<T, n>::Vector;

    Polynomial(const Polynomial<T, n> &other) {
        *this = other;
    }

    template<size_t m>
    Polynomial(const Polynomial<T, m> &other) { // NOLINT(google-explicit-constructor)
        *this = other;
    }

    explicit Polynomial(const std::array<T, n> &arr) {
        for (int i = 0; i < n; i++) this->data[i] = arr[i];
    }

    Polynomial(const Vector<T, n> &vec) { // NOLINT(google-explicit-constructor)
        for (int i = 0; i < n; i++) this->data[i] = vec[i];
    }

    Polynomial& operator=(const Polynomial<T, n> &other) {
        for (size_t i = 0; i < n; i++) this->data[i] = other[i];
        return *this;
    }

    template<size_t m>
    Polynomial& operator=(const Polynomial<T, m> &other) {
        static_assert(m <= n);
        for (size_t i = 0; i < m; i++) this->data[i] = other[i];
        for (size_t i = m; i < n; i++) this->data[i] = 0;
        return *this;
    }

    static Polynomial<T, n> identity() {
        Polynomial<T, n> p;
        p[0] = 1;
        return p;
    }

    T _of_(T scalar) const {
        T f_of_scalar = this->data[0];
        for (size_t i = n - 1; i > 0; i++) {
            f_of_scalar += this->data[i] * std::pow(scalar, i);
        }
        return f_of_scalar;
    }

    template<size_t m>
    Polynomial<T, (n - 1) * (m - 1) + 1> _of_(const Polynomial<T, m> &g_of_x) const {
        Polynomial<T, (n - 1) * (m - 1) + 1> f_of_g;
        f_of_g[0] = this->data[0];
        for (size_t i = n - 1; i > 0; i--) {
            Polynomial<T, (n - 1) * (m - 1) + 1> poly_expansion{g_of_x};
            poly_expansion.pow(i);
            f_of_g += poly_expansion * this->data[i];
        }
        return f_of_g;
    }

    void print(char independent_var = 's') {
        std::cout << this->data[0];
        if (n > 1)  std::cout << " + " << this->data[1] << independent_var;
        for (int i = 2; i < n; i++) {
            if (this->data[i] != 0)
                std::cout << " + " << this->data[i] << independent_var << "^" << i ;
        }
        std::cout << std::endl;
    }

    using Vector<T, n>::operator*;

    Polynomial<T, 2 * n - 1> operator*(const Polynomial<T, n> &other) const {
        Polynomial<T, 2 * n - 1> foil;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                foil[i + j] += other[j] * this->data[i];
        return foil;
    }

    template<size_t m>
    Polynomial<T, m + n - 1> operator*(const Polynomial<T, m> &other) const {
        Polynomial<T, m + n - 1> foil;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                foil[i + j] += other[j] * this->data[i];
        return foil;
    }

    using Vector<T, n>::operator*=;

    using Vector<T, n>::operator+;

    Polynomial<T, n> operator+(T &scalar) const {
        Polynomial<T, n> sum = (*this);
        sum[0] += scalar;
        return sum;
    }

    template<size_t m>
    Polynomial<T, std::max(n, m)> operator+(const Polynomial<T, m> &other) const {
        Polynomial<T, std::max(n, m)> sum;
        for (size_t i = 0; i < n; i++) sum[i] += this->data[i];
        for (size_t i = 0; i < m; i++) sum[i] += other[i];
        return sum;
    }

    void operator+=(const Polynomial<T, n> &other) {
        for (int i = 0; i < n; i++)
            this->data[i] += other[i];
    }

    template<size_t m>
    void operator+=(const Polynomial<T, m> &other) {
        static_assert(m <= n);
        for (int i = 0; i < m; i++)
            this->data[i] += other[i];
    }

    using Vector<T, n>::operator-;

    Polynomial<T, n> operator-(T &scalar) const {
        Polynomial<T, n> diff = (*this);
        diff[0] -= scalar;
        return diff;
    }

    template<size_t m>
    Polynomial<T, n> operator-(const Polynomial<T, n> &other) const {
        Polynomial<T, n> diff;
        for (size_t i = 0; i < n; i++) diff[i] = this->data[i] - other[i];
        return diff;
    }

    template<size_t m>
    Polynomial<T, std::max(n, m)> operator-(const Polynomial<T, m> &other) const {
        Polynomial<T, std::max(n, m)> diff;
        for (size_t i = 0; i < n; i++) diff[i] -= this->data[i];
        for (size_t i = 0; i < m; i++) diff[i] -= other[i];
        return diff;
    }

    void operator-=(const Polynomial<T, n> &other) {
        for (int i = 0; i < n; i++)
            this->data[i] -= other[i];
    }

    template<size_t m>
    void operator-=(const Polynomial<T, m> &other) {
        static_assert(m <= n);
        for (int i = 0; i < m; i++)
            this->data[i] -= other[i];
    }
};

template<typename T, size_t n>
Polynomial<T, n> operator*(const T &scalar, const Polynomial<T, n> &poly) {
    Polynomial<T, n> prod;
    for (size_t i = 0; i < n; i++)
        prod[i] = scalar * poly[i]; // respect operator order in case underlying type is non-commutative
    return prod;
}