#pragma once
#include <iostream>
//#include <vector>
#include "Vector.h"
#include <iostream>
#include <algorithm>
#include <exception>

template <typename T, size_t n>
class Polynomial: public Vector<T, n> {
    template<int m>
    void operator*=(const Polynomial<T, m> &other) {
        std::array<T, n> foil = {};
        int i = 0;
        while ((i < n) && (this->data[i] != 0)) {
            for (int j = 0; j < m; j++)
                foil[i + j] += other.data[j] * this->data[i];
            ++i;
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
public:
    Polynomial() = default;

    static Polynomial<T, n> identity() {
        Polynomial<T, n> p;
        p[0] = 1;
        return p;
    }

    Polynomial(const Polynomial<T, n> &other) {
        *this = other;
    }

    template<size_t m>
    Polynomial(const Polynomial<T, m> &other) { // NOLINT(google-explicit-constructor)
        *this = other;
    }

    template<size_t m>
    Polynomial& operator=(const Polynomial<T, m> &other) {
        if (m > n) throw std::invalid_argument("Cannot add dimensions to a constant size array");
        for (size_t i = 0; i < m; i++) this->data[i] = other[i];
        return *this;
    }

    template<int m>
    auto _of_(const Polynomial<T, m> &g_of_x) {
        Polynomial<T, (n - 1) * (m - 1) + 1> f_of_g;
        f_of_g[0] = this->data[0];
        for (size_t i = n - 1; i > 0; i++) {
            Polynomial<T, (n - 1) * (m - 1) + 1> poly_expansion{g_of_x};
            poly_expansion.pow(i);
            f_of_g += this->data[i] * poly_expansion;
        }
        return f_of_g;
    }

    void print() {
        for (int i = n - 1; i > 1; i--)
            std::cout << this->data[i] << "s^" << i << " + "; //TODO: setw
        if (n > 0) std::cout << this->data[1] << "s + " << this->data[0] << std::endl;
        else std::cout << this->data[0] << std::endl;
    }

    template<int m>
    auto operator*(const Polynomial<T, m> &other) {
        std::array<T, m + n - 1> foil;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                foil[i + j] += other.data[j] * this->data[i];
        return Polynomial{foil};
    }


    template<size_t m>
    auto operator+(const Polynomial<T, m> &other) {
        Polynomial<T, std::max(n, m)> sum;
        for (size_t i = 0; i < n; i++) sum[i] += this->data[i];
        for (size_t i = 0; i < m; i++) sum[i] += other.data[i];
        return sum;
    }

    template<size_t m>
    void operator+=(const Polynomial<T, m> &other) {
        if (m > n) throw std::invalid_argument("Cannot add dimensions to a constant size array");
        for (int i = 0; i < m; i++)
            this->data[i] += other.data[i];
    }

    template<size_t m>
    auto operator-(const Polynomial<T, m> &other) {
        Polynomial<T, std::max(n, m)> sum;
        for (size_t i = 0; i < n; i++) sum[i] -= this->data[i];
        for (size_t i = 0; i < m; i++) sum[i] -= other.data[i];
        return sum;
    }

    template<size_t m>
    Polynomial<T, n> operator-=(const Polynomial<T, m> &other) {
        if (m > n) throw std::invalid_argument("Cannot add dimensions to a constant size array");
        for (int i = 0; i < m; i++)
            this->data[i] -= other.data[i];
    }
};
