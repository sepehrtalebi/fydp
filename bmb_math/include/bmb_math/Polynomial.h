#pragma once

#include <bmb_math/Vector.h>
#include <bmb_utilities/ConstexprUtils.h>

#include <algorithm>
#include <cmath>
#include <iostream>

template <typename T, size_t n>
class Polynomial: public Vector<T, n> {
 public:
    static_assert(n > 0);

    using Vector<T, n>::Vector;

    Polynomial(const Polynomial<T, n> &other) {
        *this = other;
    }

    template<size_t m>
    Polynomial(const Polynomial<T, m> &other) { // NOLINT(google-explicit-constructor)
        *this = other;
    }

    explicit Polynomial(const std::array<T, n> &arr) {
        for (size_t i = 0; i < n; i++) this->data[i] = arr[i];
    }

    Polynomial(const Vector<T, n> &vec) { // NOLINT(google-explicit-constructor)
        for (size_t i = 0; i < n; i++) this->data[i] = vec[i];
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

    template<size_t p>
    Polynomial<T, (n - 1) * p + 1> pow() const {
      static constexpr size_t p2 = p / 2;
      if constexpr (p == 0) {
        return Polynomial<T, (n - 1) * p + 1>::identity();
      }
      else if constexpr (p % 2 == 0) {
        return ((*this) * (*this)).template pow<p2>();
      }
      else {
        return (*this) * ((*this) * (*this)).template pow<p2>();
      }
    }

    T _of_(const T& scalar) const {
        T f_of_scalar = this->data[0];
        for (size_t i = 1; i < n; i++) {
            f_of_scalar += this->data[i] * std::pow(scalar, i);
        }
        return f_of_scalar;
    }

    template<size_t m>
    Polynomial<T, (n - 1) * (m - 1) + 1> _of_(const Polynomial<T, m> &g_of_x) const {
        Polynomial<T, (n - 1) * (m - 1) + 1> f_of_g;
        f_of_g[0] = this->data[0];

        bmb_utilities::constexprFor<1, n>([&](auto i) {
          f_of_g += g_of_x.template pow<i>() * this->data[i];
        });
        return f_of_g;
    }

    void print(const char& independent_var = 's') {
        std::cout << this->data[0];
        if constexpr (n > 1)  std::cout << " + " << this->data[1] << independent_var;
        for (size_t i = 2; i < n; i++) {
            if (this->data[i] != 0)
                std::cout << " + " << this->data[i] << independent_var << "^" << i;
        }
        std::cout << std::endl;
    }

    using Vector<T, n>::operator*;

    template<size_t m>
    Polynomial<T, m + n - 1> operator*(const Polynomial<T, m> &other) const {
        Polynomial<T, m + n - 1> foil;
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < m; j++)
                foil[i + j] += other[j] * this->data[i];
        return foil;
    }

    using Vector<T, n>::operator*=;

    using Vector<T, n>::operator+;

    Polynomial<T, n> operator+(const T &scalar) const {
        Polynomial<T, n> sum = (*this);
        sum[0] += scalar;
        return sum;
    }

    template<size_t m>
    Polynomial<T, std::max(n, m)> operator+(const Polynomial<T, m> &other) const {
        Polynomial<T, std::max(n, m)> sum = *this;
        for (size_t i = 0; i < m; i++) sum[i] += other[i];
        return sum;
    }

    template<size_t m>
    void operator+=(const Polynomial<T, m> &other) {
        static_assert(m <= n);
        for (size_t i = 0; i < m; i++)
            this->data[i] += other[i];
    }

    using Vector<T, n>::operator-;

    Polynomial<T, n> operator-(T &scalar) const {
        Polynomial<T, n> diff = (*this);
        diff[0] -= scalar;
        return diff;
    }

    template<size_t m>
    Polynomial<T, std::max(n, m)> operator-(const Polynomial<T, m> &other) const {
        Polynomial<T, std::max(n, m)> diff = *this;
        for (size_t i = 0; i < m; i++) diff[i] -= other[i];
        return diff;
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
