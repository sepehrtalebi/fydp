#pragma once

#include <bmb_math/Vector.h>
#include <bmb_utilities/ConstexprUtils.h>
#include <algorithm>
#include <cmath>
#include <iostream>

template <typename T, size_t n>
class Polynomial {
  static_assert(n > 0);

 private:
  Vector<T, n> data{};

 public:
  using iterator = typename Vector<T, n>::iterator;
  using const_iterator = typename Vector<T, n>::const_iterator;

  Polynomial() = default;

  Polynomial(std::initializer_list<T> elements) {
    // Function caller must ensure the number of arguments matches the template
    // argument. Excess arguments will be ignored
    size_t i = 0;
    for (auto it = elements.begin(); i < n && it != elements.end(); it++)
      data[i++] = *it;
  }

  explicit Polynomial(const std::array<T, n>& arr) {
    for (size_t i = 0; i < n; i++) data[i] = arr[i];
  }

  Polynomial(const Vector<T, n>& vec) {  // NOLINT(google-explicit-constructor)
    for (size_t i = 0; i < n; i++) data[i] = vec[i];
  }

  template <size_t m>
  Polynomial(
      const Polynomial<T, m>& other) {  // NOLINT(google-explicit-constructor)
    *this = other;
  }

  template <size_t m>
  Polynomial& operator=(const Polynomial<T, m>& other) {
    static_assert(m <= n);
    for (size_t i = 0; i < m; i++) data[i] = other[i];
    for (size_t i = m; i < n; i++) data[i] = 0;
    return *this;
  }

  static Polynomial<T, n> identity() {
    Polynomial<T, n> p{};
    p[0] = 1;
    return p;
  }

  template <size_t p>
  Polynomial<T, (n - 1) * p + 1> pow() const {
    if constexpr (p == 0) {
      return Polynomial<T, (n - 1) * p + 1>::identity();
    } else if constexpr (p % 2 == 0) {
      return ((*this) * (*this)).template pow<p / 2>();
    } else {
      return (*this) * ((*this) * (*this)).template pow<p / 2>();
    }
  }

  T _of_(const T& scalar) const {
    T f_of_scalar = data[0];
    for (size_t i = 1; i < n; i++) {
      f_of_scalar += data[i] * std::pow(scalar, i);
    }
    return f_of_scalar;
  }

  template <size_t m>
  Polynomial<T, (n - 1) * (m - 1) + 1> _of_(
      const Polynomial<T, m>& g_of_x) const {
    Polynomial<T, (n - 1) * (m - 1) + 1> f_of_g;
    f_of_g[0] = data[0];

    bmb_utilities::constexprFor<1, n>(
        [&](auto i) { f_of_g += g_of_x.template pow<i>() * data[i]; });
    return f_of_g;
  }

  void print(const char& independent_var = 's') const {
    std::cout << data[0];
    if constexpr (n > 1) std::cout << " + " << data[1] << independent_var;
    for (size_t i = 2; i < n; i++) {
      if (data[i] != 0)
        std::cout << " + " << data[i] << independent_var << "^" << i;
    }
    std::cout << '\n';
  }

  template <size_t m>
  Polynomial<T, std::max(n, m)> operator+(const Polynomial<T, m>& other) const {
    Polynomial<T, std::max(n, m)> sum = *this;
    for (size_t i = 0; i < m; i++) sum[i] += other[i];
    return sum;
  }

  Polynomial<T, n> operator+(const T& scalar) const {
    Polynomial<T, n> sum = (*this);
    sum[0] += scalar;
    return sum;
  }

  Polynomial<T, n> operator+() const {
    Polynomial<T, n> result = (*this);
    return result;
  }

  template <size_t m>
  void operator+=(const Polynomial<T, m>& other) {
    static_assert(m <= n);
    for (size_t i = 0; i < m; i++) this->data[i] += other[i];
  }

  void operator+=(const T& scalar) { data[0] += scalar; }

  template <size_t m>
  Polynomial<T, std::max(n, m)> operator-(const Polynomial<T, m>& other) const {
    Polynomial<T, std::max(n, m)> diff = *this;
    for (size_t i = 0; i < m; i++) diff[i] -= other[i];
    return diff;
  }

  Polynomial<T, n> operator-(const T& scalar) const {
    Polynomial<T, n> diff = (*this);
    diff[0] -= scalar;
    return diff;
  }

  Polynomial<T, n> operator-() const {
    Polynomial<T, n> result;
    for (size_t i = 0; i < n; i++) result[i] = -data[i];
    return result;
  }

  template <size_t m>
  void operator-=(const Polynomial<T, m>& other) {
    static_assert(m <= n);
    for (size_t i = 0; i < m; i++) this->data[i] -= other[i];
  }

  void operator-=(const T& scalar) { data[0] -= scalar; }

  template <size_t m>
  Polynomial<T, m + n - 1> operator*(const Polynomial<T, m>& other) const {
    Polynomial<T, m + n - 1> foil;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++) foil[i + j] += other[j] * this->data[i];
    return foil;
  }

  Polynomial<T, n> operator*(const T& scalar) const {
    Polynomial<T, n> product;
    for (size_t i = 0; i < n; i++) product[i] = scalar * data[i];
    return product;
  }

  void operator*=(const T& scalar) {
    for (size_t i = 0; i < n; i++) data[i] *= scalar;
  }

  Polynomial<T, n> operator/(const T& scalar) const {
    Polynomial<T, n> quotient;
    for (size_t i = 0; i < n; i++) quotient[i] = data[i] / scalar;
    return quotient;
  }

  void operator/=(const T& scalar) {
    for (size_t i = 0; i < n; i++) data[i] /= scalar;
  }

  const T& operator[](const size_t& idx) const { return data[idx]; }

  T& operator[](const size_t& idx) { return data[idx]; }

  iterator begin() { return data.begin(); }

  iterator end() { return data.end(); }

  const_iterator begin() const { return data.begin(); }

  const_iterator end() const { return data.end(); }

  const_iterator cbegin() const { return data.begin(); }

  const_iterator cend() const { return data.end(); }
};

template <typename T, size_t n>
Polynomial<T, n> operator+(const T& scalar, const Polynomial<T, n>& poly) {
  return poly + scalar;
}

template <typename T, size_t n>
Polynomial<T, n> operator-(const T& scalar, const Polynomial<T, n>& poly) {
  return (-poly) + scalar;
}

template <typename T, size_t n>
Polynomial<T, n> operator*(const T& scalar, const Polynomial<T, n>& poly) {
  Polynomial<T, n> prod;
  for (size_t i = 0; i < n; i++)
    prod[i] = scalar * poly[i];  // respect operator order in case underlying
                                 // type is non-commutative
  return prod;
}
