#pragma once

#include <bmb_math/Polynomial.h>
#include <bmb_utilities/ConstexprUtils.h>

#include <algorithm>
#include <iostream>
#include <vector>

// TODO: move to Utility.h
template<typename T>
static constexpr T abs_difference(const T& a, const T&b) {
    return a > b ? a - b : b - a;
}

// TODO: move to Utility.h
template<typename T>
static constexpr T heaviside_difference(const T& a, const T&b) {
    return a > b ? a - b : 0;
}

template <typename T, size_t n, size_t m>
class RationalFunction {
protected:
    Polynomial<T, n> numerator;
    Polynomial<T, m> denominator;

    template<typename, size_t, size_t>
    friend class RationalFunction;
    template<typename, size_t, size_t>
    friend class TransferFunction;
public:
    RationalFunction() = default;

    RationalFunction(std::initializer_list<T> elements) {
        // Function caller must ensure the number of arguments matches the template argument
        // Excess arguments will be ignored
        size_t i = 0;
        for (auto it = elements.begin(); i < n && it != elements.end(); it++)
            numerator[i++] = *it;
        for (auto it = elements.begin() + i; i < m + n && it != elements.end(); it++) {
            denominator[i - n] = *it;
            i++;
        }
    }

    RationalFunction(const Polynomial<T, n> &numerator, const Polynomial<T, m> &denominator){
        this->numerator = numerator;
        this->denominator = denominator;
    }

    RationalFunction(const RationalFunction<T, n, m> &other) {
        (*this) = other;
    }

    RationalFunction& operator=(const RationalFunction<T, n, m> &other) {
        this->numerator = other.numerator;
        this->denominator = other.denominator;
        return (*this);
    }

    void print(const char& independent_var = 's') {
        numerator.print(independent_var);
        for (size_t i = 0; i < std::max(n, m); i++) {
            std::cout << "-----";
        }
        std::cout << std::endl;
        denominator.print(independent_var);
    }

    T numerator_data(const size_t& num_index) const {
        return numerator[num_index];
    }

    T denominator_data(const size_t& den_index) const {
        return denominator[den_index];
    }

    template<size_t p>
    RationalFunction<T, (n - 1) * (p - 1) + 1, (m - 1) * (p - 1) + 1> _of_(const Polynomial<T, p> &p_of_x) const {
        auto num = numerator._of_(p_of_x);
        auto den = denominator._of_(p_of_x);
        return {num, den};
    }

    template<size_t p, size_t q>
    using composite_rational = RationalFunction<T, (n - 1) * std::max(p - 1, q - 1) + (q - 1) * heaviside_difference(m, n) + 1,
            (m - 1) * std::max(p - 1, q - 1) + (q - 1) * heaviside_difference(n, m) + 1>;

    template<size_t p, size_t q>
    composite_rational<p, q> _of_(const RationalFunction<T, p, q> &g_of_x) const {
        Polynomial<T, (n - 1) * std::max(p - 1, q - 1) + 1> num;
        Polynomial<T, (m - 1) * std::max(p - 1, q - 1) + 1> den;

        constexprFor<0, n>([](auto i) {
          num += numerator[i] * g_of_x.numerator.pow<i>() * g_of_x.denominator.pow<n - 1 - i>();
        });
        constexprFor<0, m>([](auto i) {
          den += denominator[i] * g_of_x.numerator.pow<i>() * g_of_x.denominator.pow<m - 1 - i>();
        });

        if constexpr (n != m) { //because the denominators of the numerator and denominator are the same, they will cancel
            static constexpr size_t abs_diff = abs_difference(m, n);
            Polynomial<T, abs_diff + 1> canceled_denominators = g_of_x.denominator.pow<abs_diff>();
            if constexpr (m > n) return {num * canceled_denominators, den};
            else return {num, den * canceled_denominators};
        }
        else return {num, den};
    }

    template<size_t p, size_t q>
    RationalFunction<T, n + p - 1, m + q - 1> operator*(const RationalFunction<T, p, q> &other) const {
        auto num = numerator * other.numerator;
        auto den = denominator * other.denominator;
        return {num, den};
    }

    template<size_t p>
    RationalFunction<T, n + p - 1, m> operator*(const Polynomial<T, p> &poly) const {
        auto num = numerator * poly;
        return {num, this->denominator};
    }

    RationalFunction<T, n, m> operator*(const T &scalar) const {
        Polynomial<T, n> num;
        num = numerator * scalar;
        return {num, this->denominator};
    }

    void operator*=(const T &scalar) {
        numerator *= scalar;
    }

    template<size_t p, size_t q>
    RationalFunction<T, std::max(n + q - 1, m + p - 1), m + q - 1>
            operator+(const RationalFunction<T, p, q> &other) const {
        Polynomial<T, std::max(n + q - 1, m + p - 1)> num;
        Polynomial<T, m + q - 1> den;
        den = denominator * other.denominator;
        num = numerator * other.denominator;
        num += denominator * other.numerator;
        return {num, den};
    }

    template<size_t p, size_t q>
    RationalFunction<T, std::max(n + q - 1, m + p - 1), m + q - 1>
    operator-(const RationalFunction<T, p, q> &other) const {
        Polynomial<T, std::max(n + q - 1, m + p - 1)> num;
        Polynomial<T, m + q - 1> den;
        den = denominator * other.denominator;
        num = numerator * other.denominator;
        num -= denominator * other.numerator;
        return {num, den};
    }

    template<size_t p, size_t q>
    RationalFunction<T, n + q - 1, m + p - 1> operator/(const RationalFunction<T, p, q> &other) const {
        Polynomial<T, n + q - 1> num;
        Polynomial<T, m + p - 1> den;
        num = numerator * other.denominator;
        den = denominator * other.numerator;
        return {num, den};
    }
};

template<typename T, size_t n, size_t m>
RationalFunction<T, n, m> operator*(const T &scalar, const RationalFunction<T, n, m> &rf) {
    Polynomial<T, n> num;
    Polynomial<T, m> den;
    for (int i = 0; i < n; i++)
        num[i] = scalar * rf.numerator_data(i);
    for (int i = 0; i < m; i++)
        den[i] = scalar * rf.denominator_data(i);
    return {num, den};
}

template<typename T, size_t n, size_t m, size_t p>
RationalFunction<T, n + p - 1, m> operator*(const Polynomial<T, p> &poly, const RationalFunction<T, n, m> &rf) {
    Polynomial<T, p + n - 1> num;
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < m; j++)
            num[i + j] += poly[j] * rf.numerator_data(i);
    }
    return {num, rf.denominator};
}
