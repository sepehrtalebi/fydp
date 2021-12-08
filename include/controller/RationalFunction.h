#pragma once
#include <iostream>
#include <vector>
#include "Polynomial.h"
#include <algorithm>

template<typename T>
static constexpr T abs_difference(const T& a, const T&b) {
    return a > b ? a - b : b - a;
}

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

    void print() {
        numerator.print();
        for (int i = 0; i < std::max(n, m); i++) {
            std::cout << "-----";
        }
        std::cout << std::endl;
        denominator.print();
    }

    T numerator_data(size_t num_index) {
        return numerator[num_index];
    }

    T denominator_data(size_t den_index) {
        return denominator[den_index];
    }

    template<size_t p>
    RationalFunction<T, (n - 1) * (p - 1) + 1, (m - 1) * (p - 1) + 1> _of_(const Polynomial<T, p> &p_of_x) {
        auto num = numerator._of_(p_of_x);
        auto den = denominator._of_(p_of_x);
        return {num, den};
    }

    template<size_t p, size_t q>
    RationalFunction<T, (n - 1) * std::max(p - 1, q - 1) + (q - 1) * heaviside_difference(m, n) + 1,
    (m - 1) * std::max(p - 1, q - 1) + (q - 1) * heaviside_difference(n, m) + 1>
    _of_(const RationalFunction<T, p, q> &g_of_x) {
        Polynomial<T, (n - 1) * std::max(p - 1, q - 1) + (q - 1) * heaviside_difference(m, n) + 1> num;
        Polynomial<T, (m - 1) * std::max(p - 1, q - 1) + (q - 1) * heaviside_difference(n, m) + 1> den;

        for (size_t i = 0; i < n; i++) { //this is how you add the rational functions in the numerator
            Polynomial<T, (n - 1) * (p - 1) + 1> num_of_num_expansion{g_of_x.numerator};
            Polynomial<T, (n - 1) * (q - 1) + 1> den_of_num_expansion; //edge n = 1
            if (n > 1) den_of_num_expansion = {g_of_x.denominator};
            else den_of_num_expansion.identity();
            num_of_num_expansion.pow(i);
            den_of_num_expansion.pow(n - 1 - i);
            auto hacky_product = Polynomial<T, std::max((n - 1) * (p - 1) + 1, (n - 1) * (q - 1) + 1)>::hacky_product(
                    num_of_num_expansion, den_of_num_expansion
                    ); //* usually creates a larger size polynomial based on sizes of the operands. We don't want this.
            num += numerator[i] * hacky_product;
        }

        for (size_t i = 0; i < m; i++) { //this is how you add the rational functions in the denominator
            Polynomial<T, (m - 1) * (p - 1) + 1> num_of_den_expansion{g_of_x.numerator};
            Polynomial<T, (m - 1) * (q - 1) + 1> den_of_den_expansion; //edge m = 1
            if (m > 1) den_of_den_expansion = {g_of_x.denominator};
            else den_of_den_expansion.identity();
            num_of_den_expansion.pow(i);
            den_of_den_expansion.pow(m - 1 - i);
            auto hacky_product = Polynomial<T, std::max((n - 1) * (p - 1) + 1, (n - 1) * (q - 1) + 1)>::hacky_product(
                    num_of_den_expansion, den_of_den_expansion
            ); //* usually creates a larger size polynomial based on sizes of the operands. We don't want this.
            den += denominator[i] * hacky_product;
        }

        if ((n > m) || (m > n)) { //because the denominators of the numerator and denominator are the same, they will cancel
            constexpr size_t abs_diff = abs_difference(m, n);
            Polynomial<T, abs_diff + 1> canceled_denominators{g_of_x.denominator};
            canceled_denominators.pow(abs_diff);
            if (m > n) num *= canceled_denominators;
            else den *= canceled_denominators;
        }
        return {num, den};
    }

    template<size_t p, size_t q>
    RationalFunction<T, n + p - 1, m + q - 1> operator*(const RationalFunction<T, p, q> &other) {
        auto num = numerator * other.numerator;
        auto den = denominator * other.denominator;
        return {num, den};
    }

    template<size_t p>
    RationalFunction<T, n + p - 1, m> operator*(const Polynomial<T, p> &poly) {
        auto num = numerator * poly;
        return {num, this->denominator};
    }

    RationalFunction<T, n, m> operator*(const T &scalar) {
        Polynomial<T, n> num;
        Polynomial<T, m> den;
        num = numerator * scalar;
        den = denominator * scalar;
        return {num, den};
    }

    template<size_t p, size_t q>
    RationalFunction<T, std::max(n + q - 1, m + p - 1), m + q - 1>
            operator+(const RationalFunction<T, p, q> &other) {
        Polynomial<T, std::max(n + q - 1, m + p - 1)> num;
        Polynomial<T, m + q - 1> den;
        den = denominator * other.denominator;
        num = numerator * other.denominator;
        num += denominator * other.numerator;
        return {num, den};
    }

    template<size_t p, size_t q>
    RationalFunction<T, std::max(n + q - 1, m + p - 1), m + q - 1>
    operator-(const RationalFunction<T, p, q> &other) {
        Polynomial<T, std::max(n + q - 1, m + p - 1)> num;
        Polynomial<T, m + q - 1> den;
        den = denominator * other.denominator;
        num = numerator * other.denominator;
        num -= denominator * other.numerator;
        return {num, den};
    }

    template<size_t p, size_t q>
    RationalFunction<T, n + q - 1, m + p - 1> operator/(const RationalFunction<T, p, q> &other) {
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
    num = rf.numerator * scalar;
    den = rf.denominator * scalar;
    return {num, den};
}

template<typename T, size_t n, size_t m, size_t p>
RationalFunction<T, n + p - 1, m> operator*(const Polynomial<T, p> &poly, const RationalFunction<T, n, m> &rf) {
    auto num = rf.numerator * poly;
    return {num, rf.denominator};
}