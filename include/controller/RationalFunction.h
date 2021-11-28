#pragma once
#include <iostream>
#include <vector>
#include "Polynomial.h"

template <typename T, size_t n, size_t m>
class RationalFunction {
protected:
    Polynomial<T, n> numerator;
    Polynomial<T, m> denominator;
public:
    RationalFunction(const Polynomial<T, n> &numerator, const Polynomial<T, m> &denominator){
        this->numerator = numerator;
        this->denominator = denominator;
    }

    RationalFunction(const RationalFunction<T, n, m> &other) {
        this->numerator = other.numerator;
        this->denominator = other.denominator;
    }

    void print() {
        numerator.print();
        for (int i = 0; i < std::max(numerator.get_size(), denominator.get_size()); i++) {
            std::cout << " - ";
        }
        std::cout << std::endl;
        denominator.print();
    }

    template<size_t p>
    auto _of_(const Polynomial<T, p> &p_of_x) {
        auto num = numerator._of_(p_of_x);
        auto den = denominator._of_(p_of_x);
        return RationalFunction{num, den};
    }

    template<size_t p, size_t q>
    RationalFunction<T, n * p, m * p> _of_(const RationalFunction<T, p, q> &x) {
        RationalFunction<T, n * p, n * q> num;
        RationalFunction<T, n, m> den;
        return;
    }

    template<size_t p, size_t q>
    auto operator*(const RationalFunction<T, p, q> &other) {
        auto num = numerator * other.numerator;
        auto den = denominator * other.denominator;
        return RationalFunction{num, den};
    }

    template<size_t p>
    auto operator*(const Polynomial<T, p> &other) {
        auto num = numerator * other.numerator;
        return RationalFunction{num, this->denominator};
    }

    RationalFunction<T, n, m> operator*(const T &scalar) {
        Polynomial<T, n> num;
        Polynomial<T, m> den;
        num = numerator * scalar;
        den = denominator * scalar;
        return {num, den};
    }

    template<size_t p, size_t q>
    auto operator+(const RationalFunction<T, p, q> &other) {
        Polynomial<T, std::max(n + q - 1, m + p - 1)> num;
        Polynomial<T, m + q - 1> den;
        den = denominator * other.denominator;
        num = numerator * other.denominator;
        num += denominator * other.numerator;
        return RationalFunction{num, den};
    }

    template<size_t p, size_t q>
    auto operator-(const RationalFunction<T, p, q> &other) {
        Polynomial<T, std::max(n + q - 1, m + p - 1)> num;
        Polynomial<T, m + q - 1> den;
        den = denominator * other.denominator;
        num = numerator * other.denominator;
        num -= denominator * other.numerator;
        return RationalFunction{num, den};
    }

    template<size_t p, size_t q>
    auto operator/(const RationalFunction<T, p, q> &other) {
        Polynomial<T, n + q - 1> num;
        Polynomial<T, m + p - 1> den;
        num = numerator * other.denominator;
        den = denominator * other.numerator;
        return RationalFunction{num, den};
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
auto operator*(const Polynomial<T, p> &poly, const RationalFunction<T, n, m> &rf) {
    auto num = rf.numerator * poly.numerator;
    return RationalFunction{num, rf.denominator};
}