#pragma once
#include <iostream>
#include <array>
#include "RationalFunction.h"

template <typename T, size_t n, size_t m>
class TransferFunction: public RationalFunction<T, n, m> {
    Vector<T, std::max<size_t>(n, 0)> past_inputs;
    Vector<T, std::max<size_t>(m - 1, 0)> past_outputs;

    T next_output() {
        T next_output = 0;
        for (int i = 0; i < m - 1; i++) next_output += -this->denominator[i] / this->denominator[0] * past_outputs[m - 1 - i];
        for (int i = 0; i < n; i++) next_output += this->numerator / this->denominator[0] * past_inputs[n - i];
        return next_output;
    }

    template<size_t p>
    static Vector<T, p> reallocate_push(const Vector<T, p> &other, T val) { //deletes pth element, inserts 0th element
        Vector<T, p> copy;
        for (int i = 1; i < p; i++) copy[i] = other[i - 1];
        copy[0] = val;
        return copy;
    }

    template<typename, size_t, size_t>
    friend class TransferFunction;
public:
    TransferFunction() = default;

    TransferFunction(const Polynomial<T, n> &numerator, const Polynomial<T, m> &denominator):
            RationalFunction<T, n, m>(numerator, denominator), past_inputs(), past_outputs() {}

    using RationalFunction<T, n, m>::RationalFunction;

    TransferFunction(RationalFunction<T, n, m> rf): TransferFunction(rf.numerator, rf.denominator) {}  // NOLINT(google-explicit-constructor)

    TransferFunction& operator=(const TransferFunction<T, n, m> &other) {
        this->numerator = other.numerator;
        this->denominator = other.denominator;
        for (int i = 0; i < std::max<size_t>(n, 0); i++) this->past_inputs[i] = other.past_inputs[i];
        for (int i = 0; i < std::max<size_t>(m - 1, 0); i++) this->past_outputs[i] = other.past_outputs[i];
        return *this;
    }

    TransferFunction(const TransferFunction<T, n, m> &other) {
        *this = other;
    }

    template<size_t p, size_t q, size_t r, size_t s>
    static TransferFunction<T, p + r - 1, std::max(p+r - 1, q + s - 1)>
    feedbackLoop(TransferFunction<T, p, q> controller, TransferFunction<T, r, s> plant) {
        TransferFunction<T, p + r - 1, q + s - 1> CP = controller * plant;
        return {CP.numerator, CP.numerator + CP.denominator};
    }

    static TransferFunction<T, n, std::max(n, m)>
    feedbackLoop(const TransferFunction<T, n, m> &CP) {
        Polynomial<T, std::max(m, n)> den;
        den = CP.numerator + CP.denominator;
        return {CP.numerator, den};
    }

    template<size_t p, size_t q>
    TransferFunction<T, n + p - 1, std::max(n + p - 1, m + q - 1)>
    feedbackLoop(const TransferFunction<T, p, q> &other) const {
        TransferFunction<T, n + p - 1, m + q - 1> CP = (*this) * other;
        return {CP.numerator, CP.numerator + CP.denominator};
    }

    static TransferFunction<T, (n - 1) + heaviside_difference(m, n) + 1, (m - 1) + heaviside_difference(n, m) + 1>
            discretize(const TransferFunction<T, n, m> &tf, T dt) {
        //trapezoidal method, SCH looks hard
        RationalFunction<T, 2, 2> trapezoidal{Polynomial<T, 2>{2, -2}, Polynomial<T, 2>{dt, dt}};
        auto discretized = tf._of_(trapezoidal);
        return discretized;
    }

    TransferFunction<T, (n - 1) + heaviside_difference(m, n) + 1, (m - 1) + heaviside_difference(n, m) + 1>
            discretize(T dt) {
        //trapezoidal method, SCH looks hard
        RationalFunction<T, 2, 2> trapezoidal{Polynomial<T, 2>{2, -2}, Polynomial<T, 2>{dt, dt}};
        auto discretized = this->_of_(trapezoidal);
        return {discretized};
    }

    template<size_t output_size>
    Vector<T, output_size> step(const TransferFunction<T, n, m> &tf, T dt = 1E-5) {
        Vector<T, output_size> step_response;
        auto discrete = tf.discretize(dt);
        for (int i = 0; i < n; i++) past_inputs[i] = 1;
        for (int i = 0; i < output_size; i++) {
            step_response[i] = next_output();
            past_outputs = reallocate_push(past_outputs, step_response[i]);
            past_inputs = reallocate_push(past_inputs, past_inputs[0] - step_response[i]);
        }
        return step_response;
    }

    using RationalFunction<T, n, m>::operator*=;
};

//template<typename T, size_t p, size_t q, size_t r, size_t s>
//static TransferFunction<T, p + r - 1, std::max(p+r - 1, q + s - 1)>
//feedbackLoop(TransferFunction<T, p, q> controller, TransferFunction<T, r, s> plant) {
//    const TransferFunction<T, p + r - 1, q + s - 1> CP = controller * plant;
//    return {CP.numerator, CP.numerator + CP.denominator};
//}
//
//template<typename T, size_t p, size_t q>
//static TransferFunction<T, p, std::max(p, q)>
//feedbackLoop(const TransferFunction<T, p, q> &CP) {
//    Polynomial<T, std::max(p, q)> den;
//    den += CP.numerator + CP.denominator;
//    return {CP.numerator, den};
//}