#pragma once

#include <bmb_math/RationalFunction.h>
#include <bmb_utilities/MathUtils.h>

#include <array>
#include <cassert>
#include <iostream>

template <typename T, size_t n, size_t m>
class TransferFunction: public RationalFunction<T, n, m> {
    static_assert(m > 0);

    Vector<T, m> past_inputs;
    Vector<T, std::max<size_t>(m - 1, 0)> past_outputs;
    bool discretized;

    T next_output(T input) {
        assert(this->discretized);
        T next_output = 0;
        past_inputs = reallocate_push(past_inputs, input);
        if (m > 1)
            for (int i = 0; i < m - 1; i++) next_output -= this->denominator[i] * past_outputs[m - 2 - i];
        for (int i = 0; i < m && i < n; i++) next_output += this->numerator[i] * past_inputs[m - 1 - i];
        next_output /= this->denominator[m - 1];
        past_outputs = reallocate_push(past_outputs, next_output);
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
    template<typename>
    friend class PID;
public:
    TransferFunction() = default;

    TransferFunction(const Polynomial<T, n> &numerator, const Polynomial<T, m> &denominator, bool discrete = false):
            RationalFunction<T, n, m>(numerator, denominator), past_inputs(), past_outputs(), discretized(discrete) {
    }

    using RationalFunction<T, n, m>::RationalFunction;

    TransferFunction(RationalFunction<T, n, m> rf):
        TransferFunction(rf.numerator, rf.denominator) {}  // NOLINT(google-explicit-constructor)

    TransferFunction& operator=(const TransferFunction<T, n, m> &other) {
        this->numerator = other.numerator;
        this->denominator = other.denominator;
        this->discretized = other.discretized;
        for (int i = 0; i < std::max<size_t>(n, 0); i++) this->past_inputs[i] = other.past_inputs[i];
        for (int i = 0; i < std::max<size_t>(m - 1, 0); i++) this->past_outputs[i] = other.past_outputs[i];
        return *this;
    }

    TransferFunction(const TransferFunction<T, n, m> &other) {
        *this = other;
    }

    void print() {
        if (this->discretized) RationalFunction<T, n, m>::print('z');
        else RationalFunction<T, n, m>::print();
    }

    template<size_t p, size_t q>
    TransferFunction<T, n + p - 1, std::max(n + p - 1, m + q - 1)>
    feedbackLoop(const TransferFunction<T, p, q> &other) const {
        TransferFunction<T, n + p - 1, m + q - 1> CP = (*this) * other;
        return {CP.numerator, CP.numerator + CP.denominator, this->discretized};
    }

    TransferFunction<T, n, std::max(n, m)>
    feedbackLoop() const {
        Polynomial<T, std::max(m, n)> den;
        den = this->numerator + this->denominator;
        return {this->numerator, den, this->discretized};
    }

    TransferFunction<T, std::max(m, n), std::max(n, m)>
            discretize(T dt = 1E-4) const {
        //trapezoidal method, SCH looks hard
        static constexpr size_t p = (n - 1) + bmb_utilities::heaviside_difference(m, n) + 1;
        static constexpr size_t q = (m - 1) + bmb_utilities::heaviside_difference(n, m) + 1;
        RationalFunction<T, 2, 2> trapezoidal{Polynomial<T, 2>{-2, 2}, Polynomial<T, 2>{dt, dt}};
        TransferFunction<T, p, q> discrete = this->_of_(trapezoidal);
        discrete.discretized = true;
        for (size_t i = 0; i < p; i++) discrete.numerator[i] /= discrete.denominator[q - 1];
        for (size_t i = 0; i < q; i++) discrete.denominator[i] /= discrete.denominator[q - 1];
        discrete.denominator[q - 1] = 1;
        return {discrete};
    }

    template<size_t output_size>
    Vector<T, output_size> step(T dt = 1E-4) {
        Vector<T, output_size> step_response;
        if (this->discretized) {
            assert(m >= n);
            for (int i = 0; i < output_size; i++) {
                step_response[i] = next_output(1);
            }
        }
        else {
            auto discrete = this->discretize(dt);
            for (int i = 0; i < output_size; i++) {
                step_response[i] = discrete.next_output(1);
            }
        }
        return step_response;
    }

    using RationalFunction<T, n, m>::operator*=;
};