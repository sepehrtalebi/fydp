#pragma once
#include <iostream>
#include <vector>
#include "RationalFunction.h"

template <typename T, size_t n, size_t m>
class TransferFunction: public RationalFunction<T, n, m> {
public:
    TransferFunction(const Polynomial<T, n> &numerator, const Polynomial<T, m> &denominator):
            RationalFunction<T, n, m>(numerator, denominator) {}

    TransferFunction(RationalFunction<T, n, m> rf): TransferFunction(rf.numerator, rf.denominator) {}

    template<size_t p, size_t q, size_t r, size_t s>
    static TransferFunction<T, p + r, std::max(p+r, q+s)> feedbackLoop(TransferFunction<T, p, q> controller, TransferFunction<T, r, s> plant) {
        TransferFunction<T, p + r, q + s> CP = controller * plant;
        return {CP.numerator, CP.numerator + CP.denominator};
    }

    TransferFunction<T, n, m> discretize(T dt) {
        //trapezoidal method, SCH looks hard
        RationalFunction<T, 2, 2> trapezoidal{std::vector<T>{2, -2}, std::vector<T>{dt, dt}};
        Polynomial<T, n * 2> num;


        }
    }
};
