#include "TestTransferFunction.h"
#include "TransferFunction.h"
#include <cassert>
#include "../venv/matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;

const double EPSILON = 1E-5;

template<typename T>
constexpr bool AreSame(T a, T b)
{
    return abs_difference(a, b) < EPSILON;
}

void testTransferFunction() {
    TransferFunction<double, 1, 3> P{4, 1, 2, 3};
    TransferFunction<double, 4, 2> C{3, 4, 5, 1, 4, 1};
    TransferFunction<double, 4, 4> CP = C*P;
    CP.print();
    TransferFunction<double, 4, 4> truth{12, 16, 20, 4, 4, 9, 14, 3};
    for (int i = 0; i < 4; i++)
        assert(CP.numerator_data(i) == truth.numerator_data(i));
    for (int i = 0; i < 4; i++)
        assert(CP.denominator_data(i) == truth.denominator_data(i));
    auto FBL = C.feedbackLoop(P);
    FBL.print();
    TransferFunction<double, 4, 4> truth1 = {12, 16, 20, 4, 16, 25, 34, 7};
    for (int i = 0; i < 2; i++)
        assert(FBL.numerator_data(i) == truth1.numerator_data(i));
    for (int i = 0; i < 4; i++)
        assert(FBL.denominator_data(i) == truth1.denominator_data(i));
    auto P_D = P.discretize(1e-3);
    P_D.print();
    TransferFunction<double, 3, 3> truth2{4e-6, 8e-6, 4e-6, 12-4e-3+1e-6, -24+2e-6, 12+4e-3+1e-6};
    for (int i = 0; i < 2; i++)
        assert(AreSame(P_D.numerator_data(i), truth2.numerator_data(i)));
    for (int i = 0; i < 2; i++)
        assert(AreSame(P_D.denominator_data(i), truth2.denominator_data(i)));
    std::array<double, 10000> output = FBL.step<10000>();
    Vector<double, 10000> t;
    t.range(0, 10);
    std::array<double, 10000> x = t.data_to_array();
    plt::plot(x, output);
};