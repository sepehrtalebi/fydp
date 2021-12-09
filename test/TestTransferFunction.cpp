#include "TestTransferFunction.h"
#include "TransferFunction.h"
#include <cassert>

const double EPSILON = 1E-4;

template<typename T>
bool AreSame(T a, T b)
{
    return std::fabs(a - b) < EPSILON;
}

void testTransferFunction() {
    TransferFunction<double, 1, 3> P{2, 3, 1, 0};
    TransferFunction<double, 2, 2> C{2, 69 * 42.0, 1, 38};
    TransferFunction<double, 2, 4> CP = C*P;
    auto FBL = TransferFunction<double, 2, 4>::feedbackLoop(CP);
    FBL.print();
    TransferFunction<double, 5, 7> truth = {0, 220248, 66692, 17848, 12, 0, 220248, 668136, 26588, 13465, 690, 9};
    for (int i = 0; i < 5; i++)
        assert(FBL.numerator_data(i) == truth.numerator_data(i));
    for (int i = 0; i < 7; i++)
        assert(FBL.denominator_data(i) == truth.denominator_data(i));
    auto C_D = C.discretize(1E-3);
    C_D.print();
    TransferFunction<double, 2, 2> truth1{2, 0.8436, 1, -0.9627};
    for (int i = 0; i < 2; i++)
        assert(AreSame(FBL.numerator_data(i), truth1.numerator_data(i)));
    for (int i = 0; i < 2; i++)
        assert(AreSame(FBL.denominator_data(i), truth1.denominator_data(i)));

};