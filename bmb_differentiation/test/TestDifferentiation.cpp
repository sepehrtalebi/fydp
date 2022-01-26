#include "TestDifferentiation.h"

#include <bmb_differentiation/runtime/Expression.h>
#include <bmb_differentiation/runtime/Variable.h>
#include <bmb_math/Vector.h>
#include <bmb_math/Vector3.h>
#include <bmb_math/Quaternion.h>
#include <bmb_math/Matrix.h>
#include <cassert>
#include <iostream>

void testDifferentiation() {
    ExprPtr x = Variable::make("x");
    ExprPtr test = 2 * (2 * x + 1);
    assert(test->nodeCount() == 7);
    assert(test->simplify()->nodeCount() == 5);

    Quaternion<ExprPtr> quat{Variable::make("q0"),
                             Variable::make("q1"),
                             Variable::make("q2"),
                             Variable::make("q3")};

    Vector3<ExprPtr> w{Variable::make("wx"),
                       Variable::make("wy"),
                       Variable::make("wz")};
    ExprPtr dt = Variable::make("dt");
    Vector<ExprPtr, 4> mat = quat.E().transpose() * quat.cong().toDCM() * w * (dt / 2);
    Matrix<ExprPtr, 4, 4> jac;
    for (size_t i = 0; i < 4; i++) for (size_t j = 0; j < 4; j++) {
        jac[i][j] = mat[i]->diff("q" + std::to_string(j));
        // std::cout << jac[i][j]->toStr() << std::endl;
    }

    std::cout << "Passed All Tests for Differentiation!\n";
}
