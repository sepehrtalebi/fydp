#include "TestDifferentiation.h"

#include "Expression.h"
#include "Variable.h"
#include "Vector.h"
#include "Vector3.h"
#include "Quaternion.h"
#include "Matrix.h"
#include <iostream>

void testDifferentiation() {
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
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) {
        jac[i][j] = mat[i]->diff("q" + std::to_string(j));
        std::cout << jac[i][j]->toStr() << std::endl;
    }

    std::cout << "Passed All Tests for Differentiation!" << std::endl;
}
