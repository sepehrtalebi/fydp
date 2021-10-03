#include "TestDifferentiation.h"
#include "Variable.h"
#include "Quaternion.h"
#include "Matrix.h"
#include "Vector3.h"
#include "Vector.h"
#include <iostream>

void testDifferentiation() {
    Quaternion<ExprPtr> quat{std::make_shared<Variable>("q0"),
                                                 std::make_shared<Variable>("q1"),
                                                 std::make_shared<Variable>("q2"),
                                                 std::make_shared<Variable>("q3")};

    Vector3<ExprPtr> w{std::make_shared<Variable>("wx"),
                                           std::make_shared<Variable>("wy"),
                                           std::make_shared<Variable>("wz")};
    ExprPtr dt = std::make_shared<Variable>("dt");
    Vector<ExprPtr, 4> mat = quat.E().transpose() * quat.cong().toDCM() * w * (dt / 2);
    Matrix<ExprPtr, 4, 4> jac;
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) {
        jac[i][j] = mat[i]->diff("q" + std::to_string(j));
        std::cout << jac[i][j]->toStr() << std::endl;
    }

    std::cout << "Passed All Tests for Differentiation!" << std::endl;
}
