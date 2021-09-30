#include "TestDifferentiation.h"
#include "Expression.h"
#include "Variable.h"
#include "Constant.h"
#include "Quaternion.h"
#include "Matrix.h"

void testDifferentiation() {
    Constant<double> c = Constant<double>{5.6};

    Variable<double> q0 = Variable<double>{"q0"};
    Quaternion<Expression<double>> quat{Variable<double>{"q0"},
                                        Variable<double>{"q1"},
                                        Variable<double>{"q2"},
                                        Variable<double>{"q3"}};
    Matrix<Expression<double>, 4, 3> mat = quat.E().transpose() * quat.cong().toDCM();
}
