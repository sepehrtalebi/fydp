#include "TestDifferentiation.h"
#include "Constant.h"
#include "Variable.h"
#include "Sum.h"
#include "Vector.h"
#include <iostream>

void testDifferentiation() {
    ExpressionPointer a = ExpressionPointer{new Variable("a")};
    ExpressionPointer b = ExpressionPointer{new Variable("b")};

//    ExpressionPointer no{};
//    ExpressionPointer c = a + b;
//
//    ExpressionPointer p = no + c;
//    std::cout << p.toStr() << std::endl;
//    std::cout << c.toStr() << std::endl;
//    std::cout << c.evaluate({{"a", 5}, {"b", 7}}) << std::endl;
//    std::cout << c.diff("a").toStr() << std::endl;

    Vector<ExpressionPointer, 2> vec{a, b};
    std::cout << "test" << std::endl;
    vec = vec + vec;
    std::cout << vec[0].toStr() << std::endl;
}
