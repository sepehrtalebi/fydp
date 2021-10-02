#include "Cos.h"
#include "One.h"
#include "Nan.h"

#include <cmath>

double Cos::call(const double &operand) const {
    return cos(operand);
}

std::shared_ptr<Expression> Cos::call(const std::shared_ptr<Expression> &operand) const {
    return cos(operand);
}

std::shared_ptr<Expression> Cos::derivative(const std::shared_ptr<Expression> &expr) const {
    return -sin(expr);
}

std::string Cos::toStrWrapper(const std::string &operandString) const {
    return "cos(" + operandString + ")";
}

std::shared_ptr<Expression> cos(const std::shared_ptr<Expression> &expr) {
    if (!expr) return std::make_shared<Nan>();
    if (expr->isZero()) return std::make_shared<One>();
    return std::make_shared<Cos>(expr);
}
