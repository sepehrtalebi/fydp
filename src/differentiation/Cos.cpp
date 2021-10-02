#include "Cos.h"
#include "One.h"
#include "Nan.h"

#include <cmath>

double Cos::call(const double &operand) const {
    return cos(operand);
}

ExprPtr Cos::call(const ExprPtr &operand) const {
    return cos(operand);
}

ExprPtr Cos::derivative(const ExprPtr &expr) const {
    return -sin(expr);
}

std::string Cos::toStrWrapper(const std::string &operandString) const {
    return "cos(" + operandString + ")";
}

ExprPtr cos(const ExprPtr &expr) {
    if (!expr) return std::make_shared<Nan>();
    if (expr->isZero()) return std::make_shared<One>();
    return std::make_shared<Cos>(expr);
}
