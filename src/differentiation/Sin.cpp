#include "Sin.h"
#include "Zero.h"
#include "Nan.h"

#include <cmath>

double Sin::call(const double &operand) const {
    return sin(operand);
}

ExprPtr Sin::call(const ExprPtr &operand) const {
    return sin(operand);
}

ExprPtr Sin::derivative(const ExprPtr &expr) const {
    return cos(expr);
}

std::string Sin::toStrWrapper(const std::string &operandString) const {
    return "sin(" + operandString + ")";
}

ExprPtr sin(const ExprPtr &expr) {
    if (!expr) return std::make_shared<Nan>();
    if (expr->isZero()) return std::make_shared<Zero>();
    return std::make_shared<Sin>(expr);
}
