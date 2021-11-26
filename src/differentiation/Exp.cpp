#include "Exp.h"
#include "Zero.h"
#include "One.h"
#include "Nan.h"

#include <cmath>

double Exp::call(const double &operand) const {
    return exp(operand);
}

ExprPtr Exp::call(const ExprPtr &operand) const {
    return exp(operand);
}

ExprPtr Exp::derivative(const ExprPtr &expr) const {
    return exp(expr);
}

std::string Exp::toStrWrapper(const std::string &operandString) const {
    return "exp(" + operandString + ")";
}

ExprPtr exp(const ExprPtr &expr) {
    if (!expr) return Nan::INSTANCE;
    if (expr == Zero::INSTANCE) return One::INSTANCE;
    return std::make_shared<Exp>(expr);
}
