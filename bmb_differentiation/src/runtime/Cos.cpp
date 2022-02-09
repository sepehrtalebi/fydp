#include "bmb_differentiation/runtime/Cos.h"

#include <bmb_differentiation/runtime/Zero.h>
#include <bmb_differentiation/runtime/One.h>
#include <bmb_differentiation/runtime/Nan.h>

#include <cmath>
#include <string>
#include <memory>

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
    if (!expr) return Nan::INSTANCE;
    if (expr == Zero::INSTANCE) return One::INSTANCE;
    return std::make_shared<Cos>(expr);
}
