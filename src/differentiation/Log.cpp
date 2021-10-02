#include "Log.h"
#include "Zero.h"
#include "Nan.h"

#include <cmath>

double Log::call(const double &operand) const {
    return log(operand);
}

ExprPtr Log::call(const ExprPtr &operand) const {
    return log(operand);
}

ExprPtr Log::derivative(const ExprPtr &expr) const {
    return 1 / expr;
}

std::string Log::toStrWrapper(const std::string &operandString) const {
    return "ln(" + operandString + ")";
}

ExprPtr log(const ExprPtr &expr) {
    if (!expr || expr->isZero()) return std::make_shared<Nan>();
    if (expr->isOne()) return std::make_shared<Zero>();
    return std::make_shared<Log>(expr);
}
