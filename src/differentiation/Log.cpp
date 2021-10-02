#include "Log.h"
#include "Zero.h"
#include "Nan.h"

#include <cmath>

double Log::call(const double &operand) const {
    return log(operand);
}

std::shared_ptr<Expression> Log::call(const std::shared_ptr<Expression> &operand) const {
    return log(operand);
}

std::shared_ptr<Expression> Log::derivative(const std::shared_ptr<Expression> &expr) const {
    return 1 / expr;
}

std::string Log::toStrWrapper(const std::string &operandString) const {
    return "ln(" + operandString + ")";
}

std::shared_ptr<Expression> log(const std::shared_ptr<Expression> &expr) {
    if (!expr || expr->isZero()) return std::make_shared<Nan>();
    if (expr->isOne()) return std::make_shared<Zero>();
    return std::make_shared<Log>(expr);
}
