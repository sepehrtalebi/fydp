#include "Sin.h"
#include "Zero.h"
#include "Nan.h"

#include <cmath>

double Sin::call(const double &operand) const {
    return sin(operand);
}

std::shared_ptr<Expression> Sin::call(const std::shared_ptr<Expression> &operand) const {
    return sin(operand);
}

std::shared_ptr<Expression> Sin::derivative(const std::shared_ptr<Expression> &expr) const {
    return cos(expr);
}

std::string Sin::toStrWrapper(const std::string &operandString) const {
    return "sin(" + operandString + ")";
}

std::shared_ptr<Expression> sin(const std::shared_ptr<Expression> &expr) {
    if (!expr) return std::make_shared<Nan>();
    if (expr->isZero()) return std::make_shared<Zero>();
    return std::make_shared<Sin>(expr);
}
