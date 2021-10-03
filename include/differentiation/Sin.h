#pragma once

#include "UnaryOperator.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Sin : public UnaryOperator {
public:
    explicit Sin(const ExprPtr &operand) : UnaryOperator(operand) {}

protected:
    double call(const double &operand) const override;

    ExprPtr call(const ExprPtr &operand) const override;

    ExprPtr derivative(const ExprPtr &expr) const override;

    std::string toStrWrapper(const std::string &operandString) const override;
};

ExprPtr sin(const ExprPtr &expr);
