#pragma once

#include "UnaryOperator.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Cos : public UnaryOperator {
public:
    explicit Cos(const ExprPtr &operand) : UnaryOperator(operand) {}

protected:
    double call(const double &operand) const override;

    ExprPtr call(const ExprPtr &operand) const override;

    ExprPtr derivative(const ExprPtr &expr) const override;

    std::string toStrWrapper(const std::string &operandString) const override;
};

ExprPtr cos(const ExprPtr &expr);
