#pragma once

#include "UnaryOperator.h"

#include <string>

class Exp : public UnaryOperator {
public:
    explicit Exp(const ExprPtr &operand) : UnaryOperator(operand) {}

protected:
    [[nodiscard]] double call(const double &operand) const override;

    [[nodiscard]] ExprPtr call(const ExprPtr &operand) const override;

    [[nodiscard]] ExprPtr derivative(const ExprPtr &expr) const override;

    [[nodiscard]] std::string toStrWrapper(const std::string &operandString) const override;
};
