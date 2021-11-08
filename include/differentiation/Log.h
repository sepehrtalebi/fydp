#pragma once

#include "UnaryOperator.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Log : public UnaryOperator {
public:
    explicit Log(const ExprPtr &operand) : UnaryOperator(operand) {}

protected:
    [[nodiscard]] double call(const double &operand) const override;

    [[nodiscard]] ExprPtr call(const ExprPtr &operand) const override;

    [[nodiscard]] ExprPtr derivative(const ExprPtr &expr) const override;

    [[nodiscard]] std::string toStrWrapper(const std::string &operandString) const override;
};
