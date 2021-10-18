#pragma once

#include "BinaryOperator.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Power : public BinaryOperator {
public:
    Power(ExprPtr first, ExprPtr second) : BinaryOperator(std::move(first), std::move(second)) {}

    [[nodiscard]] ExprPtr diff(const std::string &id) const override;

    [[nodiscard]] std::string toStr() const override;

protected:
    [[nodiscard]] double call(const double &first, const double &second) const override;

    [[nodiscard]] ExprPtr call(const ExprPtr &first, const ExprPtr &second) const override;

    [[nodiscard]] std::string type() const override;
};

ExprPtr pow(const ExprPtr &expr1, const ExprPtr &expr2);

ExprPtr pow(const ExprPtr &expr, const double &num);

ExprPtr pow(const double &num, const ExprPtr &expr);

ExprPtr sqrt(const ExprPtr &expr);
