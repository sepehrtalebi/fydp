#pragma once

#include "BinaryOperator.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Power : public BinaryOperator {
public:
    Power(ExprPtr first, ExprPtr second) : BinaryOperator(std::move(first), std::move(second)) {}

    ExprPtr diff(const std::string &id) const override;

    std::string toStr() const override;

protected:
    double call(const double &first, const double &second) const override;

    ExprPtr call(const ExprPtr &first, const ExprPtr &second) const override;

    std::string type() const override;
};

ExprPtr pow(const ExprPtr &expr1, const ExprPtr &expr2);

ExprPtr pow(const ExprPtr &expr, const double &num);

ExprPtr pow(const double &num, const ExprPtr &expr);

ExprPtr sqrt(const ExprPtr &expr);
