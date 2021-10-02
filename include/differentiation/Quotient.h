#pragma once

#include "BinaryOperator.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Quotient : public BinaryOperator {
public:
    Quotient(ExprPtr first, ExprPtr second) : BinaryOperator(std::move(first), std::move(second)) {}

    ExprPtr diff(const std::string &id) const override;

    ExprPtr simplify() const override;

    std::string toStr() const override;

protected:
    double call(const double &first, const double &second) const override;

    ExprPtr call(const ExprPtr &first, const ExprPtr &second) const override;

    std::string type() const override;
};

ExprPtr operator/(const ExprPtr &expr1, const ExprPtr &expr2);

ExprPtr operator/(const ExprPtr &expr, const double &num);

ExprPtr operator/(const double &num, const ExprPtr &expr);

void operator/=(ExprPtr &expr1, const ExprPtr &expr2);

void operator/=(ExprPtr &expr, const double &num);

void operator/=(const double &num, ExprPtr &expr);