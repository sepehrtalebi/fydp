#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Power : public Expression {
private:
    ExprPtr base;
    ExprPtr power;
public:
    Power(ExprPtr base, ExprPtr power) : base(std::move(base)), power(std::move(power)) {}

    double evaluate(const std::map<std::string, double> &variables) const override;

    ExprPtr diff(const std::string &id) const override;

    ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    ExprPtr simplify() const override;

    std::string toStr() const override;
};

ExprPtr pow(const ExprPtr &expr1, const ExprPtr &expr2);

ExprPtr pow(const ExprPtr &expr, const double &num);

ExprPtr pow(const double &num, const ExprPtr &expr);

ExprPtr sqrt(const ExprPtr &expr);
