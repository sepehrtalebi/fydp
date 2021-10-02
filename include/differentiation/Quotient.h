#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class Quotient : public Expression {
private:
    ExprPtr num;
    ExprPtr den;
public:
    Quotient(ExprPtr num, ExprPtr den) : num(std::move(num)), den(std::move(den)) {}

    double evaluate(const std::map<std::string, double> &variables) const override;

    ExprPtr diff(const std::string &id) const override;

    ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    ExprPtr simplify() const override;

    std::string toStr() const override;
};

ExprPtr operator/(const ExprPtr &expr1, const ExprPtr &expr2);

ExprPtr operator/(const ExprPtr &expr, const double &num);

ExprPtr operator/(const double &num, const ExprPtr &expr);

void operator/=(ExprPtr &expr1, const ExprPtr &expr2);

void operator/=(ExprPtr &expr, const double &num);

void operator/=(const double &num, ExprPtr &expr);