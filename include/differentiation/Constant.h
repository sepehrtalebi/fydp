#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>

class Constant : public Expression {
private:
    double value;

public:
    explicit Constant(const double &value) : value(value) {}

    double evaluate(const std::map<std::string, double> &variables) const override;

    ExprPtr diff(const std::string &identifier) const override;

    ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    ExprPtr simplify() const override;

    std::string toStr() const override;

    double getValue() const;
};
