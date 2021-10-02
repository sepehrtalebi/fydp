#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>

class One : public Expression {
public:
    double evaluate(const std::map<std::string, double> &variables) const override;

    ExprPtr diff(const std::string &identifier) const override;

    ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    ExprPtr simplify() const override;

    std::string toStr() const override;

    bool isOne() const override;
};
