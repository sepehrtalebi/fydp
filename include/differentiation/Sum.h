#pragma once

#include "Expression.h"
#include "ExpressionPointer.h"

class Sum : public Expression {
private:
    Expression *left;
    Expression *right;
public:
    Sum(Expression *left, Expression *right) : left(left), right(right) {}

    ~Sum() override;

    double evaluate(const std::map<std::string, double> &variables) const override;

    Expression *diff(const std::string &id) const override;

    std::string toStr() const override;

    Expression *copy() const override;
};
