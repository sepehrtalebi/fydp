#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class UnaryOperator : public Expression {
public:
    UnaryOperator() = delete;

    explicit UnaryOperator(ExprPtr operand) : operand(std::move(operand)) {}

    double evaluate(const std::map<std::string, double> &variables) const override;

    ExprPtr diff(const std::string &id) const override;

    ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    ExprPtr simplify() const override;

    std::string toStr() const override;

    int nodeCount() const override;

protected:
    virtual double call(const double &operand) const = 0;

    virtual ExprPtr call(const ExprPtr &operand) const = 0;

    virtual ExprPtr derivative(const ExprPtr &expr) const = 0;

    virtual std::string toStrWrapper(const std::string &operandString) const = 0;

private:
    ExprPtr operand;
};

#include "Log.h"
#include "Sin.h"
#include "Cos.h"
