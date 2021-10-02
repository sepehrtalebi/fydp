#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

class BinaryOperator : public Expression {
public:
    BinaryOperator() = delete;

    explicit BinaryOperator(ExprPtr first, ExprPtr second) : first(std::move(first)), second(std::move(second)) {}

    double evaluate(const std::map<std::string, double> &variables) const override;

    ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    ExprPtr simplify() const override;

    int nodeCount() const override;

protected:
    ExprPtr first;
    ExprPtr second;

    virtual double call(const double &first, const double &second) const = 0;

    virtual ExprPtr call(const ExprPtr &first, const ExprPtr &second) const = 0;

    virtual std::string type() const = 0;

    virtual bool isAssociative() const;

    virtual bool isCommutative() const;

    virtual bool isDistributiveOn(const std::string &type) const;
};

#include "Sum.h"
#include "Difference.h"
#include "Product.h"
#include "Quotient.h"
#include "Power.h"
