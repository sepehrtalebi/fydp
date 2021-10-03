#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>

class Constant;

typedef std::shared_ptr<Constant> ConstPtr;

class Constant : public Expression {
public:
    Constant() = delete;

    static ConstPtr make(const double &value);

    double evaluate(const std::map<std::string, double> &variables) const override;

    ExprPtr diff(const std::string &identifier) const override;

    ExprPtr subs(const std::map<std::string, ExprPtr> &subs) const override;

    ExprPtr simplify() const override;

    std::string toStr() const override;

    double getValue() const;

protected:
    explicit Constant(const double &value) : value(value) {}

private:
    double value;
};
