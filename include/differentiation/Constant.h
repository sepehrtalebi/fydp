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

    std::shared_ptr<Expression> diff(const std::string &identifier) const override;

    std::shared_ptr<Expression> subs(const std::map<std::string, std::shared_ptr<Expression>> &subs) const override;

    std::shared_ptr<Expression> simplify() const override;

    std::string toStr() const override;

    double getValue() const;
};
