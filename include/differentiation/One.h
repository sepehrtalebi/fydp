#pragma once

#include "Expression.h"

#include <map>
#include <memory>
#include <string>

class One : public Expression {
public:
    double evaluate(const std::map<std::string, double> &variables) const override;

    std::shared_ptr<Expression> diff(const std::string &identifier) const override;

    std::shared_ptr<Expression> subs(const std::map<std::string, std::shared_ptr<Expression>> &subs) const override;

    std::string toStr() const override;

    bool isOne() const override;
};
